/**
 * lidsled - Lit Sled Brake Light System
 * Arduino Nano + MPU6050 IMU + WS2812 LED Strip
 *
 * A sled brake light with pitch-reactive rainbow side lights.
 *
 * Features:
 *   - IMU-based brake detection (foot braking)
 *   - Pitch-controlled rainbow animation on side LEDs
 *   - Automatic brake light on rear section
 *
 * Wiring:
 *   MPU6050:
 *     VCC -> 5V
 *     GND -> GND
 *     SCL -> A5
 *     SDA -> A4
 *     INT -> D2 (optional, for interrupt-driven reads)
 *
 *   LED Strip (WS2812/NeoPixel):
 *     VCC -> 5V (external power for many LEDs!)
 *     GND -> GND (shared with Arduino)
 *     DIN -> D6
 */

#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <FastLED.h>

// ============== LED Configuration ==============
#define LED_PIN       6
#define LED_TYPE      WS2812B
#define COLOR_ORDER   GRB
#define BRIGHTNESS    128

// LED Strip: 2m WS2812B @ 33 LEDs/m = 66 LEDs total
// Physical wiring order: [LEFT SIDE] → [BACK] → [RIGHT SIDE]
// Adjust the order below to match your actual wiring!

constexpr uint8_t NUM_LEDS = 66;

// Section definitions (indices are inclusive)
// Wiring assumption: Left(0-25) → Back(26-38) → Right(39-65)
// Change these if your wiring order is different!

constexpr uint8_t SECTION_LEFT_START  = 0;
constexpr uint8_t SECTION_LEFT_END    = 25;   // 26 LEDs (80cm)
constexpr uint8_t SECTION_LEFT_COUNT  = SECTION_LEFT_END - SECTION_LEFT_START + 1;

constexpr uint8_t SECTION_BACK_START  = 26;
constexpr uint8_t SECTION_BACK_END    = 38;   // 13 LEDs (40cm) - BRAKE LIGHT
constexpr uint8_t SECTION_BACK_COUNT  = SECTION_BACK_END - SECTION_BACK_START + 1;

constexpr uint8_t SECTION_RIGHT_START = 39;
constexpr uint8_t SECTION_RIGHT_END   = 65;   // 27 LEDs (80cm)
constexpr uint8_t SECTION_RIGHT_COUNT = SECTION_RIGHT_END - SECTION_RIGHT_START + 1;

// ============== MPU6050 Configuration ==============
#define INTERRUPT_PIN 2  // D2 for interrupt (optional)
constexpr float G = 9.81f;  // m/s²

// ============== Brake Detection Configuration ==============
// Filter settings
constexpr float EMA_ALPHA = 0.15f;          // EMA smoothing factor (0.1=smooth, 0.3=responsive)
constexpr float MEDIAN_WINDOW = 5;           // Median filter window size (odd number)

// Threshold with hysteresis (in m/s²)
constexpr float BRAKE_THRESHOLD_ON  = -1.2f; // Trigger braking when accel drops below this
constexpr float BRAKE_THRESHOLD_OFF = -0.3f; // Release braking when accel rises above this

// Timing requirements
constexpr unsigned long BRAKE_MIN_DURATION_MS = 150;  // Minimum duration to confirm brake
constexpr unsigned long BRAKE_DEBOUNCE_MS     = 50;   // Debounce time after release

// ============== Globals ==============
CRGB leds[NUM_LEDS];
MPU6050 mpu;

// MPU6050 DMP variables
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// Orientation and motion data
Quaternion q;           // Quaternion container
VectorInt16 aa;         // Raw accelerometer readings
VectorInt16 aaReal;     // Gravity-removed acceleration (body frame)
VectorFloat gravity;    // Gravity vector
float ypr[3];           // Yaw, Pitch, Roll in radians

// ============== Sensor Data (use these in your application) ==============
struct IMUData {
    // Angles in degrees (from DMP sensor fusion)
    float roll;
    float pitch;
    float yaw;

    // Body-frame linear acceleration in m/s² (gravity removed by DMP)
    float accelX;  // Forward (along sled direction) - RAW
    float accelY;  // Lateral (left/right)
    float accelZ;  // Vertical (up/down relative to sled)

    // Filtered forward acceleration (use this for brake detection)
    float accelForwardFiltered;

    // Brake detection state
    bool isBraking;           // True when confirmed braking
    bool brakePending;        // True when threshold crossed but not yet confirmed
} imuData;

// ============== Brake Detection State Machine ==============
enum class BrakeState : uint8_t {
    IDLE,       // No braking, waiting for threshold
    PENDING,    // Threshold crossed, waiting for duration confirmation
    BRAKING,    // Confirmed braking event
    RELEASING   // Brake released, waiting for debounce
};

struct BrakeDetector {
    // Current state
    BrakeState state = BrakeState::IDLE;

    // Median filter buffer
    float medianBuffer[5] = {0};
    uint8_t medianIndex = 0;

    // EMA state
    float emaValue = 0.0f;
    bool emaInitialized = false;

    // Timing (state entry timestamps)
    unsigned long stateEntryTime = 0;
} brakeDetector;

// Interrupt detection
volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

// ============== Function Declarations ==============
bool initMPU6050();
bool updateIMU();
void updateBrakeDetection(float rawAccelX);
float applyMedianFilter(float newValue);
float applyEMAFilter(float newValue);
void updateLEDs();
void updateSideRainbow();
void showBrakeLight();
void showSidePatternLeft();
void showSidePatternRight();
void showIdlePattern();
void fillSection(uint8_t start, uint8_t end, CRGB color);
void printIMUData();

// ============== Setup ==============
void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    while (!Serial);  // Wait for serial on Leonardo/Micro
    Serial.println(F("Project Rodel Racelight - IMU Data Logger"));
    Serial.println(F("=========================================="));

    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000);  // 400kHz I2C clock

    // Initialize MPU6050 with DMP
    if (!initMPU6050()) {
        Serial.println(F("MPU6050 initialization failed!"));
        // Blink onboard LED to indicate error
        pinMode(LED_BUILTIN, OUTPUT);
        while (1) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(100);
            digitalWrite(LED_BUILTIN, LOW);
            delay(100);
        }
    }

    // Initialize FastLED
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS)
           .setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(BRIGHTNESS);
    FastLED.clear();
    FastLED.show();

    Serial.println(F("\nSystem ready. Streaming IMU data...\n"));
    Serial.println(F("Roll\tPitch\tAccX\tFiltered\tBrake"));
}

// ============== Main Loop ==============
void loop() {
    // Check if DMP data is available
    if (updateIMU()) {
        printIMUData();
    }

    // Update LED display based on brake state
    updateLEDs();
}

// ============== MPU6050 DMP Initialization ==============
bool initMPU6050() {
    Serial.println(F("Initializing MPU6050..."));
    mpu.initialize();

    // Verify connection
    pinMode(INTERRUPT_PIN, INPUT);
    Serial.print(F("Testing MPU6050 connection... "));
    if (!mpu.testConnection()) {
        Serial.println(F("FAILED"));
        return false;
    }
    Serial.println(F("OK"));

    // Initialize DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // Supply your own gyro offsets here (or use calibration)
    // These are example values - run calibration for best results
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) {
        // Calibration
        Serial.println(F("Calibrating... Keep device still!"));
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();

        // Enable DMP
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // Enable interrupt
        Serial.print(F("Enabling interrupt on pin "));
        Serial.println(INTERRUPT_PIN);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();

        Serial.println(F("DMP ready!"));
        return true;
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        return false;
    }
}

// ============== IMU Update ==============
bool updateIMU() {
    if (!dmpReady) return false;

    // Check for new data (interrupt or polling)
    if (!mpuInterrupt && fifoCount < packetSize) {
        fifoCount = mpu.getFIFOCount();
        if (fifoCount < packetSize) return false;
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    // Check for overflow
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
        return false;
    }

    // Check for DMP data ready
    if (mpuIntStatus & 0x02) {
        // Wait for complete packet
        while (fifoCount < packetSize) {
            fifoCount = mpu.getFIFOCount();
        }

        // Read packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        // Get quaternion
        mpu.dmpGetQuaternion(&q, fifoBuffer);

        // Get gravity vector
        mpu.dmpGetGravity(&gravity, &q);

        // Get Yaw/Pitch/Roll angles from DMP (onboard EKF)
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Convert to degrees and store
        imuData.yaw   = ypr[0] * 180.0f / M_PI;
        imuData.pitch = ypr[1] * 180.0f / M_PI;
        imuData.roll  = ypr[2] * 180.0f / M_PI;

        // Get raw acceleration
        mpu.dmpGetAccel(&aa, fifoBuffer);

        // Get linear acceleration (gravity removed by DMP)
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

        // Convert to m/s²
        // DMP FIFO uses 8192 LSB per g (as per library source)
        const float LSB_TO_MS2 = G / 8192.0f;
        imuData.accelX = aaReal.x * LSB_TO_MS2;
        imuData.accelY = aaReal.y * LSB_TO_MS2;
        imuData.accelZ = aaReal.z * LSB_TO_MS2;

        // Apply filtering and brake detection
        updateBrakeDetection(imuData.accelX);

        return true;
    }

    return false;
}

// ============== Median Filter ==============
// Removes spike noise (bumps, impacts) while preserving edges
float applyMedianFilter(float newValue) {
    // Add new value to circular buffer
    brakeDetector.medianBuffer[brakeDetector.medianIndex] = newValue;
    brakeDetector.medianIndex = (brakeDetector.medianIndex + 1) % 5;

    // Copy buffer and sort to find median
    float sorted[5];
    memcpy(sorted, brakeDetector.medianBuffer, sizeof(sorted));

    // Simple bubble sort for 5 elements
    for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 5; j++) {
            if (sorted[i] > sorted[j]) {
                float tmp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = tmp;
            }
        }
    }

    return sorted[2];  // Middle element
}

// ============== Exponential Moving Average Filter ==============
// Smooths high-frequency noise while maintaining responsiveness
float applyEMAFilter(float newValue) {
    if (!brakeDetector.emaInitialized) {
        brakeDetector.emaValue = newValue;
        brakeDetector.emaInitialized = true;
        return newValue;
    }

    brakeDetector.emaValue = EMA_ALPHA * newValue + (1.0f - EMA_ALPHA) * brakeDetector.emaValue;
    return brakeDetector.emaValue;
}

// ============== Brake Detection ==============
// State machine with filtering: Median → EMA → State transitions
//
// State Diagram:
//
//     ┌─────────────────────────────────────────────────────────┐
//     │                                                         │
//     ▼                                                         │
//   IDLE ──────────────────► PENDING ──────────────► BRAKING    │
//     ▲   accel < ON_THRESH     │    duration >= MIN    │       │
//     │                         │                       │       │
//     │   accel > OFF_THRESH    │                       ▼       │
//     └─────────────────────────┘               RELEASING ──────┘
//       (released too early)                      debounce elapsed
//
void updateBrakeDetection(float rawAccelX) {
    unsigned long now = millis();

    // ===== Stage 1: Apply filters =====
    float medianFiltered = applyMedianFilter(rawAccelX);
    float filtered = applyEMAFilter(medianFiltered);
    imuData.accelForwardFiltered = filtered;

    // ===== Stage 2: Threshold checks =====
    bool belowOnThreshold  = (filtered < BRAKE_THRESHOLD_ON);
    bool aboveOffThreshold = (filtered > BRAKE_THRESHOLD_OFF);

    // ===== Stage 3: State machine =====
    switch (brakeDetector.state) {

        case BrakeState::IDLE:
            // Waiting for brake threshold to be crossed
            if (belowOnThreshold) {
                brakeDetector.state = BrakeState::PENDING;
                brakeDetector.stateEntryTime = now;
            }
            break;

        case BrakeState::PENDING:
            // Threshold crossed, waiting for confirmation duration
            if (aboveOffThreshold) {
                // Released before confirmation → back to IDLE
                brakeDetector.state = BrakeState::IDLE;
            } else if (now - brakeDetector.stateEntryTime >= BRAKE_MIN_DURATION_MS) {
                // Duration met → confirmed braking
                brakeDetector.state = BrakeState::BRAKING;
            }
            break;

        case BrakeState::BRAKING:
            // Confirmed braking, waiting for release
            if (aboveOffThreshold) {
                brakeDetector.state = BrakeState::RELEASING;
                brakeDetector.stateEntryTime = now;
            }
            break;

        case BrakeState::RELEASING:
            // Brake released, debouncing
            if (belowOnThreshold) {
                // Re-engaged brake before debounce finished → back to BRAKING
                brakeDetector.state = BrakeState::BRAKING;
            } else if (now - brakeDetector.stateEntryTime >= BRAKE_DEBOUNCE_MS) {
                // Debounce complete → back to IDLE
                brakeDetector.state = BrakeState::IDLE;
            }
            break;
    }

    // ===== Stage 4: Update output flags =====
    imuData.isBraking    = (brakeDetector.state == BrakeState::BRAKING);
    imuData.brakePending = (brakeDetector.state == BrakeState::PENDING);
}

// ============== LED Control ==============

// Helper: fill a section with a solid color
void fillSection(uint8_t start, uint8_t end, CRGB color) {
    for (uint8_t i = start; i <= end; i++) {
        leds[i] = color;
    }
}

// Main LED update function - called every loop iteration
void updateLEDs() {
    static unsigned long lastLEDUpdate = 0;
    const unsigned long LED_UPDATE_INTERVAL_MS = 20;  // 50 Hz LED refresh

    if (millis() - lastLEDUpdate < LED_UPDATE_INTERVAL_MS) return;
    lastLEDUpdate = millis();

    // Update rainbow animation (pitch controls speed)
    updateSideRainbow();

    // Always update side sections (independent of brake state)
    showSidePatternLeft();
    showSidePatternRight();

    // Update back section based on brake state
    switch (brakeDetector.state) {
        case BrakeState::BRAKING:
        case BrakeState::RELEASING:  // Keep brake light on during release debounce
        case BrakeState::PENDING:    // Show brake light as early warning
            showBrakeLight();
            break;

        case BrakeState::IDLE:
        default:
            showIdlePattern();
            break;
    }

    FastLED.show();
}

// ============== BACK Section (Brake Light) ==============
// Mounted on rear - 13 LEDs (40cm)
void showBrakeLight() {
    // ========================================
    // TODO: Customize brake light effect!
    // ========================================
    //
    // Available data:
    //   imuData.accelForwardFiltered  - Brake intensity (more negative = harder)
    //   SECTION_BACK_START/END        - LED indices for back section
    //
    // Ideas:
    //   - Solid red
    //   - Intensity based on brake force
    //   - Flashing for hard braking (accel < -3 m/s²)

    // Placeholder: solid bright red
    fillSection(SECTION_BACK_START, SECTION_BACK_END, CRGB::Red);
}

// Back section when NOT braking
void showIdlePattern() {
    // ========================================
    // TODO: Customize idle back pattern!
    // ========================================
    //
    // Ideas:
    //   - Dim red tail light
    //   - Off completely
    //   - Subtle glow

    // Placeholder: dim red tail light
    fillSection(SECTION_BACK_START, SECTION_BACK_END, CRGB(30, 0, 0));
}

// ============== Side Sections: Pitch-Controlled Rainbow ==============
//
// Rainbow animation speed is controlled by pitch angle:
//   - Flat (0°):     Slow/static rainbow
//   - Tilted down:   Fast rainbow (going downhill!)
//   - Tilted up:     Reverse rainbow (going uphill)

// Persistent state for rainbow animation
static uint16_t rainbowHueOffset = 0;

// Configuration for rainbow effect
constexpr float RAINBOW_MIN_SPEED = 0.5f;    // Minimum hue change per update (when flat)
constexpr float RAINBOW_MAX_SPEED = 8.0f;    // Maximum hue change per update (steep pitch)
constexpr float RAINBOW_PITCH_SCALE = 0.15f; // How much pitch affects speed (per degree)
constexpr uint8_t RAINBOW_HUE_DELTA = 4;     // Hue difference between adjacent LEDs

// Helper: Update rainbow based on pitch and apply to a section
void updateSideRainbow() {
    // Calculate speed based on pitch angle
    // Positive pitch = tilted forward (downhill) = faster animation
    // Negative pitch = tilted backward (uphill) = slower/reverse
    float pitch = imuData.pitch;

    // Speed scales with pitch: faster when going downhill
    float speed = RAINBOW_MIN_SPEED + (pitch * RAINBOW_PITCH_SCALE);

    // Clamp speed to reasonable bounds (allow negative for reverse)
    speed = constrain(speed, -RAINBOW_MAX_SPEED, RAINBOW_MAX_SPEED);

    // Update the hue offset (wraps automatically due to uint16_t)
    rainbowHueOffset += (int16_t)(speed * 10);  // *10 for sub-integer precision
}

// LEFT Side Section - 26 LEDs (80cm)
void showSidePatternLeft() {
    // Fill left section with rainbow, direction: start → end
    uint8_t hue = rainbowHueOffset / 10;  // Convert back from fixed-point

    for (uint8_t i = SECTION_LEFT_START; i <= SECTION_LEFT_END; i++) {
        leds[i] = CHSV(hue, 255, 200);  // Full saturation, slightly dimmed
        hue += RAINBOW_HUE_DELTA;
    }
}

// RIGHT Side Section - 27 LEDs (80cm)
void showSidePatternRight() {
    // Fill right section with rainbow, direction: end → start (mirrors left)
    uint8_t hue = rainbowHueOffset / 10;  // Same starting hue as left

    for (uint8_t i = SECTION_RIGHT_END; i >= SECTION_RIGHT_START; i--) {
        leds[i] = CHSV(hue, 255, 200);  // Full saturation, slightly dimmed
        hue += RAINBOW_HUE_DELTA;
        if (i == 0) break;  // Prevent underflow
    }
}

// ============== Debug Output ==============
void printIMUData() {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint < 50) return;  // 20 Hz print rate
    lastPrint = millis();

    Serial.print(imuData.roll, 1);
    Serial.print(F("\t"));
    Serial.print(imuData.pitch, 1);
    Serial.print(F("\t"));
    Serial.print(imuData.accelX, 2);
    Serial.print(F("\t"));
    Serial.print(imuData.accelForwardFiltered, 2);
    Serial.print(F("\t\t"));

    // Brake state indicator
    switch (brakeDetector.state) {
        case BrakeState::IDLE:      Serial.println(F("IDLE"));      break;
        case BrakeState::PENDING:   Serial.println(F("PENDING"));   break;
        case BrakeState::BRAKING:   Serial.println(F("BRAKE!"));    break;
        case BrakeState::RELEASING: Serial.println(F("RELEASING")); break;
    }
}
