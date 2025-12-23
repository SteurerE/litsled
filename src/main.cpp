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
#include <EEPROM.h>
#include "I2Cdev.h"
#include "MPU6050.h"           // Basic MPU6050 (no DMP)
#include <MadgwickAHRS.h>      // Software sensor fusion
#include <FastLED.h>

// ============== EEPROM Calibration Storage ==============
#define EEPROM_CAL_ADDR 0
#define EEPROM_CAL_MAGIC 0xCA1B  // Magic number to verify valid data

struct CalibrationData {
    uint16_t magic;       // Magic number to verify data
    float gyroBiasX;
    float gyroBiasY;
    float gyroBiasZ;
    uint16_t checksum;    // Simple checksum
};

uint16_t calcChecksum(const CalibrationData& cal) {
    uint16_t sum = 0;
    const uint8_t* ptr = (const uint8_t*)&cal;
    for (size_t i = 0; i < offsetof(CalibrationData, checksum); i++) {
        sum += ptr[i];
    }
    return sum;
}

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
Madgwick filter;        // Madgwick sensor fusion filter

// Timing for sensor fusion
unsigned long lastUpdate = 0;
const float SAMPLE_RATE_HZ = 100.0f;  // Target sample rate

// Gyro bias (software correction)
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

// Sensor ready flag
bool sensorReady = false;

// ============== Sensor Data (use these in your application) ==============
struct IMUData {
    // Angles in degrees (from Madgwick sensor fusion)
    float roll;
    float pitch;
    float yaw;

    // Debug: raw acceleration (includes gravity)
    float rawAccelX;
    
    // Debug: estimated gravity X component
    float gravityX;

    // Body-frame linear acceleration in m/s² (gravity removed by DMP)
    float accelX;  // Forward (along sled direction) - LINEAR (gravity removed)
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

// ============== Function Declarations ==============
bool initMPU6050();
void updateIMU();
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
bool loadCalibration();
void saveCalibration();
void performCalibration();
void checkSerialCommands();

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
    Serial.println(F("Pitch\tRoll\tRawX\tGravX\tLinX\tFiltered\tBrake"));
}

// ============== Main Loop ==============
void loop() {
    // Check for serial commands (e.g., 'c' to recalibrate)
    checkSerialCommands();
    
    // Update IMU (Madgwick sensor fusion)
    updateIMU();
    
    // Print data at reduced rate
    printIMUData();

    // Update LED display based on brake state
    updateLEDs();
}

// ============== MPU6050 + Madgwick Initialization ==============
bool initMPU6050() {
    Serial.println(F("Initializing MPU6050..."));
    mpu.initialize();

    // Verify connection
    Serial.print(F("Testing MPU6050 connection... "));
    if (!mpu.testConnection()) {
        Serial.println(F("FAILED"));
        return false;
    }
    Serial.println(F("OK"));

    // Configure sensor ranges
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);   // ±250 deg/s
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);   // ±2g
    
    // Set sample rate divider (1kHz / (1 + 9) = 100Hz)
    mpu.setRate(9);
    
    // Configure low-pass filter (reduces noise)
    mpu.setDLPFMode(MPU6050_DLPF_BW_20);
    
    // Initialize Madgwick filter
    filter.begin(SAMPLE_RATE_HZ);
    
    // Try to load calibration from EEPROM
    if (!loadCalibration()) {
        // No valid calibration found - perform new calibration
        Serial.println(F("No stored calibration found."));
        performCalibration();
        saveCalibration();
    }
    
    Serial.print(F("Using gyro bias (deg/s): "));
    Serial.print(gyroBiasX, 2);
    Serial.print(F(", "));
    Serial.print(gyroBiasY, 2);
    Serial.print(F(", "));
    Serial.println(gyroBiasZ, 2);
    
    // Warm up the filter with bias-corrected samples
    Serial.println(F("Warming up Madgwick filter..."));
    for (int i = 0; i < 100; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        
        // Convert to proper units
        float ax_g = ax / 16384.0f;
        float ay_g = ay / 16384.0f;
        float az_g = az / 16384.0f;
        
        // Apply software bias correction
        float gx_dps = (gx / 131.0f) - gyroBiasX;
        float gy_dps = (gy / 131.0f) - gyroBiasY;
        float gz_dps = (gz / 131.0f) - gyroBiasZ;
        
        filter.updateIMU(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g);
        delay(10);
    }
    
    sensorReady = true;
    Serial.println(F("Madgwick filter ready!"));
    return true;
}

// ============== IMU Update (Madgwick Filter) ==============
void updateIMU() {
    if (!sensorReady) return;

    // Calculate actual delta time for accurate filter update
    unsigned long now = micros();
    float dt = (now - lastUpdate) / 1000000.0f;
    lastUpdate = now;
    
    // Clamp dt to reasonable range (avoid issues on first call or overflow)
    if (dt <= 0 || dt > 0.1f) dt = 0.01f;

    // Read raw sensor data
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Convert accelerometer to g (±2g range = 16384 LSB/g)
    float ax_g = ax / 16384.0f;
    float ay_g = ay / 16384.0f;
    float az_g = az / 16384.0f;
    
    // Convert gyroscope to deg/s and apply software bias correction
    float gx_dps = (gx / 131.0f) - gyroBiasX;
    float gy_dps = (gy / 131.0f) - gyroBiasY;
    float gz_dps = (gz / 131.0f) - gyroBiasZ;
    
    // Update Madgwick filter (sensor fusion)
    filter.updateIMU(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g);
    
    // Get orientation from filter
    imuData.roll  = filter.getRoll();
    imuData.pitch = filter.getPitch();
    imuData.yaw   = filter.getYaw();
    
    // Convert accel to m/s²
    float rawX = ax_g * G;
    float rawY = ay_g * G;
    float rawZ = az_g * G;
    
    // Calculate gravity components from orientation
    // (Using pitch angle to estimate gravity on X axis)
    float pitchRad = imuData.pitch * M_PI / 180.0f;
    float rollRad  = imuData.roll * M_PI / 180.0f;
    
    float gravX = -sin(pitchRad) * G;
    float gravY = sin(rollRad) * cos(pitchRad) * G;
    float gravZ = cos(rollRad) * cos(pitchRad) * G;
    
    // Linear acceleration = raw - gravity
    imuData.accelX = rawX - gravX;
    imuData.accelY = rawY - gravY;
    imuData.accelZ = rawZ - gravZ;
    
    // Debug values
    imuData.rawAccelX = rawX;
    imuData.gravityX = gravX;

    // Apply filtering and brake detection
    updateBrakeDetection(imuData.accelX);
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
    static BrakeState lastState = BrakeState::IDLE;
    
    // Only print on state change OR every 200ms (5Hz) - reduces serial overhead
    bool stateChanged = (brakeDetector.state != lastState);
    if (!stateChanged && (millis() - lastPrint < 200)) return;
    
    lastPrint = millis();
    lastState = brakeDetector.state;

    // Compact output to reduce serial time
    Serial.print(imuData.pitch, 0);
    Serial.print(F("\t"));
    Serial.print(imuData.roll, 0);
    Serial.print(F("\t"));
    Serial.print(imuData.accelX, 1);      // Linear accel (main value)
    Serial.print(F("\t"));
    Serial.print(imuData.accelForwardFiltered, 1);
    Serial.print(F("\t"));

    // Brake state indicator
    switch (brakeDetector.state) {
        case BrakeState::IDLE:      Serial.println(F("IDLE"));      break;
        case BrakeState::PENDING:   Serial.println(F("PENDING"));   break;
        case BrakeState::BRAKING:   Serial.println(F("BRAKE!"));    break;
        case BrakeState::RELEASING: Serial.println(F("RELEASING")); break;
    }
}

// ============== EEPROM Calibration Functions ==============

bool loadCalibration() {
    CalibrationData cal;
    EEPROM.get(EEPROM_CAL_ADDR, cal);
    
    // Check magic number
    if (cal.magic != EEPROM_CAL_MAGIC) {
        Serial.println(F("EEPROM: No valid magic number"));
        return false;
    }
    
    // Verify checksum
    if (cal.checksum != calcChecksum(cal)) {
        Serial.println(F("EEPROM: Checksum mismatch"));
        return false;
    }
    
    // Load calibration values
    gyroBiasX = cal.gyroBiasX;
    gyroBiasY = cal.gyroBiasY;
    gyroBiasZ = cal.gyroBiasZ;
    
    Serial.println(F("Loaded calibration from EEPROM"));
    return true;
}

void saveCalibration() {
    CalibrationData cal;
    cal.magic = EEPROM_CAL_MAGIC;
    cal.gyroBiasX = gyroBiasX;
    cal.gyroBiasY = gyroBiasY;
    cal.gyroBiasZ = gyroBiasZ;
    cal.checksum = calcChecksum(cal);
    
    EEPROM.put(EEPROM_CAL_ADDR, cal);
    Serial.println(F("Calibration saved to EEPROM"));
}

void performCalibration() {
    Serial.println(F("Calibrating gyro (keep still 2 sec)..."));
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    const int samples = 200;  // 2 seconds
    
    for (int i = 0; i < samples; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;
        delay(10);
    }
    
    // Store gyro bias in deg/s (131 LSB/(deg/s) at ±250°/s range)
    gyroBiasX = (gx_sum / samples) / 131.0f;
    gyroBiasY = (gy_sum / samples) / 131.0f;
    gyroBiasZ = (gz_sum / samples) / 131.0f;
    
    Serial.println(F("Calibration complete!"));
}

void checkSerialCommands() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'c' || cmd == 'C') {
            Serial.println(F("\n=== Recalibration requested ==="));
            performCalibration();
            saveCalibration();
            
            // Re-warm the filter
            Serial.println(F("Warming up filter..."));
            for (int i = 0; i < 50; i++) {
                int16_t ax, ay, az, gx, gy, gz;
                mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
                float ax_g = ax / 16384.0f;
                float ay_g = ay / 16384.0f;
                float az_g = az / 16384.0f;
                float gx_dps = (gx / 131.0f) - gyroBiasX;
                float gy_dps = (gy / 131.0f) - gyroBiasY;
                float gz_dps = (gz / 131.0f) - gyroBiasZ;
                filter.updateIMU(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g);
                delay(10);
            }
            Serial.println(F("Ready!\n"));
        }
    }
}
