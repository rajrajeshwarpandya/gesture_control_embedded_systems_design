/*
 * ESP32 Gesture Glove - BLE AirMouse
 *
 * Uses 3 flex sensors (Thumb, Index, Middle) and an MPU6050 IMU over I2C
 * to send standard BLE mouse events:
 *   - Cursor move:  Move hand with fingers relaxed
 *   - Left click:   Bend index finger (> 140 kOhm); quick double-tap for double click
 *   - Right click:  Bend middle finger (> 170 kOhm)
 *   - Double click: Bend thumb + index together
 *   - Middle drag:  Bend index + middle together (holds MOUSE_MIDDLE)
 *   - Scroll:       Bend thumb + middle together and tilt hand up/down
 *   - Right drag:   Bend thumb + index + middle into a fist
 *
 * Wiring:
 *   - MPU6050: SDA -> GPIO 21, SCL -> GPIO 22, VCC -> 3.3V, GND -> GND
 *   - Flex sensors (3.3V -> Flex -> GPIO -> 68kOhm -> GND):
 *       Thumb:  GPIO 35 (active)
 *       Index:  GPIO 32 (active)
 *       Middle: GPIO 33 (active)
 *       Ring:   GPIO 25 (unused)
 *       Pinky:  GPIO 26 (unused)
 *   - Status LED: GPIO 2 (blinks while advertising, solid when connected)
 */

#include <Wire.h>
#include <BleMouse.h>

// Pin configuration
#define I2C_SDA_PIN             21
#define I2C_SCL_PIN             22

#define NUM_FLEX_SENSORS        5
#define PIN_FLEX_THUMB          35 // Finger 0: Thumb  - ACTIVE
#define PIN_FLEX_INDEX          32 // Finger 1: Index  - ACTIVE
#define PIN_FLEX_MIDDLE         33 // Finger 2: Middle - ACTIVE
#define PIN_FLEX_RING           25 // Finger 3: Ring   - Declared
#define PIN_FLEX_PINKY          26 // Finger 4: Pinky  - Declared

const int flexPins[NUM_FLEX_SENSORS] = {
  PIN_FLEX_THUMB,
  PIN_FLEX_INDEX,
  PIN_FLEX_MIDDLE,
  PIN_FLEX_RING,
  PIN_FLEX_PINKY
};

const char* fingerNames[NUM_FLEX_SENSORS] = {
  "Thumb ",
  "Index ",
  "Middle",
  "Ring  ",
  "Pinky "
};

#define LED_PIN                 2

// Voltage divider parameters (3.3V -> Flex -> GPIO -> 68kOhm -> GND)
const float VCC                 = 3.3;
const float R_FIXED             = 68000.0; // 68k Ohms

// MPU6050 & AirMouse tuning
#define MPU_ADDR                0x68
#define PWR_MGMT_1              0x6B
#define ACCEL_CONFIG            0x1C
#define GYRO_CONFIG             0x1B
#define ACCEL_XOUT_H            0x3B

// Cursor sensitivity (0.008-0.010 slower/precise, 0.012-0.016 balanced)
float SENSITIVITY_X             = 0.012;
float SENSITIVITY_Y             = 0.012;

// Axis inversion
const bool INVERT_X             = true;
const bool INVERT_Y             = false;

// Deadzone to filter out small hand tremors
const float GYRO_DEADZONE       = 200.0;

// EMA smoothing factor (0.0 = frozen, 1.0 = raw)
const float SMOOTHING_FACTOR    = 0.65;

// Mouse button used for the 3-finger fist drag (MOUSE_RIGHT or MOUSE_MIDDLE)
#define FUSION_ORBIT_BUTTON     MOUSE_RIGHT

// Flex sensor thresholds (Ohms)
float BEND_THRESHOLD[NUM_FLEX_SENSORS] = {
  160000.0, // Thumb  (GPIO 35) - ACTIVE
  140000.0, // Index  (GPIO 32) - ACTIVE
  170000.0, // Middle (GPIO 33) - ACTIVE
  130000.0, // Ring   (GPIO 25 - unused)
  130000.0  // Pinky  (GPIO 26 - unused)
};

float RELEASE_THRESHOLD[NUM_FLEX_SENSORS] = {
  160000.0, // Thumb  (GPIO 35)
  140000.0, // Index  (GPIO 32)
  170000.0, // Middle (GPIO 33)
  100000.0, // Ring   (GPIO 25 - unused)
  100000.0  // Pinky  (GPIO 26 - unused)
};

// Gesture timing parameters
const unsigned long GESTURE_HOLD_TIME_MS      = 50;  // Normal multi-finger gesture hold time (ms)
const unsigned long RIGHT_CLICK_SETTLE_MS     = 150; // Settle window for Right Click: prevents accidental trigger when folding for multi-finger drag
const unsigned long DOUBLE_CLICK_WINDOW_MS    = 450; // Time window (ms) to detect rapid consecutive taps for double click
const unsigned long DOUBLE_CLICK_CONFIRM_MS   = 15;  // Fast-track debounce (ms) for the second tap of a double click

// Thumb + Index double click gesture
const unsigned long THUMB_INDEX_HOLD_MS       = 100; // Hold time before Thumb+Index fires (avoids false triggers while closing into a fist)
const unsigned long THUMB_INDEX_CLICK_GAP_MS  = 40;  // Gap (ms) between the two clicks fired by Thumb+Index

// Click cursor freeze parameters (locks cursor so finger taps don't shake mouse off target)
const int CLICK_FREEZE_CYCLES                 = 20;  // Freeze for ~240 ms after click (keeps cursor locked on folder)
const int DOUBLE_CLICK_FREEZE_CYCLES          = 15;  // Freeze for ~180 ms after double-click

// Scroll parameters
const float SCROLL_ANGLE_THRESHOLD_DEG   = 8.0;  // Tilt threshold for scrolling
const unsigned long SCROLL_TICK_RATE_MS  = 120;  // Milliseconds between scroll wheel ticks

// Global state & BLE mouse instance
BleMouse bleMouse("Glove AirMouse", "ESP32-Gesture", 100);

float currentResistance[NUM_FLEX_SENSORS];
bool  fingerBent[NUM_FLEX_SENSORS] = { false, false, false, false, false };

float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float mpuFlatPitch = 0.0;

// Filtered cursor velocity
float filteredDX = 0.0;
float filteredDY = 0.0;

// Gesture candidates
enum GestureType {
  GESTURE_IDLE = 0,
  GESTURE_LEFT_CLICK,     // Index bent alone (Left Click)
  GESTURE_RIGHT_CLICK,    // Middle bent alone (Right Click)
  GESTURE_DOUBLE_CLICK,   // Thumb + Index bent (Double Click)
  GESTURE_PAN,            // Index + Middle bent (Middle Mouse Drag)
  GESTURE_SCROLL_ZOOM,    // Thumb + Middle bent (Scroll Up / Down via Hand Tilt)
  GESTURE_ORBIT           // Thumb + Index + Middle bent - Fist (Right Mouse Drag)
};

GestureType activeCandidate      = GESTURE_IDLE;
unsigned long candidateStartTime = 0;

// Latches & action states
bool leftClickDispatched         = false;
bool rightClickDispatched        = false;
bool doubleClickDispatched       = false; // Thumb+Index double click latch
bool isPanning                   = false; // Middle drag
bool isOrbiting                  = false; // Right/middle drag
bool isZooming                   = false; // Scroll zoom

float scrollBasePitch            = 0.0;
unsigned long lastScrollTickTime = 0;

// Double/Triple click tracking
unsigned long lastLeftClickTime  = 0;
int leftClickCount               = 0;
int cursorFreezeCounter          = 0; // Freeze counter during/after clicks to absorb hand tremors

// Optional debug mode
bool debugMode                   = false;
unsigned long lastDebugPrint     = 0;

// MPU6050 helpers
void mpuWrite(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

bool initMPU() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);
  Wire.setTimeOut(50);
  delay(100);

  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("ERROR: MPU6050 not detected at 0x68! Check SDA (21) & SCL (22).");
    return false;
  }

  // Wake up MPU
  mpuWrite(PWR_MGMT_1, 0x00);
  delay(30);

  // +/-4g accel, +/-500dps gyro
  mpuWrite(ACCEL_CONFIG, 0x08);
  delay(15);
  mpuWrite(GYRO_CONFIG, 0x08);
  delay(15);

  Serial.println("MPU6050 initialized.");
  return true;
}

bool readMPU(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  ax = ay = az = gx = gy = gz = 0;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) == 14) {
    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // Skip temp
    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();
    return true;
  }
  return false;
}

// Flex sensor helpers
float calculateFlexResistance(int adcVal) {
  if (adcVal <= 5) return 999999.0;
  if (adcVal >= 4090) return 100.0;

  float vOut = ((float)adcVal / 4095.0) * VCC;
  float rFlex = R_FIXED * ((VCC / vOut) - 1.0);
  return rFlex;
}

void calibrateAllSensors() {
  Serial.println("\nCalibrating MPU6050... Keep glove flat on table.");
  digitalWrite(LED_PIN, HIGH);

  long sumGX = 0, sumGY = 0, sumGZ = 0;
  float sumPitch = 0.0;
  int mpuSamples = 0;
  
  for (int attempt = 0; attempt < 250 && mpuSamples < 120; attempt++) {
    int16_t ax, ay, az, gx, gy, gz;
    if (readMPU(ax, ay, az, gx, gy, gz)) {
      sumGX += gx; sumGY += gy; sumGZ += gz;
      sumPitch += (atan2((float)ax, (float)az) * 180.0 / PI);
      mpuSamples++;
    }
    delay(10);
  }

  if (mpuSamples > 0) {
    gyroBiasX    = (float)sumGX / mpuSamples;
    gyroBiasY    = (float)sumGY / mpuSamples;
    gyroBiasZ    = (float)sumGZ / mpuSamples;
    mpuFlatPitch = sumPitch / mpuSamples;
  }

  // Initial read of flex sensors
  for (int f = 0; f < NUM_FLEX_SENSORS; f++) {
    float sumR = 0;
    for (int i = 0; i < 20; i++) {
      int adc = analogRead(flexPins[f]);
      sumR += calculateFlexResistance(adc);
      delay(5);
    }
    currentResistance[f] = sumR / 20.0;
    fingerBent[f] = (currentResistance[f] > BEND_THRESHOLD[f]);
    Serial.printf("  %s (Pin %d) Current: %5.1f kOhm  [Fold Trigger: > %.0f kOhm]\n", 
                  fingerNames[f], flexPins[f], currentResistance[f] / 1000.0, BEND_THRESHOLD[f] / 1000.0);
  }

  // Reset latches
  leftClickDispatched   = false;
  rightClickDispatched  = false;
  doubleClickDispatched = false;
  isPanning             = false;
  isOrbiting            = false;
  isZooming             = false;
  activeCandidate       = GESTURE_IDLE;

  digitalWrite(LED_PIN, LOW);
  Serial.println("\nSensors calibrated.");
  Serial.printf("  Flat Pitch: %.1f deg | Mouse Inversions: X=%s, Y=%s\n", 
                mpuFlatPitch, INVERT_X ? "INV" : "NORM", INVERT_Y ? "INV" : "NORM");
  Serial.println("-----------------------------------------------------------------------");
}

void updateFlexSensors() {
  for (int i = 0; i < NUM_FLEX_SENSORS; i++) {
    int adc = analogRead(flexPins[i]);
    float r = calculateFlexResistance(adc);

    // Exponential smoothing filter
    currentResistance[i] = (currentResistance[i] * 0.70) + (r * 0.30);

    // Schmitt trigger hysteresis with fast release detection
    if (!fingerBent[i]) {
      if (currentResistance[i] > BEND_THRESHOLD[i]) {
        fingerBent[i] = true;
      }
    } else {
      // Release when filtered drops OR raw ADC drops below 95% of threshold (fast-tracks unbend)
      if (currentResistance[i] < RELEASE_THRESHOLD[i] || r < (BEND_THRESHOLD[i] * 0.95)) {
        fingerBent[i] = false;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("\nESP32 Glove AirMouse");
  Serial.println("Active fingers: Thumb(35), Index(32), Middle(33)");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  analogReadResolution(12);

  if (!initMPU()) {
    while (true) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(300);
    }
  }

  calibrateAllSensors();

  Serial.println("Starting BLE Mouse service (Glove AirMouse)...");
  bleMouse.begin();
  Serial.println("Bluetooth advertising. Connect to 'Glove AirMouse' on your PC/Mac.\n");
}

void loop() {
  // Check Bluetooth connection
  if (!bleMouse.isConnected()) {
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 800) {
      lastBlink = millis();
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      Serial.println("Waiting for Bluetooth connection to 'Glove AirMouse'...");
    }
    delay(40);
    return;
  } else {
    digitalWrite(LED_PIN, HIGH);
  }

  // Serial user commands ('c'=recalibrate, 'd'=debug toggle)
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'c' || cmd == 'r' || cmd == 'C' || cmd == 'R') {
      calibrateAllSensors();
      return;
    } else if (cmd == 'd' || cmd == 'D') {
      debugMode = !debugMode;
      Serial.printf("\n[DEBUG MODE] %s\n\n", debugMode ? "ENABLED" : "DISABLED");
    }
  }

  // 1. Read Flex Sensors
  updateFlexSensors();
  bool thumbBent  = fingerBent[0]; // Pin 35
  bool indexBent  = fingerBent[1]; // Pin 32
  bool middleBent = fingerBent[2]; // Pin 33

  if (debugMode && millis() - lastDebugPrint > 500) {
    lastDebugPrint = millis();
    Serial.printf("[DEBUG R] T: %5.1fk (%s) | I: %5.1fk (%s) | M: %5.1fk (%s) | Gesture: %d\n",
                  currentResistance[0] / 1000.0, thumbBent ? "BENT" : "--- ",
                  currentResistance[1] / 1000.0, indexBent ? "BENT" : "--- ",
                  currentResistance[2] / 1000.0, middleBent ? "BENT" : "--- ",
                  activeCandidate);
  }

  // 2. Read MPU6050 Motion
  int16_t ax, ay, az, gx, gy, gz;
  if (!readMPU(ax, ay, az, gx, gy, gz)) {
    delay(12);
    return;
  }

  float gyroY = gy - gyroBiasY; // Pitch rate
  float gyroZ = gz - gyroBiasZ; // Yaw rate

  float cleanRateX = (fabsf(gyroZ) > GYRO_DEADZONE) ? (gyroZ > 0 ? gyroZ - GYRO_DEADZONE : gyroZ + GYRO_DEADZONE) : 0;
  float cleanRateY = (fabsf(gyroY) > GYRO_DEADZONE) ? (gyroY > 0 ? gyroY - GYRO_DEADZONE : gyroY + GYRO_DEADZONE) : 0;

  if (INVERT_X) cleanRateX = -cleanRateX;
  if (INVERT_Y) cleanRateY = -cleanRateY;

  float currentPitch = atan2((float)ax, (float)az) * 180.0 / PI;

  // Gesture classification (state machine & debounce)
  GestureType currentCandidate = GESTURE_IDLE;

  if (thumbBent && indexBent && middleBent) {
    // All 3 fingers folded into a FIST -> RIGHT MOUSE DRAG
    currentCandidate = GESTURE_ORBIT;
  }
  else if (indexBent && middleBent && !thumbBent) {
    // Index + Middle folded together (2 fingers) -> MIDDLE MOUSE DRAG
    currentCandidate = GESTURE_PAN;
  }
  else if (thumbBent && middleBent && !indexBent) {
    // Thumb + Middle folded -> SCROLL UP / DOWN (Tilt hand UP/DOWN)
    currentCandidate = GESTURE_SCROLL_ZOOM;
  }
  else if (thumbBent && indexBent && !middleBent) {
    // Thumb + Index folded -> DOUBLE CLICK (two quick left clicks)
    currentCandidate = GESTURE_DOUBLE_CLICK;
  }
  else if (indexBent && !thumbBent && !middleBent) {
    // Index bent alone -> LEFT CLICK
    currentCandidate = GESTURE_LEFT_CLICK;
  }
  else if (middleBent && !thumbBent && !indexBent) {
    // Middle bent alone -> RIGHT CLICK (Context Menu)
    currentCandidate = GESTURE_RIGHT_CLICK;
  }

  // Fast re-arm when Index unbends (ensures double-click can register immediately)
  if (!indexBent || (currentResistance[1] < BEND_THRESHOLD[1])) {
    leftClickDispatched = false;
  }

  // Candidate stability filter with adaptive hold timing
  if (currentCandidate != activeCandidate) {
    activeCandidate = currentCandidate;
    candidateStartTime = millis();
  }

  unsigned long candidateDuration = millis() - candidateStartTime;
  unsigned long requiredHoldTime  = GESTURE_HOLD_TIME_MS;

  if (activeCandidate == GESTURE_RIGHT_CLICK) {
    // Settle delay: ensures other fingers aren't just curling slightly later
    requiredHoldTime = RIGHT_CLICK_SETTLE_MS;
  } else if (activeCandidate == GESTURE_DOUBLE_CLICK) {
    // Longer hold so passing through Thumb+Index while closing a fist doesn't fire
    requiredHoldTime = THUMB_INDEX_HOLD_MS;
  } else if (activeCandidate == GESTURE_LEFT_CLICK) {
    // Fast-track confirmation to 15ms if this is the 2nd tap of a double-click!
    if (millis() - lastLeftClickTime < DOUBLE_CLICK_WINDOW_MS && lastLeftClickTime > 0) {
      requiredHoldTime = DOUBLE_CLICK_CONFIRM_MS;
    }
  }

  bool isGestureConfirmed = (candidateDuration >= requiredHoldTime);

  // Process gestures & send BLE mouse events

  // Right mouse drag (Thumb + Index + Middle fist)
  if (isGestureConfirmed && activeCandidate == GESTURE_ORBIT) {
    if (!isOrbiting) {
      bleMouse.press(FUSION_ORBIT_BUTTON);
      isOrbiting = true;
      Serial.println("Right drag: active");
    }
  } else {
    if (isOrbiting) {
      bleMouse.release(FUSION_ORBIT_BUTTON);
      isOrbiting = false;
      Serial.println("Right drag: released");
    }
  }

  // Middle mouse drag (Index + Middle folded together)
  if (isGestureConfirmed && activeCandidate == GESTURE_PAN) {
    if (!isPanning) {
      bleMouse.press(MOUSE_MIDDLE);
      isPanning = true;
      Serial.println("Middle drag: active");
    }
  } else {
    if (isPanning) {
      bleMouse.release(MOUSE_MIDDLE);
      isPanning = false;
      Serial.println("Middle drag: released");
    }
  }

  // Double click (Thumb + Index folded together)
  // Fires two quick left clicks once per pose. Unfold and refold to fire again.
  if (isGestureConfirmed && activeCandidate == GESTURE_DOUBLE_CLICK) {
    if (!doubleClickDispatched) {
      bleMouse.click(MOUSE_LEFT);
      delay(THUMB_INDEX_CLICK_GAP_MS);
      bleMouse.click(MOUSE_LEFT);
      cursorFreezeCounter = DOUBLE_CLICK_FREEZE_CYCLES;
      doubleClickDispatched = true;
      Serial.println("Double click (Thumb + Index)");
    }
  } else {
    doubleClickDispatched = false; // Re-arm once the pose is released
  }

  // Scroll up / down (Thumb + Middle folded together)
  if (isGestureConfirmed && activeCandidate == GESTURE_SCROLL_ZOOM) {
    if (!isZooming) {
      isZooming = true;
      scrollBasePitch = currentPitch;
      lastScrollTickTime = millis();
      Serial.printf("Scroll mode: active (base pitch: %.1f deg)\n", scrollBasePitch);
    } else {
      unsigned long now = millis();
      if (now - lastScrollTickTime >= SCROLL_TICK_RATE_MS) {
        float deltaPitch = currentPitch - scrollBasePitch;
        if (deltaPitch > SCROLL_ANGLE_THRESHOLD_DEG) {
          bleMouse.move(0, 0, 1); // Scroll UP (Wheel UP)
          lastScrollTickTime = now;
        } else if (deltaPitch < -SCROLL_ANGLE_THRESHOLD_DEG) {
          bleMouse.move(0, 0, -1); // Scroll DOWN (Wheel DOWN)
          lastScrollTickTime = now;
        }
      }
    }
  } else {
    if (isZooming) {
      isZooming = false;
      Serial.println("Scroll mode: exited");
    }
  }

  // Left click (Index folded alone)
  if (isGestureConfirmed && activeCandidate == GESTURE_LEFT_CLICK) {
    if (!leftClickDispatched) {
      bleMouse.click(MOUSE_LEFT);

      unsigned long now = millis();
      if (now - lastLeftClickTime < DOUBLE_CLICK_WINDOW_MS && lastLeftClickTime > 0) {
        Serial.println("Double click (rapid double tap)");
        lastLeftClickTime = 0; // Reset window so 3rd tap isn't a triple double-click
        cursorFreezeCounter = DOUBLE_CLICK_FREEZE_CYCLES; // Freeze cursor after double-click
      } else {
        Serial.println("Left click");
        lastLeftClickTime = now;
        cursorFreezeCounter = CLICK_FREEZE_CYCLES; // Freeze cursor for 20 cycles (~240 ms) so double click hits same spot
      }
      leftClickDispatched = true;
    }
  }

  // Right click (Middle folded alone)
  // Requires Middle finger held alone for >= RIGHT_CLICK_SETTLE_MS (150ms)
  // OR a clean quick tap-and-release (40ms - 150ms) of Middle finger alone.
  // If Thumb or Index bends during this window, Right Click is cancelled.
  static unsigned long middleAloneStartTime = 0;
  static bool middleWasCleanTap = false;

  if (middleBent && !thumbBent && !indexBent) {
    if (middleAloneStartTime == 0) {
      middleAloneStartTime = millis();
      middleWasCleanTap = true;
    }
  } else {
    // Middle is no longer alone (either unbent, or Thumb/Index is also bent)
    if (middleAloneStartTime != 0) {
      unsigned long heldDuration = millis() - middleAloneStartTime;
      // Clean quick release with no other fingers bent
      if (middleWasCleanTap && !thumbBent && !indexBent && heldDuration >= 40 && heldDuration < RIGHT_CLICK_SETTLE_MS && !rightClickDispatched) {
        bleMouse.click(MOUSE_RIGHT);
        cursorFreezeCounter = CLICK_FREEZE_CYCLES;
        Serial.println("Right click (quick tap)");
      }
      middleAloneStartTime = 0;
      middleWasCleanTap = false;
    }
  }

  // Sustained hold (>= 150ms) of Middle finger alone
  if (isGestureConfirmed && activeCandidate == GESTURE_RIGHT_CLICK) {
    if (!rightClickDispatched) {
      bleMouse.click(MOUSE_RIGHT);
      cursorFreezeCounter = CLICK_FREEZE_CYCLES;
      Serial.println("Right click (hold)");
      rightClickDispatched = true;
    }
  }
  if (!middleBent) {
    rightClickDispatched = false;
  }

  // Air mouse cursor movement
  // Cursor moves during normal pointing and drag gestures.
  // Paused during:
  //   1. Scroll Mode (so hand tilt scrolls without cursor drift)
  //   2. Clicking Gestures (locks cursor in place so finger taps don't push pointer off target)
  //   3. Click Freeze Window (absorbs hand tremor so double-clicks hit the exact same pixel)

  bool isClickFreezing = (cursorFreezeCounter > 0);
  if (cursorFreezeCounter > 0) {
    cursorFreezeCounter--;
  }

  bool isClickCandidate = (activeCandidate == GESTURE_LEFT_CLICK ||
                           activeCandidate == GESTURE_RIGHT_CLICK ||
                           activeCandidate == GESTURE_DOUBLE_CLICK);

  if (!isZooming && !isClickCandidate && !isClickFreezing) {
    float handMoveX = cleanRateX * SENSITIVITY_X;
    float handMoveY = cleanRateY * SENSITIVITY_Y;

    // Exponential Moving Average (EMA) smoothing filter
    filteredDX = (filteredDX * (1.0 - SMOOTHING_FACTOR)) + (handMoveX * SMOOTHING_FACTOR);
    filteredDY = (filteredDY * (1.0 - SMOOTHING_FACTOR)) + (handMoveY * SMOOTHING_FACTOR);

    signed char moveX = (signed char)constrain((int)filteredDX, -127, 127);
    signed char moveY = (signed char)constrain((int)filteredDY, -127, 127);

    if (moveX != 0 || moveY != 0) {
      bleMouse.move(moveX, moveY, 0);
    }
  } else {
    // Reset filtered velocity memory while frozen so there is no sudden jump on unfreeze
    filteredDX = 0.0;
    filteredDY = 0.0;
  }

  delay(12); // ~80 Hz polling loop for low-latency cursor tracking
}