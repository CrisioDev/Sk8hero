#include <Wire.h>                     // I2C library for communication with sensors
#include <VL53L0X.h>                  // Library for VL53L0X time-of-flight distance sensors
#include <MPU6050.h>                  // Library for MPU6050 accelerometer/gyro sensor
#include <XInput.h>                   // Library to simulate Xbox controller input
#include <math.h>                     // Math functions like sqrt and atan2

// Pin-to-Control Mapping: (hardware pins to functions in the code)
// D4  -> laserLinks   (left distance sensor)
// D5  -> laserHinten  (back distance sensor, also D-Pad Down)
// D6  -> laserRechts  (right distance sensor)
// D7  -> laserVorne   (front distance sensor, also Button Y)
// A0  -> Button R3    (press right stick) -> Alternativer Modus: Button B
// A1  -> Button LT    (left trigger, full on/off) -> Alternativer Modus: Button RT
// A2  -> Mode Switch  (hold to map roll to right stick, else left stick) -> Alternativer Modus: DPAD UP
// A3  -> Slider RT    (right trigger, analog controlled) -> Alternativer Modus: Button LB
// D8  -> Select       (maps to BACK)
// D9  -> Button B     (normal button) -> Alternativer Modus: DPAD rechts
// D10 -> Button START -> Alternativer Modus: START
// D11 -> Button RB    -> Alternativer Modus: DPAD Links
// D12 -> Button X     -> Alternativer Modus: Button X
// D13 -> Button LB    (left bumper) -> Alternativer Modus: Button RB

// ----- Sensor objects -----
VL53L0X laserLinks, laserHinten, laserRechts, laserVorne;
MPU6050 mpu1(0x68), mpu2(0x69);  // Two MPU6050 sensors with I2C addresses 0x68 and 0x69

// ----- Pin definitions -----
#define XSHUT_LEFT    4
#define XSHUT_BACK    5
#define XSHUT_RIGHT   6
#define XSHUT_FRONT   7
#define BTN_DIG_1     8
#define BTN_DIG_2     9
#define BTN_DIG_3     10
#define BTN_DIG_4     11
#define BTN_DIG_5     12
#define BTN_DIG_6     13
#define MODE_SWITCH   A2
#define SLIDER_R2     A3  // Slider temporarily disabled
#define BTN_R3        A0
#define BTN_LT        A1

// ----- Thresholds & constants -----
const uint16_t NEAR_TH    = 200;    // distance threshold in millimeters
const uint16_t LASER_HYST = 20;     // hysteresis margin in millimeters
const float   MAX_ANGLE   = 12.5f;  // maximum tilt angle in degrees
const float   ROLL_DEAD   = 5.0f;   // small deadzone for roll

// ----- NEUE KONSTANTEN für Pitch-Erkennung -----
const float   PITCH_THRESHOLD = 10.0f;  // Grad, ab dem das vordere Anheben erkannt wird
const float   PITCH_HYST = 3.0f;        // Hysterese für Pitch-Erkennung

// ----- NEUE KONSTANTEN für Blockier-Stopp -----
const uint32_t BLOCK_TIME = 500;         // Zeit in ms bis Bewegung gestoppt wird

// ----- Step detection variables -----
bool lastNearL = false, lastNearR = false;  // store previous 'near' states
int32_t forwardLevel = 0;                   // current forward/backward level
const int32_t STEP_INC = 4000;              // how much to add per step
const int32_t DECAY    = 50;                // how fast to decay when idle
const int32_t MAX_FWD  = 32767;             // maximum forward level

// ----- NEUE VARIABLEN für Blockier-Erkennung -----
uint32_t leftBlockStartTime = 0;         // Zeitpunkt, wann linker Sensor blockiert wurde
uint32_t rightBlockStartTime = 0;        // Zeitpunkt, wann rechter Sensor blockiert wurde
bool leftBlocked = false;                // Status ob linker Sensor blockiert ist
bool rightBlocked = false;               // Status ob rechter Sensor blockiert ist
bool movementStopped = false;            // Status ob Bewegung gestoppt wurde

// ----- NEUE VARIABLE für Pitch-Erkennung -----
bool noseUp = false;                      // Status des vorderen Anhebens

// ----- NEUE VARIABLEN für alternativen Button-Modus -----
bool alternativeMode = false;           // Status des alternativen Modus
uint8_t startButtonPressCount = 0;      // Zähler für Start-Button Drücke
uint32_t firstStartPressTime = 0;       // Zeitpunkt des ersten Start-Drucks
uint32_t lastStartPressTime = 0;        // Zeitpunkt des letzten Start-Drucks
bool lastStartButtonState = false;      // Vorheriger Zustand des Start-Buttons
const uint32_t MODE_SWITCH_WINDOW = 10000;  // 10 Sekunden Zeitfenster
const uint8_t REQUIRED_PRESSES = 5;     // 5 Drücke erforderlich

// ----- NEUE VARIABLEN für RS->LS Doppel-Click -----
uint8_t r3ButtonPressCount = 0;         // Zähler für R3-Button Drücke
uint32_t firstR3PressTime = 0;          // Zeitpunkt des ersten R3-Drucks
bool lastR3ButtonState = false;         // Vorheriger Zustand des R3-Buttons
bool lsButtonActive = false;            // Status ob LS gerade aktiv ist
uint32_t lsButtonStartTime = 0;         // Zeitpunkt wann LS aktiviert wurde
const uint32_t R3_CLICK_WINDOW = 500;   // 500ms Zeitfenster für 2 Klicks
const uint8_t REQUIRED_R3_CLICKS = 2;   // 2 Klicks erforderlich
const uint32_t LS_HOLD_TIME = 200;      // LS wird 200ms gehalten

// ----- Funktion für RS->LS Doppel-Click Erkennung -----
void checkR3ToLSDoubleClick() {
  // Aktuellen R3-Button Zustand lesen (je nach Modus)
  bool currentR3State;
  if (alternativeMode) {
    // Im alternativen Modus ist A0 -> Button B, also kein R3
    currentR3State = false;
  } else {
    currentR3State = (digitalRead(BTN_R3) == LOW);
  }
  
  uint32_t currentTime = millis();
  
  // Erkennung eines neuen R3-Button-Drucks (Flanken-Erkennung)
  if (currentR3State && !lastR3ButtonState) {
    // R3-Button wurde gerade gedrückt
    
    if (r3ButtonPressCount == 0) {
      // Erster Druck - Timer starten
      firstR3PressTime = currentTime;
      r3ButtonPressCount = 1;
    } else {
      // Prüfen ob noch im Zeitfenster
      if (currentTime - firstR3PressTime <= R3_CLICK_WINDOW) {
        r3ButtonPressCount++;
        
        // Prüfen ob 2 Klicks erreicht
        if (r3ButtonPressCount >= REQUIRED_R3_CLICKS) {
          // LS-Button aktivieren
          lsButtonActive = true;
          lsButtonStartTime = currentTime;
          r3ButtonPressCount = 0;  // Reset
        }
      } else {
        // Zeitfenster abgelaufen - Reset mit neuem ersten Klick
        firstR3PressTime = currentTime;
        r3ButtonPressCount = 1;
      }
    }
  }
  
  // Reset wenn zu lange kein Klick
  if (r3ButtonPressCount > 0 && 
      currentTime - firstR3PressTime > R3_CLICK_WINDOW) {
    r3ButtonPressCount = 0;
  }
  
  // LS-Button nach bestimmter Zeit deaktivieren
  if (lsButtonActive && 
      currentTime - lsButtonStartTime > LS_HOLD_TIME) {
    lsButtonActive = false;
  }
  
  lastR3ButtonState = currentR3State;
}

// ----- Funktion für alternativen Modus-Wechsel -----
void checkAlternativeModeSwitch() {
  // Aktuellen Start-Button Zustand lesen
  bool currentStartState = (digitalRead(BTN_DIG_3) == LOW);
  uint32_t currentTime = millis();
  
  // Erkennung eines neuen Button-Drucks (Flanken-Erkennung)
  if (currentStartState && !lastStartButtonState) {
    // Start-Button wurde gerade gedrückt
    
    if (startButtonPressCount == 0) {
      // Erster Druck - Timer starten
      firstStartPressTime = currentTime;
      startButtonPressCount = 1;
    } else {
      // Prüfen ob noch im Zeitfenster
      if (currentTime - firstStartPressTime <= MODE_SWITCH_WINDOW) {
        startButtonPressCount++;
        
        // Prüfen ob 5 Drücke erreicht
        if (startButtonPressCount >= REQUIRED_PRESSES) {
          // Modus wechseln
          alternativeMode = !alternativeMode;
          startButtonPressCount = 0;  // Reset
          
          // Optional: Feedback durch kurzes Vibrieren der Trigger
          // (simuliert durch kurzes Setzen der Trigger)
          if (alternativeMode) {
            // 3x kurz blinken für "Alternative Mode AN"
            for (int i = 0; i < 3; i++) {
              XInput.setTrigger(TRIGGER_LEFT, 255);
              XInput.setTrigger(TRIGGER_RIGHT, 255);
              XInput.send();
              delay(100);
              XInput.setTrigger(TRIGGER_LEFT, 0);
              XInput.setTrigger(TRIGGER_RIGHT, 0);
              XInput.send();
              delay(100);
            }
          } else {
            // 1x lang für "Alternative Mode AUS"
            XInput.setTrigger(TRIGGER_LEFT, 255);
            XInput.setTrigger(TRIGGER_RIGHT, 255);
            XInput.send();
            delay(500);
            XInput.setTrigger(TRIGGER_LEFT, 0);
            XInput.setTrigger(TRIGGER_RIGHT, 0);
            XInput.send();
          }
        }
      } else {
        // Zeitfenster abgelaufen - Reset
        firstStartPressTime = currentTime;
        startButtonPressCount = 1;
      }
    }
    
    lastStartPressTime = currentTime;
  }
  
  // Reset wenn zu lange kein Druck
  if (startButtonPressCount > 0 && 
      currentTime - firstStartPressTime > MODE_SWITCH_WINDOW) {
    startButtonPressCount = 0;
  }
  
  lastStartButtonState = currentStartState;
}

// ----- Funktion für alternative Button-Belegung -----
void handleAlternativeButtonMapping() {
  if (alternativeMode) {
    // ALTERNATIVE BELEGUNG
    // Digital Pins (D8-D13)
    if (digitalRead(BTN_DIG_1) == LOW) XInput.press(BUTTON_BACK);        else XInput.release(BUTTON_BACK);   // D8 -> BACK (unverändert)
    
    // D-Pad Handling mit kombinierter Logik
    bool dpadLeft = (digitalRead(BTN_DIG_2) == LOW);   // D11 -> DPAD Links
    bool dpadRight = (digitalRead(BTN_DIG_4) == LOW);  // D9 -> DPAD Rechts
    bool dpadUp = (digitalRead(MODE_SWITCH) == LOW);   // A2 -> DPAD UP
    bool backNear = (laserHinten.readRangeContinuousMillimeters() < (NEAR_TH - LASER_HYST));  // Hinterer Sensor -> DPAD Down
    XInput.setDpad(dpadUp,backNear,dpadLeft,dpadRight);
    
    if (digitalRead(BTN_DIG_5) == LOW) XInput.press(BUTTON_X);           else XInput.release(BUTTON_X);      // D12 -> X (unverändert)
    if (digitalRead(BTN_DIG_6) == LOW) XInput.press(BUTTON_RB);          else XInput.release(BUTTON_RB);     // D13 -> RB statt LB
    
    // Analog Pins (A0, A1, A3) - neue Belegungen
    if (digitalRead(BTN_R3) == LOW) XInput.press(BUTTON_B);              else XInput.release(BUTTON_B);      // A0 -> B statt R3
    if (digitalRead(BTN_LT) == LOW) XInput.setTrigger(TRIGGER_RIGHT, 255); else XInput.setTrigger(TRIGGER_RIGHT, 0); // A1 -> RT statt LT
    
    // A3 Slider -> LB Button (vereinfacht)
    int slide = analogRead(SLIDER_R2);
    if (slide > 512) XInput.press(BUTTON_LB);  else XInput.release(BUTTON_LB);  // A3 -> LB statt RT Slider
    
    // Vorderer Sensor -> Y (unverändert)
    if (laserVorne.readRangeContinuousMillimeters() < (NEAR_TH - LASER_HYST)) {
      XInput.press(BUTTON_Y);
    } else {
      XInput.release(BUTTON_Y);
    }
    
    // A-Button durch Pitch auch im alternativen Modus
    if (noseUp) {
      XInput.press(BUTTON_A);
    } else {
      XInput.release(BUTTON_A);
    }
    
  } else {
    // NORMALE BELEGUNG (bestehender Code)
    if (digitalRead(BTN_DIG_1) == LOW) XInput.press(BUTTON_BACK);        else XInput.release(BUTTON_BACK);
    if (digitalRead(BTN_DIG_2) == LOW) XInput.press(BUTTON_B);           else XInput.release(BUTTON_B);
    if (digitalRead(BTN_DIG_4) == LOW) XInput.press(BUTTON_RB);          else XInput.release(BUTTON_RB);
    if (digitalRead(BTN_DIG_5) == LOW) XInput.press(BUTTON_X);           else XInput.release(BUTTON_X);
    if (digitalRead(BTN_DIG_6) == LOW) XInput.press(BUTTON_LB);          else XInput.release(BUTTON_LB);
    
    // Normale Sensor-Belegung
    bool backNear = (laserHinten.readRangeContinuousMillimeters() < (NEAR_TH - LASER_HYST));
    XInput.setDpad(false, backNear, false, false);  // D-pad Down
    
    if (laserVorne.readRangeContinuousMillimeters() < (NEAR_TH - LASER_HYST)) {
      XInput.press(BUTTON_Y);
    } else {
      XInput.release(BUTTON_Y);
    }
    
    // A-Button durch Pitch auslösen (nur im normalen Modus)
    if (noseUp) {
      XInput.press(BUTTON_A);
    } else {
      XInput.release(BUTTON_A);
    }
  }
  
  // Start-Button funktioniert in beiden Modi normal
  if (digitalRead(BTN_DIG_3) == LOW) XInput.press(BUTTON_START);       else XInput.release(BUTTON_START);
}

void setup() {
  // Initialize I2C and XInput
  Wire.begin();
  XInput.begin();

  // 1) Reset all four VL53L0X sensors by pulling their XSHUT pins low
  int xPins[4] = {XSHUT_LEFT, XSHUT_BACK, XSHUT_RIGHT, XSHUT_FRONT};
  for (int i = 0; i < 4; ++i) {
    pinMode(xPins[i], OUTPUT);      // set XSHUT pins as outputs
    digitalWrite(xPins[i], LOW);    // hold in reset
  }
  delay(50); // wait for sensors to reset

  // 2) Activate and configure each sensor one by one
  // Left sensor
  digitalWrite(XSHUT_LEFT, HIGH);  // bring out of reset
  delay(10);
  laserLinks.init();               // initialize sensor object
  laserLinks.setAddress(0x30);     // give it a unique I2C address
  laserLinks.startContinuous(20);  // start continuous measurement every 20ms

  // Back sensor
  digitalWrite(XSHUT_BACK, HIGH);  delay(10);
  laserHinten.init();
  laserHinten.setAddress(0x31);
  laserHinten.startContinuous(20);

  // Right sensor
  digitalWrite(XSHUT_RIGHT, HIGH); delay(10);
  laserRechts.init();
  laserRechts.setAddress(0x32);
  laserRechts.startContinuous(20);

  // Front sensor
  digitalWrite(XSHUT_FRONT, HIGH); delay(10);
  laserVorne.init();
  laserVorne.setAddress(0x33);
  laserVorne.startContinuous(20);

  // 3) Initialize both MPU sensors and calibrate them
  MPU6050* mpus[2] = {&mpu1, &mpu2};
  for (int i = 0; i < 2; ++i) {
    mpus[i]->initialize();         // wake up sensor
    mpus[i]->CalibrateAccel();     // calibrate accelerometer
    mpus[i]->CalibrateGyro();      // calibrate gyroscope
  }

  // 4) Set up digital input pins with pull-up resistors
  // D8 to D13 are buttons, so use INPUT_PULLUP (reads HIGH when not pressed)
  for (int pin = BTN_DIG_1; pin <= BTN_DIG_6; ++pin) {
    pinMode(pin, INPUT_PULLUP);
  }
  // Mode switch is also a button-like input
  pinMode(MODE_SWITCH, INPUT_PULLUP);
  // Analog pins A0, A1, A3 read voltage, no pull-up needed
  pinMode(BTN_R3, INPUT_PULLUP);  // enable pull-up to avoid floating input
  pinMode(BTN_LT, INPUT_PULLUP);  // enable pull-up to avoid floating input
  pinMode(SLIDER_R2, INPUT);  // slider disabled
}

void loop() {
  // ----- 1) Step Detection for forward/backward movement (nur im normalen Modus) -----
  if (!alternativeMode) {
    // Read distances from left and right sensors
    uint16_t dL = laserLinks.readRangeContinuousMillimeters();
    uint16_t dR = laserRechts.readRangeContinuousMillimeters();
    // Check if each is "near" (< threshold minus hysteresis)
    bool nL = dL < (NEAR_TH - LASER_HYST);
    bool nR = dR < (NEAR_TH - LASER_HYST);
    
    // ----- NEUER CODE: Blockier-Erkennung -----
    uint32_t currentTime = millis();
    
    // Linker Sensor Blockier-Logik
    if (nL && !leftBlocked) {
      // Sensor wurde gerade blockiert
      if (leftBlockStartTime == 0) {
        leftBlockStartTime = currentTime;
      } else if (currentTime - leftBlockStartTime > BLOCK_TIME) {
        // Sensor ist länger als 500ms blockiert
        leftBlocked = true;
        movementStopped = true;
        forwardLevel = 0;  // Stoppe Bewegung sofort
      }
    } else if (!nL) {
      // Sensor ist nicht mehr blockiert
      leftBlockStartTime = 0;
      leftBlocked = false;
    }
    
    // Rechter Sensor Blockier-Logik
    if (nR && !rightBlocked) {
      // Sensor wurde gerade blockiert
      if (rightBlockStartTime == 0) {
        rightBlockStartTime = currentTime;
      } else if (currentTime - rightBlockStartTime > BLOCK_TIME) {
        // Sensor ist länger als 500ms blockiert
        rightBlocked = true;
        movementStopped = true;
        forwardLevel = 0;  // Stoppe Bewegung sofort
      }
    } else if (!nR) {
      // Sensor ist nicht mehr blockiert
      rightBlockStartTime = 0;
      rightBlocked = false;
    }
    
    // Reset movement stop wenn beide Sensoren frei sind
    if (!nL && !nR && movementStopped) {
      movementStopped = false;
    }
    
    // ----- Normale Step-Detection (nur wenn nicht blockiert) -----
    int32_t stepDelta = 0;
    if (!movementStopped && !leftBlocked && !rightBlocked) {
      // If right sensor just went near, step forward
      if (nR && !lastNearR) stepDelta = STEP_INC;
      // If left sensor just went near, step backward
      if (nL && !lastNearL) stepDelta = -STEP_INC;
    }
    
    lastNearL = nL;
    lastNearR = nR;
    
    // Update forwardLevel or decay towards zero
    if (stepDelta) {
      forwardLevel = constrain(forwardLevel + stepDelta, -MAX_FWD, MAX_FWD);
    } else if (forwardLevel > 0) {
      forwardLevel = max(int32_t(0), forwardLevel - DECAY);
    } else if (forwardLevel < 0) {
      forwardLevel = min(int32_t(0), forwardLevel + DECAY);
    }
  } else {
    // Im alternativen Modus: Step-Detection deaktiviert, forwardLevel auf 0 setzen
    forwardLevel = 0;
  }

  // ----- 2) Calculate roll (tilt) using both MPU sensors -----
  int16_t ax1, ay1, az1, ax2, ay2, az2;
  mpu1.getAcceleration(&ax1, &ay1, &az1);
  mpu2.getAcceleration(&ax2, &ay2, &az2);
  // Convert accelerometer readings to roll angles in degrees
  float roll1 = atan2f(ay1, az1) * 57.2958f;
  float roll2 = atan2f(ay2, az2) * 57.2958f;
  // Average the two rolls
  float rollAvg = (roll1 - roll2) * 0.5f;
  float absRoll = fabsf(rollAvg);
  // Limit absRoll to MAX_ANGLE to prevent overflow
  absRoll = min(absRoll, MAX_ANGLE);
  float procAngle = 0;
  // Apply deadzone and scale to MAX_ANGLE
  if (absRoll > ROLL_DEAD) {
    float norm = sqrtf((absRoll - ROLL_DEAD) / (MAX_ANGLE - ROLL_DEAD));
    procAngle = (rollAvg < 0 ? -1 : 1) * norm * MAX_ANGLE;
  }
  // Convert to joystick value (-32767 to +32767)
  int16_t joyVal = -(int16_t)(procAngle / MAX_ANGLE * 32767);

  // ----- NEUER CODE: Pitch-Erkennung für A-Button -----
  // Berechne Pitch (Vorderseite hoch/runter) aus Beschleunigungsdaten
  float pitch1 = -atan2f(ax1, az1) * 57.2958f;  // Umrechnung in Grad
  float pitch2 = atan2f(ax2, az2) * 57.2958f; // Negiert wegen Spiegelung
  float pitchAvg = (pitch1 + pitch2) * 0.5f;   // Durchschnitt beider Sensoren
  
  // Hysterese anwenden für stabilere Erkennung
  if (!noseUp && pitchAvg > PITCH_THRESHOLD) {
    noseUp = true;
  } else if (noseUp && pitchAvg < (PITCH_THRESHOLD - PITCH_HYST)) {
    noseUp = false;
  }

  // ----- 3) Determine stick axes values for roll and forward -----
  bool modeHeld = (digitalRead(MODE_SWITCH) == LOW);
  // Im alternativen Modus ist MODE_SWITCH D-PAD UP, also kein Mode Switch
  if (alternativeMode) {
    // Im alternativen Modus immer links stick für roll
    int16_t leftX = joyVal;
    int16_t rightX = 0;
    int16_t leftY = forwardLevel;
    int16_t rightY = 0;
    XInput.setJoystick(JOY_LEFT, leftX, leftY);
    XInput.setJoystick(JOY_RIGHT, rightX, rightY);
  } else {
    // Normale Logik
    int16_t leftX  = modeHeld ? 0 : joyVal;
    int16_t rightX = modeHeld ? joyVal : 0;
    int16_t leftY  = forwardLevel;
    int16_t rightY = 0;
    XInput.setJoystick(JOY_LEFT,  leftX,  leftY);
    XInput.setJoystick(JOY_RIGHT, rightX, rightY);
  }

  // ----- 3.5) Prüfe auf RS->LS Doppel-Click -----
  checkR3ToLSDoubleClick();


  // ----- 5) Read triggers and buttons (nur im normalen Modus) -----
  if (!alternativeMode) {
    // Right trigger (RT) from analog slider A3 (0-1023 -> 0-255)
    int slide = analogRead(SLIDER_R2);
    uint8_t r2 = map(slide, 0, 1023, 0, 255);
    // Analoger Trigger steuern:
    if (r2 > 60) {
      XInput.setTrigger(TRIGGER_RIGHT, 255);
    } else {
      XInput.setTrigger(TRIGGER_RIGHT, 0);
    }
    // Left trigger (LT) full on/off from A1 button
    bool ltPressed = (digitalRead(BTN_LT) == LOW);
    XInput.setTrigger(TRIGGER_LEFT, ltPressed ? 255 : 0);
    
    // R3 button from A0 - funktioniert normal, auch während Doppel-Click-Erkennung
    bool r3StateToSend = (digitalRead(BTN_R3) == LOW);
    if (r3StateToSend) XInput.press(JOY_RIGHT);
    else               XInput.release(JOY_RIGHT);
    
    // L3 button (linker Stick-Click) - aktiviert durch Triple-Click
    if (lsButtonActive) XInput.press(JOY_LEFT);
    else                XInput.release(JOY_LEFT);
  }
  // Im alternativen Modus werden diese in handleAlternativeButtonMapping() behandelt

  // ----- 5) Prüfe auf alternativen Modus-Wechsel -----
  checkAlternativeModeSwitch();

  // ----- 6) Map sensors and digital buttons (jetzt mit alternativem Modus) -----
  handleAlternativeButtonMapping();

  // ----- 7) Send all changes to host -----
  XInput.send();
  delay(20);  // small delay to control loop frequency (~50Hz)
}