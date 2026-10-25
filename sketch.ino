/*
 * Project : Autonomous Warehouse Robot with Mission States
 * Platform: ESP32 DevKit V1
 * Framework: Arduino C++ / Wokwi Simulator
 * Author  : Vishnuprakash J
 * Date    : 25 October 2026
 *
 * Description:
 * Simulates a warehouse robot travelling between stations,
 * detecting obstacles, managing mission states, and handling faults.
 *
 * Features:
 * 1. Mission-state control
 * 2. Ultrasonic obstacle detection
 * 3. Three-station delivery sequence
 * 4. Emergency-stop input
 * 5. LED status indication
 * 6. Serial Monitor diagnostics
 *
 * Note:
 * Robot movement is simulated through Serial Monitor messages.
 * No physical motor driver is required for this version.
 */

#include <Arduino.h>

// =====================================================
// PIN CONFIGURATION
// =====================================================

// HC-SR04 ultrasonic sensor
const int TRIG_PIN = 5;
const int ECHO_PIN = 17;

// Pushbuttons: each button connects GPIO to GND
const int START_BUTTON   = 18;
const int STATION_BUTTON = 19;
const int ESTOP_BUTTON   = 23;

// Status LEDs
const int GREEN_LED  = 25;
const int YELLOW_LED = 26;
const int RED_LED    = 27;

// =====================================================
// ROBOT CONFIGURATION
// =====================================================

const int TOTAL_STATIONS = 3;

const float OBSTACLE_DISTANCE_CM = 15.0;

const unsigned long SENSOR_INTERVAL_MS = 3000;
const unsigned long STATION_SERVICE_MS = 2000;
const unsigned long BUTTON_DEBOUNCE_MS = 40;
const unsigned long OBSTACLE_TIMEOUT_MS = 15000;

// =====================================================
// MISSION STATES
// =====================================================

enum RobotState {
  IDLE,
  START_MISSION,
  FOLLOW_ROUTE,
  OBSTACLE_STOP,
  STATION_SERVICE,
  MISSION_COMPLETE,
  FAULT
};

RobotState currentState = IDLE;

// =====================================================
// GLOBAL VARIABLES
// =====================================================

int currentStation = 0;

float obstacleDistance = -1.0;

bool obstacleDetected = false;
bool emergencyStopActive = false;

unsigned long lastSensorRead = 0;
unsigned long stationServiceStart = 0;
unsigned long obstacleStopStart = 0;

bool startPressed = false;
bool stationPressed = false;
bool eStopPressed = false;

// Button debouncing variables
bool previousStartReading = HIGH;
bool previousStationReading = HIGH;
bool previousEStopReading = HIGH;

bool stableStartState = HIGH;
bool stableStationState = HIGH;
bool stableEStopState = HIGH;

unsigned long lastStartChange = 0;
unsigned long lastStationChange = 0;
unsigned long lastEStopChange = 0;

// Simulated motion state
bool robotIsMoving = false;

// =====================================================
// FUNCTION DECLARATIONS
// =====================================================

void updateButtons();
void readObstacleSensor();
void updateLEDs();
void changeState(RobotState newState);
void handleMission();

void moveForward();
void stopRobot();
void serviceStation();

void printStatus();
const char* stateName(RobotState state);

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  // Diagnostic message to verify Serial Monitor output
  Serial.println();
  Serial.println("ESP32 is working!");

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(STATION_BUTTON, INPUT_PULLUP);
  pinMode(ESTOP_BUTTON, INPUT_PULLUP);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  digitalWrite(TRIG_PIN, LOW);

  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, HIGH);
  digitalWrite(RED_LED, LOW);

  Serial.println();
  Serial.println("==========================================");
  Serial.println(" AUTONOMOUS WAREHOUSE ROBOT");
  Serial.println(" ESP32 Mission-State Controller");
  Serial.println("==========================================");

  Serial.println("[SYSTEM] Initialization complete.");
  Serial.println("[IDLE] Waiting for mission start.");

  printStatus();
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {

  updateButtons();

  // Emergency stop takes priority over normal operation.
  if (eStopPressed && currentState != FAULT) {

    emergencyStopActive = true;

    stopRobot();

    Serial.println("[SAFETY] Emergency stop activated!");

    changeState(FAULT);
  }

  // Remain stopped while a fault is active.
  if (currentState == FAULT) {

    stopRobot();
    updateLEDs();

    return;
  }

  // Periodic ultrasonic sensor measurement.
  if (millis() - lastSensorRead >= SENSOR_INTERVAL_MS) {

    lastSensorRead = millis();

    readObstacleSensor();
  }

  handleMission();

  updateLEDs();
}

// =====================================================
// BUTTON HANDLING WITH DEBOUNCING
// =====================================================

void updateButtons() {

  unsigned long now = millis();

  bool startReading = digitalRead(START_BUTTON);
  bool stationReading = digitalRead(STATION_BUTTON);
  bool eStopReading = digitalRead(ESTOP_BUTTON);

  startPressed = false;
  stationPressed = false;
  eStopPressed = false;

  // START BUTTON
  if (startReading != previousStartReading) {

    previousStartReading = startReading;
    lastStartChange = now;
  }

  if (now - lastStartChange >= BUTTON_DEBOUNCE_MS) {

    if (startReading != stableStartState) {

      stableStartState = startReading;

      if (stableStartState == LOW) {
        startPressed = true;
        Serial.println("[INPUT] Start button pressed.");
      }
    }
  }

  // STATION BUTTON
  if (stationReading != previousStationReading) {

    previousStationReading = stationReading;
    lastStationChange = now;
  }

  if (now - lastStationChange >= BUTTON_DEBOUNCE_MS) {

    if (stationReading != stableStationState) {

      stableStationState = stationReading;

      if (stableStationState == LOW) {
        stationPressed = true;
        Serial.println("[INPUT] Station button pressed.");
      }
    }
  }

  // EMERGENCY-STOP BUTTON
  if (eStopReading != previousEStopReading) {

    previousEStopReading = eStopReading;
    lastEStopChange = now;
  }

  if (now - lastEStopChange >= BUTTON_DEBOUNCE_MS) {

    if (eStopReading != stableEStopState) {

      stableEStopState = eStopReading;

      if (stableEStopState == LOW) {
        eStopPressed = true;
      }
    }
  }
}

// =====================================================
// ULTRASONIC SENSOR
// =====================================================

void readObstacleSensor() {

  // Send trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  // Measure echo duration with a timeout of 25 ms
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 25000);

  if (duration == 0) {

    obstacleDistance = -1.0;
    obstacleDetected = false;

    Serial.println("[SENSOR] No echo received. Check sensor or target.");

    return;
  }

  // Calculate distance in centimetres
  obstacleDistance = duration * 0.0343 / 2.0;

  // Check obstacle threshold
  obstacleDetected =
      (obstacleDistance > 0 &&
       obstacleDistance < OBSTACLE_DISTANCE_CM);

  // Print distance continuously
  Serial.print("[SENSOR] Distance: ");
  Serial.print(obstacleDistance, 1);
  Serial.print(" cm | Status: ");

  if (obstacleDetected) {
    Serial.println("OBSTACLE DETECTED");
  } else {
    Serial.println("PATH CLEAR");
  }
}

// =====================================================
// MISSION CONTROLLER
// =====================================================

void handleMission() {

  switch (currentState) {

    case IDLE:

      stopRobot();

      if (startPressed) {

        currentStation = 0;

        Serial.println("[MISSION] Starting warehouse delivery.");

        changeState(START_MISSION);
      }

      break;

    case START_MISSION:

      Serial.println("[MISSION] Mission initialized.");
      Serial.print("[MISSION] Total stations: ");
      Serial.println(TOTAL_STATIONS);

      changeState(FOLLOW_ROUTE);

      break;

    case FOLLOW_ROUTE:

      if (obstacleDetected) {

        stopRobot();

        obstacleStopStart = millis();

        Serial.print("[OBSTACLE] Detected at ");
        Serial.print(obstacleDistance, 1);
        Serial.println(" cm.");

        changeState(OBSTACLE_STOP);

        break;
      }

      moveForward();

      if (stationPressed) {

        stopRobot();

        currentStation++;

        Serial.print("[STATION] Arrived at station ");
        Serial.print(currentStation);
        Serial.print(" of ");
        Serial.println(TOTAL_STATIONS);

        stationServiceStart = millis();

        changeState(STATION_SERVICE);
      }

      break;

    case OBSTACLE_STOP:

      stopRobot();

      // Resume only after a valid, clear measurement.
      if (obstacleDistance > OBSTACLE_DISTANCE_CM) {

        Serial.print("[OBSTACLE] Path clear at ");
        Serial.print(obstacleDistance, 1);
        Serial.println(" cm.");

        Serial.println("[MISSION] Resuming route.");

        changeState(FOLLOW_ROUTE);
      }

      else if (millis() - obstacleStopStart >=
               OBSTACLE_TIMEOUT_MS) {

        Serial.println("[FAULT] Obstacle timeout exceeded.");

        changeState(FAULT);
      }

      break;

    case STATION_SERVICE:

      stopRobot();

      serviceStation();

      if (millis() - stationServiceStart >=
          STATION_SERVICE_MS) {

        Serial.println("[STATION] Service completed.");

        if (currentStation >= TOTAL_STATIONS) {

          changeState(MISSION_COMPLETE);
        }

        else {

          Serial.println("[MISSION] Proceeding to next station.");

          changeState(FOLLOW_ROUTE);
        }
      }

      break;

    case MISSION_COMPLETE:

      stopRobot();

      // Report completion once on entering this state.
      // See changeState() below for the state-entry handler.
      if (startPressed) {

        currentStation = 0;

        Serial.println("[MISSION] Starting a new mission.");

        changeState(START_MISSION);
      }

      break;

    case FAULT:

      stopRobot();

      break;
  }
}

// =====================================================
// SIMULATED MOTION CONTROLLER
// =====================================================

void moveForward() {

  if (!robotIsMoving) {

    robotIsMoving = true;

    Serial.println("[MOTION] Moving forward.");
  }
}

void stopRobot() {

  if (robotIsMoving) {

    robotIsMoving = false;

    Serial.println("[MOTION] Robot stopped.");
  }
}

// =====================================================
// STATION SERVICE
// =====================================================

void serviceStation() {

  static int lastReportedStation = -1;

  if (lastReportedStation != currentStation) {

    Serial.print("[STATION] Processing station ");
    Serial.println(currentStation);

    lastReportedStation = currentStation;
  }
}

// =====================================================
// STATE MANAGEMENT
// =====================================================

void changeState(RobotState newState) {

  if (currentState == newState) {
    return;
  }

  Serial.print("[STATE] ");
  Serial.print(stateName(currentState));
  Serial.print(" -> ");
  Serial.println(stateName(newState));

  currentState = newState;

  // State-entry actions
  if (newState == MISSION_COMPLETE) {

    Serial.println("[MISSION] All stations completed.");
    Serial.println("[MISSION] Delivery mission successful.");
  }

  if (newState == FAULT) {

    Serial.println("[FAULT] Robot requires a simulation restart.");
  }

  updateLEDs();
}

// =====================================================
// LED INDICATORS
// =====================================================

void updateLEDs() {

  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RED_LED, LOW);

  switch (currentState) {

    case IDLE:
      digitalWrite(YELLOW_LED, HIGH);
      break;

    case START_MISSION:
    case FOLLOW_ROUTE:
      digitalWrite(GREEN_LED, HIGH);
      break;

    case OBSTACLE_STOP:
    case STATION_SERVICE:
    case MISSION_COMPLETE:
      digitalWrite(YELLOW_LED, HIGH);
      break;

    case FAULT:
      digitalWrite(RED_LED, HIGH);
      break;
  }
}

// =====================================================
// STATE NAME CONVERSION
// =====================================================

const char* stateName(RobotState state) {

  switch (state) {

    case IDLE:
      return "IDLE";

    case START_MISSION:
      return "START_MISSION";

    case FOLLOW_ROUTE:
      return "FOLLOW_ROUTE";

    case OBSTACLE_STOP:
      return "OBSTACLE_STOP";

    case STATION_SERVICE:
      return "STATION_SERVICE";

    case MISSION_COMPLETE:
      return "MISSION_COMPLETE";

    case FAULT:
      return "FAULT";

    default:
      return "UNKNOWN";
  }
}

// =====================================================
// STATUS DIAGNOSTICS
// =====================================================

void printStatus() {

  Serial.print("[STATUS] Current robot state: ");
  Serial.println(stateName(currentState));
}

// =====================================================
// END OF PROGRAM
// =====================================================

