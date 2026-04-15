// Test receiver script

#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>
#include <Arduino_HS300x.h>

#define IN1 2
#define IN2 4
#define ENA A2
#define IN3 5
#define IN4 6
#define ENB A1

#define LED_PIN LED_BUILTIN

// IR pair pin assigments for line following and obstacle detection
#define leftLine A3
#define rightLine A4
#define leftFront A5
#define rightFront A6

// Threshold integer for comparing IR in front of black or white surface - may need separate one for obstacle
const int lineThreshold = 4;
const int frontThreshold = 4;

// Add states for manual mode (gesture steering), line following, and obstacle detection
enum CarState {
  MANUAL,
  ESTOP
};

CarState state = ESTOP;  // Default of motors off

const char* deviceServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* commandCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";
const char* velocityCharacteristicUuid = "19b10002-e8f2-537e-4f6c-d104768a1214";

int gesture = -1;

// struct for motor speed commands
struct MotorCommand {
  int leftSpeed;
  int rightSpeed;
};

MotorCommand currentCommand = { 0, 0 };  // set default values for motors, off

BLEService gestureService(deviceServiceUuid);
BLEByteCharacteristic commandCharacteristic(commandCharacteristicUuid, BLERead | BLEWrite);
BLECharacteristic velocityCharacteristic(velocityCharacteristicUuid, BLERead | BLEWrite, sizeof(MotorCommand));


void setup() {
  Serial.begin(9600);
  while (millis() > 1000)
    ;  // don't wait for serial connection - battery power

  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Motor pins
  pinMode(IN1, OUTPUT);  // Input 1A
  pinMode(IN2, OUTPUT);  // Input 2A
  pinMode(ENA, OUTPUT);  // Enable pin 1/2
  pinMode(IN3, OUTPUT);  // Input 3A
  pinMode(IN4, OUTPUT);  // Input 4A
  pinMode(ENB, OUTPUT);  // Enable pin 3/4

  pinMode(RECEIVER, INPUT);  // IR Receiver
  pinMode(EMITTER, OUTPUT);  // IR Emitter

  stopMotors();  // Default motors to off state


  if (!BLE.begin()) {
    Serial.println("- Starting Bluetooth® Low Energy module failed!");
    while (1)
      ;
  }

  BLE.setLocalName("Arduino Nano 33 BLE (Peripheral)");
  BLE.setAdvertisedService(gestureService);

  gestureService.addCharacteristic(commandCharacteristic);
  gestureService.addCharacteristic(velocityCharacteristic);
  BLE.addService(gestureService);
  commandCharacteristic.writeValue(1);  // Default to 1 for ESTOP activation at start
  BLE.advertise();

  Serial.println("Nano 33 BLE (Peripheral Device)");
  Serial.println(" ");
}

void loop() {
  BLEDevice central = BLE.central();
  Serial.println("- Discovering central device...");

  if (central) {
    Serial.println("* Connected to central device!");

    while (central.connected()) {
      BLE.poll();

      if (commandCharacteristic.written()) {
        byte cmd = commandCharacteristic.value();

        Serial.print("New CMD: ");
        Serial.println(cmd);

        if (cmd == 1) {
          state = ESTOP;
        } else if (cmd == 2 && state == ESTOP) {
          state = MANUAL;
          Serial.println("Reset received: MANUAL");
        }
      }

      //velocity control
      if (velocityCharacteristic.written()) {
        velocityCharacteristic.readValue((byte*)&currentCommand, sizeof(currentCommand));

        Serial.print("Left speed: ");
        Serial.print(currentCommand.leftSpeed);
        Serial.print(" Right speed: ");
        Serial.println(currentCommand.rightSpeed);
      }

      // Check if the Estop has been activated
      if (state == ESTOP) {
        stopMotors();

        // RED LED = E-stop
        digitalWrite(LEDR, LOW);
        digitalWrite(LEDG, HIGH);
        digitalWrite(LEDB, HIGH);
      } else {
        runMotors(currentCommand.leftSpeed, currentCommand.rightSpeed);

        // GREEN LED = MANUAL
        digitalWrite(LEDR, HIGH);
        digitalWrite(LEDG, LOW);
        digitalWrite(LEDB, HIGH);
      }
    }
  }
  if (!central) {
    state = ESTOP;
    stopMotors();

    // Blink red LED continuously while disconnected
    if (millis() % 200 < 100) {
      digitalWrite(LEDR, LOW);  // ON
    } else {
      digitalWrite(LEDR, HIGH);  // OFF
    }

    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
  }
}

// Script to stop motors when bluetooth disconnected or Estop activated
// Sets motor speeds to zero
void stopMotors() {
  // set pwm pins to 0 to stop them
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// Function to drive the motors with the speeds from the controller
void runMotors(int leftSpeed, int rightSpeed) {

  // LEFT MOTOR
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, leftSpeed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, abs(leftSpeed));
  }

  // RIGHT MOTOR
  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, rightSpeed);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, abs(rightSpeed));
  }
}

// Battery Monitor - should this be on controller instead? would be easier to implement the LED... (more space)
void batteryMonitor() {
  // implement a voltage divider to 
}

// IR Pairs - line following function with IR pairs spaced 5cm apart 
// NEED TO CALIBRATE FIRST - PRINT RAW SENSOR VALUES
void lineFollowing() {

  // readthe values of both sides
  int leftValue = analogRead(leftLine);
  int rightValue = analogRead(rightLine);

  // compare each side to threshold to check if it is on the line (true if on line, false if not)
  bool leftOnLine = leftValue < lineThreshold; 
  bool rightOnLine = rightValue < lineThreshold;

  if (leftOnLine && rightOnLine) {
    runMotors(250, 250); // full speed ahead when straight
  } else if (leftOnLine && !rightOnLine) {
    runMotors(100, 180); // turn left a bit
  } else if (!leftOnLine && rightOnLine) {
    setMotorSpeed(180, 100); // turn right a bit
  } else {
    stopMotors(); // stop in place if line lost - move to manual mode?
  }
}

// Obstacle detection (similar logic to IR pairs)
// NEED TO CALIBRATE SENSORS FIRST
void obstacleDetection() {

  // readthe values of both sides
  int leftValue = analogRead(leftFront);
  int rightValue = analogRead(rightFront);

  // compare each side to threshold to check if obstacle is seen (higher reflectance)
  bool leftBlocked = leftValue > frontThreshold; 
  bool rightBlocked = rightValue > frontThreshold;

  if (!leftBlocked && !rightBlocked) {
    // Remain in line following mode - do nothing
    //runMotors(250, 250); // full speed ahead when straight
  } else if (leftBlocked && !rightBlocked) {
    // change mode?
    runMotors(100, 180); // turn right 90 deg and move forward a certain distance?
  } else if (!leftBlocked && rightBlocked) {
    // change mode again - obstacle detected mode?
    setMotorSpeed(180, 100); // turn left 90 deg and move forward a certain distance?
  } else {
    stopMotors(); // stop in place if line lost - move to manual mode?
  }
}

// Pose tracking - use gyro for angular velocity, wheel velocities, and encoders for distance readings to estimate distance from origin
// Send this data back to the controller to build a map
// Monitor temperature at the same time & attach temp readings for each position (sample at same rate)
// Build serial plot of distance and temp data - save to csv?
void positionAndHeatMap() {


}

