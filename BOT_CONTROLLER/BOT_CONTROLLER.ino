// Test controller script

#include <ArduinoBLE.h>
#include <Arduino_APDS9960.h>
#include <Arduino_BMI270_BMM150.h>

const char* deviceServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* commandCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";
const char* velocityCharacteristicUuid = "19b10002-e8f2-537e-4f6c-d104768a1214";

#define BUTTON_PIN 2
#define LED_PIN LED_BUILTIN

const int ALPHA = 0.97; // Complimentary filter coefficient

// timestep function
float lastTime = 0;

// complementary filter function
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;

float roll_accel;
float pitch_accel;
float pitch_gyro = 0;
float roll_gyro = 0;

float roll_filter = 0;
float pitch_filter = 0;

int gesture = -1; // default no gesture detected
byte command = 1; // default of E-stop activated - motors default off

const unsigned long INTERVAL = 100; // Interval for sending data, in ms
unsigned long prevData = 0; // track when the last package was sent to ensure under 500ms

// reset for estop and gesture recognition
unsigned long resetStart = 0;
bool resetLatched = false;

// define a struct for sending the individual motor speeds
struct MotorCommand {
  int leftSpeed;
  int rightSpeed;
};

void setup() {
  Serial.begin(9600);
  while (!Serial);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  
  if (!APDS.begin()) {
    Serial.println("* Error initializing APDS9960 sensor!");
  } 

  APDS.setGestureSensitivity(80); 
  
  if (!BLE.begin()) {
    Serial.println("* Starting Bluetooth® Low Energy module failed!");
    while (1);
  }

  // initialize IMU acceleration and gyroscope sensors
  if (!IMU.begin()){
    Serial.println("Error initializing IMU sensor.");    
  }
  
  BLE.setLocalName("Nano 33 BLE (Central)"); 

  Serial.println("Arduino Nano 33 BLE Sense (Central Device)");
  Serial.println(" ");
}

void loop() {
  float dt = timestep(millis());
  connectToPeripheral();

  gestureSteering(dt);
}

void connectToPeripheral(){
  BLEDevice peripheral;
  
  Serial.println("- Discovering peripheral device...");

  do
  {
    BLE.scanForUuid(deviceServiceUuid);
    peripheral = BLE.available();
  } while (!peripheral);
  
  if (peripheral) {
    Serial.println("* Peripheral device found!");
    Serial.print("* Device MAC address: ");
    Serial.println(peripheral.address());
    Serial.print("* Device name: ");
    Serial.println(peripheral.localName());
    Serial.print("* Advertised service UUID: ");
    Serial.println(peripheral.advertisedServiceUuid());
    Serial.println(" ");
    BLE.stopScan();
    controlPeripheral(peripheral);
  }
}

void controlPeripheral(BLEDevice peripheral) {
  Serial.println("- Connecting to peripheral device...");

  if (peripheral.connect()) {
    Serial.println("* Connected to peripheral device!");
    Serial.println(" ");
  } else {
    Serial.println("* Connection to peripheral device failed!");
    Serial.println(" ");
    return;
  }

  Serial.println("- Discovering peripheral device attributes...");
  if (peripheral.discoverAttributes()) {
    Serial.println("* Peripheral device attributes discovered!");
    Serial.println(" ");
  } else {
    Serial.println("* Peripheral device attributes discovery failed!");
    Serial.println(" ");
    peripheral.disconnect();
    return;
  }

  BLECharacteristic commandCharacteristic = peripheral.characteristic(commandCharacteristicUuid);
  BLECharacteristic velocityCharacteristic = peripheral.characteristic(velocityCharacteristicUuid);

  if (!commandCharacteristic || !commandCharacteristic.canWrite()) {
    Serial.println("* Invalid command characteristic. Disconnecting...");
    peripheral.disconnect();
    return;
  }
  if (!velocityCharacteristic || !velocityCharacteristic.canWrite()) {
    Serial.println("* Invalid velocity characteristic. Disconnecting...");
    peripheral.disconnect();
    return;
  }
  
  while (peripheral.connected()) {

    BLE.poll();

    float dt = timestep(millis());

    if (detectGesture()) {
      resetLatched = true;
      resetStart = millis();
    }

    // Check for Estop activation
    if (digitalRead(BUTTON_PIN) == LOW) {
      command = 1; // ESTOP
    }
    // Check if any gesture has been detected, if so, reset the Estop
    else if (resetLatched) {
      command = 2;

      if (millis() - resetStart > 300) {
        resetLatched = false;
      }
    }
    // If no Estop, run the motors as normal and send velocity commands
    else {
      command = 0; // normal operation
    }

    // send data at the specified time interval
    if (millis() - prevData > INTERVAL) {

      commandCharacteristic.writeValue(command); // send estop or reset command

      // only send motor speeds when not estopped
      if (command == 0){
        MotorCommand velCmd = gestureSteering(dt);

        velocityCharacteristic.writeValue( (byte*)&velCmd, sizeof(velCmd));

        Serial.print("Left: ");
        Serial.print(velCmd.leftSpeed);
        Serial.print(" Right: ");
        Serial.println(velCmd.rightSpeed);
      }

      prevData = millis();
    }

    digitalWrite(LED_PIN, LOW); // Keep light on to indicate normal operation
  }
  Serial.println("- Peripheral device disconnected!");
}

// Look for any gesture on the controller to reset the Estop
bool detectGesture() {
  if (APDS.gestureAvailable()) {
    int gesture = APDS.readGesture();

    switch (gesture) {
      case GESTURE_UP:
      case GESTURE_DOWN:
      case GESTURE_LEFT:
      case GESTURE_RIGHT:
        Serial.println(" Gesture Found: Starting...");
        return true;
      default:
        Serial.println("No gesture detected");
        break;
      }
    }
    return false;
}

// timestep function
// returns timestep
float timestep(unsigned long currentTime){

  float dt = (currentTime - lastTime) / 1000;
  lastTime = currentTime;
  return dt;
}

// complementary filter for velocity control
// returns a roll angle
float complementaryRoll(float dt){
  if (IMU.accelerationAvailable()){

    // read x,y,z acceleration from IMU
    IMU.readAcceleration(accel_x, accel_y, accel_z);

    // calculate roll angle using Arduino math library
    roll_accel = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * 180 / PI;
  }

  if (IMU.gyroscopeAvailable()){

    // read x,y,z gyroscope values from IMU
    IMU.readGyroscope(gyro_x, gyro_y, gyro_z);

    // new angle = old angle + (angular vel x dt)
    roll_gyro = roll_gyro + (gyro_x * dt); // changed from y to x
  }
 
  roll_filter = ALPHA * (roll_filter + gyro_x*dt) + (1-ALPHA) * roll_accel;

  return roll_filter;
}

// complementary filter for velocity control
// returns a pitch angle
float complementaryPitch(float dt){
  if (IMU.accelerationAvailable()){

    // read x,y,z acceleration from IMU
    IMU.readAcceleration(accel_x, accel_y, accel_z);

    // calculate pitch angle
    pitch_accel = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180 / PI;
  }

  if (IMU.gyroscopeAvailable()){

    // read x,y,z gyroscope values from IMU
    IMU.readGyroscope(gyro_x, gyro_y, gyro_z);

    // integrate gyro around Y-axis for pitch
    pitch_gyro = pitch_gyro + (gyro_y * dt);
  }
 
  pitch_filter = ALPHA * (pitch_filter + gyro_y*dt) + (1-ALPHA) * pitch_accel;

  return pitch_filter;
}

// calculates velocities based on roll and pitch pose to send to receiver
// uses complementaryPitch, complementaryRoll
MotorCommand gestureSteering(float dt){
  MotorCommand cmd;

  float pitch_angle = complementaryPitch(dt);
  float roll_angle = complementaryRoll(dt);

  // deadband to avoid drift
  if (abs(pitch_angle) < 5) pitch_angle = 0;
  if (abs(roll_angle) < 5)  roll_angle = 0;

  // map tilt angle to motor speed
  float forward = map(constrain(pitch_angle, -30, 30), -30, 30, -255, 255);
  float turn    = map(constrain(roll_angle, -30, 30), -30, 30, -150, 150);

  // differential drive mixing
  cmd.leftSpeed  = constrain(forward + turn, -255, 255);
  cmd.rightSpeed = constrain(forward - turn, -255, 255);

  return cmd;
}


