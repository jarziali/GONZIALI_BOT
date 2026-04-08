// Controller BLE connection script
// Connects to car to send velocity commands and E-stop

#include <ArduinoBLE.h>
#include <Arduino_APDS9960.h>

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // initialize the Bluetooth® Low Energy hardware
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1);
  }
  // Initialize the gesture recognition
  if (!APDS.begin()) {
    Serial.println("Error initializing APDS-9960 sensor.");
  }

  Serial.println("Bluetooth® Low Energy Central - velocity control");

  // start scanning for peripherals
  BLE.scanForUuid("19b10000-e8f2-537e-4f6c-d104768a1214");

}

void loop() {
  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();

    if (peripheral.localName() != "CAR") {
      return;
    }

    // stop scanning
    BLE.stopScan();

    controlCar(peripheral);

    // peripheral disconnected, start scanning again
    BLE.scanForUuid("19b10000-e8f2-537e-4f6c-d104768a1214");
  }
}

void controlCar(BLEDevice peripheral) {
  // connect to the peripheral
  Serial.println("Connecting ...");

  if (peripheral.connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
  Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    Serial.println("Attributes discovered");
  } else {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }

  // retrieve the velocity characteristic
  BLECharacteristic velocityCharacteristic = peripheral.characteristic("19b10001-e8f2-537e-4f6c-d104768a1214");

  if (!velocityCharacteristic) {
    Serial.println("Peripheral does not have velocity characteristic!");
    peripheral.disconnect();
    return;
  } else if (!velocityCharacteristic.canWrite()) {
    Serial.println("Peripheral does not have a writable velocity characteristic!");
    peripheral.disconnect();
    return;
  }

  while (peripheral.connected()) {
    // while the peripheral is connected

    // read the button pin
    int buttonState = digitalRead(buttonPin);

    if (oldButtonState != buttonState) {
      // button changed
      oldButtonState = buttonState;

      if (buttonState) {
        Serial.println("ESTOP pressed");

        // button is pressed, write 0x01 to turn the motors off
        buttonCharacteristic.writeValue((byte)0x01);
      } 
      // change to else if and detect gesture - then reset button state? can that be done?
      else {
        Serial.println("ESTOP released");

        // button is released, write 0x00 to turn the motors back on
        buttonCharacteristic.writeValue((byte)0x00);
      }
    }
  }

  Serial.println("Peripheral disconnected");
}
