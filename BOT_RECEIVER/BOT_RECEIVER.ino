// BLE Receiver - CAR
// Receives steering/velocity information from the controller when in manual mode
// Should eventually send back position and mapping data to build temperature and obstacle map

#include <ArduinoBLE.h>

BLEService carService("19B10010-E8F2-537E-4F6C-D104768A1214"); // create service

// create button characteristic and allow remote device to read and write
BLEByteCharacteristic buttonCharacteristic("19B10012-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

// Trial of velocity command characteristic for receiver
BLEByteCharacteristic velocityCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1);
  }

  // set the local name peripheral advertises
  BLE.setLocalName("CAR");
  // set the UUID for the service this peripheral advertises:
  BLE.setAdvertisedService(carService);

  // add the characteristics to the service
  carService.addCharacteristic(buttonCharacteristic);
  carService.addCharacteristic(velocityCharacteristic);

  // add the service
  BLE.addService(carService);

  buttonCharacteristic.writeValue(0);
  velocityCharacteristic.writeValue(0);

  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");
}

void loop() {
  // poll for Bluetooth® Low Energy events
  BLE.poll();

  // Check if the E stop was activated
  if (buttonCharacteristic.value() == 1) {
    // button state changed, update characteristics
    buttonCharacteristic.writeValue(buttonValue);

    // Turn everything off - analogwrite low to motor pins

  }

  // continuously read the velocity data to send commands to the motors
  // figure out whether to do the math for either motor here or in the controller circuit

}
