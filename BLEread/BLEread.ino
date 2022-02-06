// Capstone Arduino Program

#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

// Variable Initialization
byte accelX = 1;
byte accelY = 1;
byte accelZ = 1;
float x, y, z;
byte data[3] = { accelX, accelY, accelZ };

// Instantiate BLE Peripheral Service
//BLEService accService("2a675dfb-a1b0-4c11-9ad1-031a84594196");
BLEService accService("2ba18a92-0427-4579-9884-a3c8e53dad59");
// Instantiate Service Characteristics
BLECharacteristic customXChar("d81c825a-4849-4606-9b43-54214c5cd8cd", BLERead | BLENotify, 1); // X coordinate value
BLECharacteristic customYChar("f86a30d0-3af0-413f-a21b-26a2ab665933", BLERead | BLENotify, 1); // Y coordinate value
BLECharacteristic customZChar("7f15286c-ac14-47e7-ac68-26151a6a9e6f", BLERead | BLENotify, 1); // Z coordinate value

void setup() {
  IMU.begin();
  Serial.begin(9600);
  while (!Serial);

  // Pin Outputs
  pinMode(LED_BUILTIN, OUTPUT);

  // Check Initialization of BLE service
  if (!BLE.begin()) {
    Serial.println("BLE failed to Initiate");
    while (1);
  }

  // Set the Arduino Name that will show up over BLE
  BLE.setDeviceName("yo mama");
  BLE.setConnectable(true);

  // Add Service Characteristics
  accService.addCharacteristic(customXChar);
  accService.addCharacteristic(customYChar);
  accService.addCharacteristic(customZChar);


  BLE.setLocalName("yo mama");
  BLE.setAdvertisedServiceUuid(accService.uuid());
  
  // Add Service w/ Characteristics attached to BLE program

  // Setup BLE write channel for values
  customXChar.writeValue(accelX);
  customYChar.writeValue(accelY);
  customYChar.writeValue(accelZ);

  // Setup BLE adveristing
  BLE.advertise();

  Serial.println("Bluetooth device is now active, waiting for connections...");
}


void loop() {

  BLEDevice central = BLE.central();
  bool m = true; 
  while (central.connected()) {
    if(m) { 
      Serial.print("Arduino Connected to ");
      Serial.println(central.localName());
      m = false; 
    }
    digitalWrite(LED_BUILTIN, HIGH);
    read_Accel();
    byte data[3] = { accelX, accelY, accelZ };
    //BLE.setManufacturerData(data, 3);
    customXChar.writeValue(data[0]);
    customYChar.writeValue(data[1]);
    customZChar.writeValue(data[2]);

    Serial.print("At Main Function");
    Serial.println("");
    Serial.print("X: ");
    Serial.print(accelX);
    Serial.print(" Y: ");
    Serial.print(accelY);
    Serial.print(" Z: ");
    Serial.print(accelZ);
    Serial.println("");
  }

  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Arduino Disconnected");
  delay(500);
}

// Read the acceleration of the arduino from the IMU
void read_Accel() {
  // Check if IMU is available
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    accelX = (1 + x) * 100;
    accelY = (1 + y) * 100;
    accelZ = (1 + z) * 100;
  }
}
