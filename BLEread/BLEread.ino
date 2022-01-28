// Capstone Arduino Program 

#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

// Variable Initialization
int accelX=1;
int accelY=1;
int accelZ=1;
float x, y, z;

// Instantiate BLE Service
BLEService customService("1101");

// Instantiate Service Characteristics 
BLEUnsignedIntCharacteristic customXChar("2101", BLERead | BLENotify); // X coordinate value
BLEUnsignedIntCharacteristic customYChar("2102", BLERead | BLENotify); // Y coordinate value
BLEUnsignedIntCharacteristic customZChar("2103", BLERead | BLENotify); // Z coordinate value

void setup() {
  IMU.begin();
  Serial.begin(9600); 
  while (!Serial);
  
  // Pin Outputs 
  pinMode(LED_BUILTIN, OUTPUT);

  // Check Initialization of BLE service
  if (!BLE.begin()) {
    Serial.println("BLE failed to Initiate");
    delay(500);
    while (1);
  }

  // Set the Arduino Name that will show up over BLE
  BLE.setLocalName("Arduino Accelerometer");
  
  // Set Services
  BLE.setAdvertisedService(customService);

  // Add Service Characteristics
  customService.addCharacteristic(customXChar);
  customService.addCharacteristic(customYChar);
  customService.addCharacteristic(customZChar);

  // Add Service w/ Characteristics attached to BLE program
  BLE.addService(customService);

  // Setup BLE write channel for values 
  customXChar.writeValue(accelX);
  customYChar.writeValue(accelY);
  customYChar.writeValue(accelZ);

  // Setup BLE adveristing 
  BLE.advertise();

  Serial.println("Bluetooth device is now active, waiting for connections...");
}


void loop() {
  // Define BLE Envrionement as the central device
  BLEDevice central = BLE.central();

  // TODO: Add comment 
  if (central) {
    
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);
    while (central.connected()) {
      delay(200);
      read_Accel();
      
      customXChar.writeValue(accelX);
      customYChar.writeValue(accelY);
      customZChar.writeValue(accelZ);
      
      Serial.print("At Main Function");
      Serial.println("");
      Serial.println("X: ");
      Serial.print(accelX);
      Serial.print(" Y: ");
      Serial.print(accelY);
      Serial.print(" Z: ");
      Serial.print(accelZ);
      Serial.println("");
    }
  }

  digitalWrite(LED_BUILTIN, LOW);
  Serial.print("Disconnected from central: ");
  Serial.println(central.address());
}

// Read the acceleration of the arduino from the IMU
void read_Accel() {
  // Check if IMU is available
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    accelX = (1+x)*100;
    accelY = (1+y)*100;
    accelZ = (1+z)*100;
  }
}
