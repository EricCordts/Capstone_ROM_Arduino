#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include "SFlibmod.h"

SF fusion;

float gx, gy, gz, ax, ay, az;
float roll=0, pitch=0, yaw=0, timet=0;
float deltat;

BLEService customService("0000181a-0000-1000-8000-00805f9b34fb");
//BLEFloatCharacteristic bleRoll("2101", BLERead | BLENotify);
BLEFloatCharacteristic blePitch("2102", BLERead | BLENotify);
//BLEFloatCharacteristic bleYaw("2103", BLERead | BLENotify);
BLEFloatCharacteristic bleTime("2104", BLERead | BLENotify);

void setup() {
  Serial.begin(9600); //9600
  if (!IMU.begin()) {Serial.println("Failed to initialize IMU!");}
  while (!Serial);

  pinMode(LED_BUILTIN, OUTPUT);

  if (!BLE.begin()) {
    Serial.println("BLE failed to Initiate");
    delay(500);
    while (1);
  }

  BLE.setLocalName("Arduino RPY 1");
  BLE.setAdvertisedService(customService);
  //customService.addCharacteristic(bleRoll);
  customService.addCharacteristic(blePitch);
  //customService.addCharacteristic(bleYaw);
  customService.addCharacteristic(bleTime);
  BLE.addService(customService);
  //bleRoll.writeValue(roll);
  blePitch.writeValue(pitch);
  //bleYaw.writeValue(yaw);
  bleTime.writeValue(timet);

  BLE.advertise();

  Serial.println("Bluetooth device is now active, waiting for connections...");
}

void loop() {

  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);
    while (central.connected()) {
      if (IMU.gyroscopeAvailable()) {IMU.readGyroscope(gx, gy, gz);} // get gyroscope data
      if (IMU.accelerationAvailable()) {IMU.readAcceleration(ax, ay, az);} // get accelerometer data
      deltat = fusion.deltatUpdate(); // call before fusion update
      timet = millis();
      
      fusion.MahonyUpdate(gx*PI/180, gy*PI/180, gz*PI/180, ax, ay, az, deltat);  // Mahony is suggested if the mcu is slow
      //fusion.MadgwickUpdate(gx*PI/180, gy*PI/180, gz*PI/180, ax, ay, az, deltat);  // Magwick is slower but more accurate

      //roll = fusion.getRoll();
      pitch = fusion.getPitch();
      //yaw = fusion.getYaw();

      //bleRoll.writeValue(roll);
      blePitch.writeValue(pitch);
      //bleYaw.writeValue(yaw);
      bleTime.writeValue(timet/1000);

      Serial.println("At Main Function");
      //Serial.print("Roll: ");
      //Serial.print(roll);
      Serial.print(" Pitch: ");
      Serial.print(pitch);
      //Serial.print(" Yaw: ");
      //Serial.println(yaw);
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
  Serial.print("Disconnected from central: ");
  Serial.println(central.address());
}
