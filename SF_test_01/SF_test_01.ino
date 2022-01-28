#include <Arduino_LSM9DS1.h>
#include <SensorFusion.h>

SF fusion;

float gx, gy, gz, ax, ay, az;
float pitch, roll, yaw;
float deltat;

void setup() {
  Serial.begin(9600); //9600
  if (!IMU.begin()) {Serial.println("Failed to initialize IMU!");}
}

void loop() {

  if (IMU.gyroscopeAvailable()) {IMU.readGyroscope(gx, gy, gz);}
  if (IMU.accelerationAvailable()) {IMU.readAcceleration(ax, ay, az);}
  deltat = fusion.deltatUpdate(); //this have to be done before calling the fusion update
  //choose only one of these two:
  //fusion.MahonyUpdate(gx*PI/180, gy*PI/180, gz*PI/180, ax, ay, az, deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
  fusion.MadgwickUpdate(gx*PI/180, gy*PI/180, gz*PI/180, ax, ay, az, deltat);  //else use the magwick, it is slower but more accurate

  pitch = fusion.getPitch();
  //roll = fusion.getRoll();    //you could also use getRollRadians() ecc
  //yaw = fusion.getYaw();

  //Serial.print("Pitch:\t");
  Serial.println(pitch);
  //Serial.print("Roll:\t"); Serial.println(roll);
  //Serial.print("Yaw:\t"); Serial.println(yaw);
  //Serial.println();
}
