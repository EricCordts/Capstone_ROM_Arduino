// Capstone Arduino Program 

#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

//need mbed.h for clock functions
#include <mbed.h>


//define UUID
#define BLE_UUID_ARDUINO_TIMESTAMP                "1805"
#define BLE_UUID_DATE_TIME                        "315f50e2-55c9-4b10-8b46-6c66957b4d98"
#define BLE_UUID_MILLISECONDS                     "C8F88594-2217-0CA6-8F06-A4270B675D69"
#define BLE_UUID_ARDUINO_MEASUREMENTS             "2a675dfb-a1b0-4c11-9ad1-031a84594196" //"1e0f9d07-42fe-4b48-b405-38374e5f2d97"
#define BLE_UUID_SENSOR_DATA                      "d80de551-8403-4bae-9f78-4d2af89ff17b"
#define BLE_DEVICE_NAME                           "Arduino Nano 33 BLE"
#define BLE_LOCAL_NAME                            "Arduino 1 (Nano 33 BLE)"
#define BLE_LED_PIN                               LED_BUILTIN

//date_time Struct
typedef struct __attribute__((packed))
{
  //uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
} date_time_t;

union date_time_data
{
  struct __attribute__((packed))
  {
    date_time_t dateTime;
  };
  uint8_t bytes[sizeof(date_time_t)];
};

union date_time_data dateTimeData;

typedef struct
{
  int16_t accelX;
  int16_t accelY; 
  int16_t accelZ;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
} data_t;
  
union measurement_data
{
  struct
  {
    data_t data_total;
  };
  uint8_t bytes[sizeof(data_t)];
};

union measurement_data accel_gyro_data;

// Variable Initialization
float fl_accel_x, fl_accel_y, fl_accel_z;
float fl_gyro_x, fl_gyro_y, fl_gyro_z;
/*
//Timestamp Characteristics
BLEService Arduino_timestamp(BLE_UUID_ARDUINO_TIMESTAMP);
BLECharacteristic dateTimeCharacteristic(BLE_UUID_DATE_TIME, BLERead | BLEWrite | BLENotify, sizeof dateTimeData.bytes);
BLEFloatCharacteristic millisecondsCharacteristic(BLE_UUID_MILLISECONDS, BLERead | BLENotify);
*/
//Measurement Characteristics
BLEService Arduino_measurements(BLE_UUID_ARDUINO_MEASUREMENTS);
BLECharacteristic Sensor_data(BLE_UUID_SENSOR_DATA , BLERead|BLENotify, sizeof accel_gyro_data.bytes);

//function declarations:
bool setupBleMode();
void bleTask();
void DateTimeWrittenHandler(BLEDevice central, BLECharacteristic bleCharacteristic);
void bleConnectHandler(BLEDevice central);
void bleDisconnectHandler(BLEDevice central);
void timeTask();
void initializeClock(void);
void setTime(date_time_t time);

bool timeUpdated = false;

void setup() {
  IMU.begin();
  Serial.begin(9600); 
  
  // Pin Outputs 
  pinMode(LED_BUILTIN, OUTPUT);

  // Check Initialization of BLE service
  if (!setupBleMode())
  {
    Serial.println(F("Failed to initialize BLE"));
    while (1);
  }
  else
  {
    Serial.println(F("BLE initialized. Waiting for client connection"));
  }
}

void loop() {
  //timeTask();
  bleTask();
}

bool setupBleMode()
{
  if (!BLE.begin())
  {
    return false;
  }

  //set device and local name 
  BLE.setDeviceName(BLE_DEVICE_NAME);
  BLE.setLocalName(BLE_LOCAL_NAME);

  //set serivce 
  BLE.setAdvertisedService(Arduino_measurements);

  //add characteristics and service
  //Timestamp
  /*
  Arduino_timestamp.addCharacteristic(dateTimeCharacteristic);
  Arduino_timestamp.addCharacteristic(millisecondsCharacteristic);
  BLE.addService(Arduino_timestamp);
  */
  //Sensor Data
  Arduino_measurements.addCharacteristic(Sensor_data);
  BLE.addService(Arduino_measurements);


  // set the initial value for the characeristics
  //dateTimeCharacteristic.writeValue(dateTimeData.bytes, sizeof dateTimeData.bytes);  

  //set BLE event handlers; this is like switch-case in c++
  BLE.setEventHandler(BLEConnected, bleConnectHandler);
  BLE.setEventHandler(BLEDisconnected, bleDisconnectHandler);

  //set service and characteristic specific event handlers
  //dateTimeCharacteristic.setEventHandler(BLEWritten, DateTimeWrittenHandler);

  //start advertising
  BLE.advertise();
  return true;
}
/*
//Handler Functions
void DateTimeWrittenHandler(BLEDevice central, BLECharacteristic bleCharacteristic)
{
  dateTimeCharacteristic.readValue(dateTimeData.bytes, sizeof dateTimeData.bytes);
  setTime(dateTimeData.dateTime);
}
*/
void bleConnectHandler(BLEDevice central)
{
  digitalWrite(BLE_LED_PIN, HIGH);
  Serial.print(F("Connected to central: "));
  Serial.println(central.address());
}

void bleDisconnectHandler(BLEDevice central)
{
  digitalWrite (BLE_LED_PIN, LOW);
  Serial.print(F("Disconnected from central: "));
  Serial.println(central.address());
}

void bleTask()
{
#define BLE_UPDATE_INTERVAL 10
  read_Accel_Gyro();
  
  static uint32_t previousMillis = 0;
  uint32_t currentMillis = millis();
  //Milliseconds Value
  //millisecondsCharacteristic.writeValue(currentMillis%1000);

  if (currentMillis - previousMillis >= BLE_UPDATE_INTERVAL)
  {
    previousMillis = currentMillis;
    BLE.poll();
  }
  /*
  if (timeUpdated)
  {
    timeUpdated = false;
    dateTimeCharacteristic.writeValue(dateTimeData.bytes, sizeof dateTimeData.bytes);
  }*/
}

void read_Accel_Gyro() {
  if (IMU.accelerationAvailable())
  {
    IMU.readAcceleration(fl_accel_x, fl_accel_y, fl_accel_z);
    accel_gyro_data.data_total.accelX = fl_accel_x*100;
    accel_gyro_data.data_total.accelY = fl_accel_y*100;
    accel_gyro_data.data_total.accelZ = fl_accel_z*100;
  }
  if (IMU.gyroscopeAvailable())
  {
    IMU.readGyroscope(fl_gyro_x, fl_gyro_y, fl_gyro_z);
    accel_gyro_data.data_total.gyroX = fl_gyro_x*10;
    accel_gyro_data.data_total.gyroY = fl_gyro_y*10;
    accel_gyro_data.data_total.gyroZ = fl_gyro_z*10;
  }
  
  Sensor_data.writeValue(accel_gyro_data.bytes, sizeof accel_gyro_data.bytes);
  Serial.println(sizeof accel_gyro_data.bytes);
  //serial print for float values
  Serial.print("Accelometer float data X: ");
  Serial.print(fl_accel_x);
  Serial.print(" Y: ");
  Serial.print(fl_accel_y);
  Serial.print(" Z: ");
  Serial.println(fl_accel_z);
  Serial.print("Gyroscope float data X: ");
  Serial.print(fl_gyro_x);
  Serial.print(" Y: ");
  Serial.print(fl_gyro_y);
  Serial.print(" Z: ");
  Serial.println(fl_gyro_z);
  
  
  //serial print for int16_t
  Serial.print("Accelometer data X: ");
  Serial.print(accel_gyro_data.data_total.accelX);
  Serial.print(" Y: ");
  Serial.print(accel_gyro_data.data_total.accelY);
  Serial.print(" Z: ");
  Serial.println(accel_gyro_data.data_total.accelZ);
  Serial.print("Gyroscope data X: ");
  Serial.print(accel_gyro_data.data_total.gyroX);
  Serial.print(" Y: ");
  Serial.print(accel_gyro_data.data_total.gyroY);
  Serial.print(" Z: ");
  Serial.println(accel_gyro_data.data_total.gyroZ);
  
}
/*
void timeTask()
{
  #define TIME_UPDATE_INTERVAL 1000
  static uint32_t previousMillis = 0;

  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis < TIME_UPDATE_INTERVAL)
  {
    return;
  }
  previousMillis = currentMillis;
  time_t currentTime = time(NULL);
  
  struct tm * now = localtime(&currentTime);

  dateTimeData.dateTime.month = now->tm_mon + 1;
  dateTimeData.dateTime.day = now->tm_mday;
  dateTimeData.dateTime.hours = now->tm_hour;
  dateTimeData.dateTime.minutes = now->tm_min;
  dateTimeData.dateTime.seconds = now->tm_sec;
  
  timeUpdated = true;
}

void initializeClock(void)
{
  //Jan 1 00:00:00
  date_time_t dateTime = {1, 1, 0, 0, 0}; 
  setTime(dateTime);
}

void setTime(date_time_t time)
{
  //tm struct, mktime and set_time are predefined in C
  struct tm setTime;

  setTime.tm_mon = time.month - 1;      // month are from (0 - 11)
  setTime.tm_mday = time.day;           // day of month (0 - 31)
  setTime.tm_hour = time.hours;         // hour (0 - 23)
  setTime.tm_min = time.minutes;        // minutes (0 - 59)
  setTime.tm_sec = time.seconds;        // seconds (0 - 59)

  set_time(mktime(&setTime));
}
*/
