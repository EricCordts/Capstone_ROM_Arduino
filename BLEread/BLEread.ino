// Capstone Arduino Program 

#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

//need mbed.h for clock functions
#include <mbed.h>


//define UUID
#define BLE_UUID_ARDUINO_TIMESTAMP                "1805"
#define BLE_UUID_DATE_TIME                        "315f50e2-55c9-4b10-8b46-6c66957b4d98"
#define BLE_UUID_MILLISECONDS                     "C8F88594-2217-0CA6-8F06-A4270B675D69"
#define BLE_UUID_ARDUINO_ACCEL                     "1e0f9d07-42fe-4b48-b405-38374e5f2d97"
#define BLE_UUID_ACCEL_X                          "d80de551-8403-4bae-9f78-4d2af89ff17b"
#define BLE_UUID_ACCEL_Y                          "fd32fada-2b1f-41b7-b9f0-2dd935cd23f3"
#define BLE_UUID_ACCEL_Z                          "7b97c302-a7f6-4d96-ac29-484187af83d7"
#define BLE_UUID_ARDUINO_GYRO                     "daf44635-bcc1-41b8-8837-c2492683898e"
#define BLE_UUID_GYRO_X                           "74045b34-3207-4d06-a90a-e4579694cca8"
#define BLE_UUID_GYRO_Y                           "fa0c104d-0824-49af-aba9-99d07609cd7d"
#define BLE_UUID_GYRO_Z                           "286f5f54-30fb-45ad-a44a-b1217bd9935e"
#define BLE_DEVICE_NAME                           "Arduino Nano 33 BLE"
#define BLE_LOCAL_NAME                            "Arduino 2 (Nano 33 BLE)"
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

// Variable Initialization
int accelX=1;
int accelY=1;
int accelZ=1;
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;

//Timestamp Characteristics
BLEService Arduino_timestamp(BLE_UUID_ARDUINO_TIMESTAMP);
BLECharacteristic dateTimeCharacteristic(BLE_UUID_DATE_TIME, BLERead | BLEWrite | BLENotify, sizeof dateTimeData.bytes);
BLEFloatCharacteristic millisecondsCharacteristic(BLE_UUID_MILLISECONDS, BLERead | BLENotify);

//Accelerometer Characteristics
BLEService Arduino_Accel(BLE_UUID_ARDUINO_ACCEL);
BLEUnsignedIntCharacteristic AccelXChar(BLE_UUID_ACCEL_X , BLERead | BLENotify); // X coordinate value
BLEUnsignedIntCharacteristic AccelYChar(BLE_UUID_ACCEL_Y , BLERead | BLENotify); // Y coordinate value
BLEUnsignedIntCharacteristic AccelZChar(BLE_UUID_ACCEL_Z , BLERead | BLENotify); // Z coordinate value

//Gyroscope Characteristics 
BLEService Arduino_Gyro(BLE_UUID_ARDUINO_GYRO);
BLEFloatCharacteristic GyroXChar(BLE_UUID_GYRO_X, BLERead|BLENotify);
BLEFloatCharacteristic GyroYChar(BLE_UUID_GYRO_Y, BLERead|BLENotify);
BLEFloatCharacteristic GyroZChar(BLE_UUID_GYRO_Z, BLERead|BLENotify);

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
  timeTask();
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
  BLE.setAdvertisedService(Arduino_timestamp);

  //add characteristics and service
  //Timestamp
  Arduino_timestamp.addCharacteristic(dateTimeCharacteristic);
  Arduino_timestamp.addCharacteristic(millisecondsCharacteristic);
  BLE.addService(Arduino_timestamp);

  //Accelerometer
  Arduino_Accel.addCharacteristic(AccelXChar);
  Arduino_Accel.addCharacteristic(AccelYChar);
  Arduino_Accel.addCharacteristic(AccelZChar);
  BLE.addService(Arduino_Accel);

  //Gyroscope
  Arduino_Gyro.addCharacteristic(GyroXChar);
  Arduino_Gyro.addCharacteristic(GyroYChar);
  Arduino_Gyro.addCharacteristic(GyroZChar);
  BLE.addService(Arduino_Gyro);
  
  // set the initial value for the characeristics
  dateTimeCharacteristic.writeValue(dateTimeData.bytes, sizeof dateTimeData.bytes);  
  AccelXChar.writeValue(accelX);
  AccelYChar.writeValue(accelY);
  AccelZChar.writeValue(accelZ);
  GyroXChar.writeValue(1.00);
  GyroYChar.writeValue(1.00);
  GyroZChar.writeValue(1.00);

  //set BLE event handlers; this is like switch-case in c++
  BLE.setEventHandler(BLEConnected, bleConnectHandler);
  BLE.setEventHandler(BLEDisconnected, bleDisconnectHandler);

  //set service and characteristic specific event handlers
  dateTimeCharacteristic.setEventHandler(BLEWritten, DateTimeWrittenHandler);

  //start advertising
  BLE.advertise();
  return true;
}

//Handler Functions
void DateTimeWrittenHandler(BLEDevice central, BLECharacteristic bleCharacteristic)
{
  dateTimeCharacteristic.readValue(dateTimeData.bytes, sizeof dateTimeData.bytes);
  setTime(dateTimeData.dateTime);
}

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
  read_Accel();
  read_Gyro();
  static uint32_t previousMillis = 0;
  uint32_t currentMillis = millis();
  //Milliseconds Value
  millisecondsCharacteristic.writeValue(currentMillis%1000);

  if (currentMillis - previousMillis >= BLE_UPDATE_INTERVAL)
  {
    previousMillis = currentMillis;
    BLE.poll();
  }

  if (timeUpdated)
  {
    timeUpdated = false;
    dateTimeCharacteristic.writeValue(dateTimeData.bytes, sizeof dateTimeData.bytes);
  }
}

void read_Accel() {
  if (IMU.accelerationAvailable())
  {
    IMU.readAcceleration(accel_x, accel_y, accel_z);
    accelX = (1+accel_x)*100;
    accelY = (1+accel_y)*100;
    accelZ = (1+accel_z)*100;
  }

  AccelXChar.writeValue(accelX);
  AccelYChar.writeValue(accelY);
  AccelZChar.writeValue(accelZ);
}

void read_Gyro(){
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
  }
  
  GyroXChar.writeValue(gyro_x);
  GyroYChar.writeValue(gyro_y);
  GyroZChar.writeValue(gyro_z);

}
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
