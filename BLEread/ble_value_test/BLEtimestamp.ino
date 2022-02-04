#include <ArduinoBLE.h>

//need mbed.h for clock functions
#include <mbed.h>

//define UUID
#define BLE_UUID_ARDUINO_TIMESTAMP                "1805"
#define BLE_UUID_DATE_TIME                        "2A08" //uuid presets
#define BLE_UUID_MILLISECONDS                     "C8F88594-2217-0CA6-8F06-A4270B675D69"

//date_time Struct
typedef struct __attribute__((packed))
{
  uint16_t year;
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


uint32_t MILLISECONDS;
#define BLE_DEVICE_NAME                           "Arduino Nano 33 BLE"
#define BLE_LOCAL_NAME                            "Arduino 1 (Nano 33 BLE)"

BLEService Arduino_timestamp(BLE_UUID_ARDUINO_TIMESTAMP);
BLECharacteristic dateTimeCharacteristic(BLE_UUID_DATE_TIME, BLERead | BLEWrite | BLENotify, sizeof dateTimeData.bytes);
BLEFloatCharacteristic millisecondsCharacteristic(BLE_UUID_MILLISECONDS, BLERead | BLENotify);

#define BLE_LED_PIN                               LED_BUILTIN

bool timeUpdated = false;
void setup()
{
  Serial.begin(9600);

  pinMode(BLE_LED_PIN, OUTPUT);

  initializeClock();

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


void loop()
{
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
  Arduino_timestamp.addCharacteristic(dateTimeCharacteristic);
  Arduino_timestamp.addCharacteristic(millisecondsCharacteristic);
  BLE.addService(Arduino_timestamp);

  // set the initial value for the characeristics
  dateTimeCharacteristic.writeValue(dateTimeData.bytes, sizeof dateTimeData.bytes);  
  //set BLE event handlers; this like switch-case in c++
  BLE.setEventHandler(BLEConnected, bleConnectHandler);
  BLE.setEventHandler(BLEDisconnected, bleDisconnectHandler);

  //set service and characteristic specific event handlers
  dateTimeCharacteristic.setEventHandler(BLEWritten, DateTimeWrittenHandler);

  //start advertising
  BLE.advertise();
  return true;
}


void bleTask()
{
#define BLE_UPDATE_INTERVAL 10
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

  dateTimeData.dateTime.year = now->tm_year + 1900;
  dateTimeData.dateTime.month = now->tm_mon + 1;
  dateTimeData.dateTime.day = now->tm_mday;
  dateTimeData.dateTime.hours = now->tm_hour;
  dateTimeData.dateTime.minutes = now->tm_min;
  dateTimeData.dateTime.seconds = now->tm_sec;
  
  timeUpdated = true;
}


void initializeClock(void)
{
  //Jan 1, 2022 00:00:00
  date_time_t dateTime = {2022, 1, 1, 0, 0, 0}; 
  setTime(dateTime);
}


void setTime(date_time_t time)
{
  //tm struct, mktime and set_time are predefined in C
  struct tm setTime;

  setTime.tm_mon = time.month - 1;      // month are from (0 - 11)
  setTime.tm_year = time.year - 1900;   // years since 1900
  setTime.tm_mday = time.day;           // day of month (0 - 31)
  setTime.tm_hour = time.hours;         // hour (0 - 23)
  setTime.tm_min = time.minutes;        // minutes (0 - 59)
  setTime.tm_sec = time.seconds;        // seconds (0 - 59)

  set_time(mktime(&setTime));
}
