// Capstone Arduino Program 
#include <ArduinoBLE.h>
//need mbed.h for clock functions
#include <mbed.h>
#include "Wire.h"
#include <SPI.h>

// See also LSM9DS1 Register Map and Descriptions, http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00103319.pdf 
// Accelerometer and Gyroscope registers
#define LSM9DS1XG_WHO_AM_I          0x0F  // should return 0x68
#define LSM9DS1XG_CTRL_REG1_G       0x10
#define LSM9DS1XG_STATUS_REG        0x17
#define LSM9DS1XG_OUT_X_L_G         0x18
#define LSM9DS1XG_CTRL_REG4         0x1E
#define LSM9DS1XG_CTRL_REG5_XL      0x1F
#define LSM9DS1XG_CTRL_REG6_XL      0x20
#define LSM9DS1XG_CTRL_REG8         0x22
#define LSM9DS1XG_CTRL_REG9         0x23
#define LSM9DS1XG_CTRL_REG10        0x24
#define LSM9DS1XG_OUT_X_L_XL        0x28
#define LSM9DS1XG_FIFO_CTRL         0x2E
#define LSM9DS1XG_FIFO_SRC          0x2F

#define LSM9DS1XG_ADDRESS 0x6B  //  Device address

typedef struct
{
  int16_t ax;
  int16_t ay; 
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
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
//===================================================================================================================
//====== Set of useful function to access acceleration and gyroscope data
//===================================================================================================================

enum Ascale: uint8_t {AFS_2G=0, AFS_16G, AFS_4G, AFS_8G}; // set of allowable accel full scale settings
enum Aodr: uint8_t {AODR_PowerDown=0, AODR_10Hz, AODR_50Hz, AODR_119Hz, AODR_238Hz, AODR_476Hz, AODR_952Hz}; // set of allowable gyro sample rates
enum Abw: uint8_t {ABW_408Hz=0, ABW_211Hz, ABW_105Hz, ABW_50Hz}; // set of allowable accewl bandwidths
enum Gscale: uint8_t {GFS_245DPS=0, GFS_500DPS, GFS_NoOp, GFS_2000DPS}; // set of allowable gyro full scale settings
enum Godr: uint8_t {GODR_PowerDown=0, GODR_14_9Hz, GODR_59_5Hz, GODR_119Hz, GODR_238Hz, GODR_476Hz, GODR_952Hz}; // set of allowable gyro sample rates
enum Gbw: uint8_t {GBW_low=0, GBW_med, GBW_high, GBW_highest}; // set of allowable gyro data bandwidths (Hz): low: 14@238, 33@952; med: 29@238, 40@952; high: 63@238, 58@952; highest 78@238, 100@952

Ascale ascale = AFS_4G;      // accel full scale
Aodr aodr = AODR_238Hz;      // accel data sample rate
Abw abw = ABW_50Hz;          // accel data bandwidth
Gscale gscale = GFS_500DPS;  // gyro full scale
Godr godr = GODR_238Hz;      // gyro data sample rate
Gbw gbw = GBW_med;           // gyro data bandwidth

//define UUID
#define BLE_UUID_ARDUINO_TIMESTAMP                "1805"
#define BLE_UUID_ARDUINO_MEASUREMENTS             "2a675dfb-a1b0-4c11-9ad1-031a84594196" //"1e0f9d07-42fe-4b48-b405-38374e5f2d97"
#define BLE_UUID_SENSOR_DATA                      "d80de551-8403-4bae-9f78-4d2af89ff17b"
#define BLE_DEVICE_NAME                           "Arduino Nano 33 BLE"
#define BLE_LOCAL_NAME                            "Arduino 2 (Nano 33 BLE)"
#define BLE_LED_PIN                               LED_BUILTIN
//Measurement Characteristics
BLEService Arduino_measurements(BLE_UUID_ARDUINO_MEASUREMENTS);
BLECharacteristic Sensor_data(BLE_UUID_SENSOR_DATA , BLERead|BLENotify, sizeof accel_gyro_data.bytes);

uint8_t readByte(uint8_t address, uint8_t subAddress) {
  uint8_t data; // `data` will store the register data   
  Wire1.beginTransmission(address);         // Initialize the Tx buffer
  Wire1.write(subAddress);                  // Put slave register address in Tx buffer
  Wire1.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire1.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
  data = Wire1.read();                      // Fill Rx buffer with result
  return data;                              // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {
  Wire1.beginTransmission(address);   // Initialize the Tx buffer
  Wire1.write(subAddress);            // Put slave register address in Tx buffer
  Wire1.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire1.requestFrom(address, count);  // Read bytes from slave register address 
  while (Wire1.available()){ 
    dest[i++] = Wire1.read(); 
    }         // Put read results in the Rx buffer
}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
  Wire1.beginTransmission(address);  // Initialize the Tx buffer
  Wire1.write(subAddress);           // Put slave register address in Tx buffer
  Wire1.write(data);                 // Put data in Tx buffer
  Wire1.endTransmission();           // Send the Tx buffer
}

//function declarations:
bool setupBleMode();
void bleTask();
void bleConnectHandler(BLEDevice central);
void bleDisconnectHandler(BLEDevice central);
void initializeClock(void);
void read_Accel_Gyro();

void setup() {
  Wire1.begin();
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG8, 0x05);
  delay(100);
  Serial.begin(9600); 
  
  // Pin Outputs 
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  // initialize LSM9DS1
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG4, 0x38); // enable the 3-axes of the gyroscope
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG1_G, godr << 5 | gscale << 3 | gbw);
  delay(200);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG5_XL, 0x38); // enable the three axes of the accelerometer
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG6_XL, aodr << 5 | ascale << 3 | 0x04 |abw);
  delay(200);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG8, 0x44); // enable block data update, allow auto-increment during multiple byte read
  
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

  //Sensor Data
  Arduino_measurements.addCharacteristic(Sensor_data);
  BLE.addService(Arduino_measurements);

  //set BLE event handlers; this is like switch-case in c++
  BLE.setEventHandler(BLEConnected, bleConnectHandler);
  BLE.setEventHandler(BLEDisconnected, bleDisconnectHandler);

  //start advertising
  BLE.advertise();
  return true;
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
  read_Accel_Gyro();
  
  static uint32_t previousMillis = 0;
  uint32_t currentMillis = millis();

  if (currentMillis - previousMillis >= BLE_UPDATE_INTERVAL)
  {
    previousMillis = currentMillis;
    BLE.poll();
  }
}

void read_Accel_Gyro() {
  if (readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_STATUS_REG) & 0x01) { 
    readAccelint16();  // Read the x/y/z adc values
    //Serial.print("a = ["); Serial.print(accel_gyro_data.data_total.ax); Serial.print(", "); Serial.print(accel_gyro_data.data_total.ay); Serial.print(", "); Serial.print(accel_gyro_data.data_total.az); Serial.println("] /32768");
  }
  
  if (readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_STATUS_REG) & 0x02) {
    readGyroint16();  // Read the x/y/z adc values 
    Serial.print("g = ["); Serial.print(accel_gyro_data.data_total.gx); Serial.print(", "); Serial.print(accel_gyro_data.data_total.gy); Serial.print(", "); Serial.print(accel_gyro_data.data_total.gz); Serial.println("] /32768");
  }
  Sensor_data.writeValue(accel_gyro_data.bytes, sizeof accel_gyro_data.bytes);
}


void readAccelint16(){
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_XL, 6, &rawData[0]);  // Read the six raw data registers into data array
  accel_gyro_data.data_total.ax = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
  accel_gyro_data.data_total.ay = ((int16_t)rawData[3] << 8) | rawData[2];
  accel_gyro_data.data_total.az = ((int16_t)rawData[5] << 8) | rawData[4];
}
    
void readGyroint16(){
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_G, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  accel_gyro_data.data_total.gx = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
  accel_gyro_data.data_total.gy = ((int16_t)rawData[3] << 8) | rawData[2];
  accel_gyro_data.data_total.gz = ((int16_t)rawData[5] << 8) | rawData[4];
}
