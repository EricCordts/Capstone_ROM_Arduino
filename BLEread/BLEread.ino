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

// Set initial input parameters
enum Ascale: uint8_t {AFS_2G, AFS_16G, AFS_4G, AFS_8G}; // set of allowable accel full scale settings
enum Aodr: uint8_t {AODR_PowerDown, AODR_10Hz, AODR_50Hz, AODR_119Hz, AODR_238Hz, AODR_476Hz, AODR_952Hz}; // set of allowable gyro sample rates
enum Abw: uint8_t {ABW_408Hz, ABW_211Hz, ABW_105Hz, ABW_50Hz}; // set of allowable accewl bandwidths
enum Gscale: uint8_t {GFS_245DPS, GFS_500DPS, GFS_NoOp, GFS_2000DPS}; // set of allowable gyro full scale settings
enum Godr: uint8_t {GODR_PowerDown, GODR_14_9Hz, GODR_59_5Hz, GODR_119Hz, GODR_238Hz, GODR_476Hz, GODR_952Hz}; // set of allowable gyro sample rates
enum Gbw: uint8_t {GBW_low, GBW_med, GBW_high, GBW_highest}; // set of allowable gyro data bandwidths (Hz): low: 14@238, 33@952; med: 29@238, 40@952; high: 63@238, 58@952; highest 78@238, 100@952

//===================================================================================================================
//====== Set of useful function to access acceleration and gyroscope data
//===================================================================================================================

class LSM9DS1IMUClass {
  public:
    LSM9DS1IMUClass(){
      Wire1.begin();
      writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG8, 0x05);
      delay(100);
      //initLSM9DS1();
    };

    int getGscale(){ // Possible gyro scales (and their register bit settings) are: 245 DPS (00), 500 DPS (01), and 2000 DPS (11)
      switch (gscale) {
        case GFS_245DPS: return 245;
        case GFS_500DPS: return 500;
        case GFS_2000DPS: return 2000;
        //default: return 500;
      }
    }
    int getAscale(){ // Possible accelerometer scales (and their register bit settings) are: 2 Gs (00), 16 Gs (01), 4 Gs (10), and 8 Gs (11)
      switch (ascale) {
        case AFS_2G: return 2;
        case AFS_16G: return 16;
        case AFS_4G: return 4;
        case AFS_8G: return 8;
        //default: return 4;
      }
    }
    float getGres(){return (float)(getGscale()/32768.0);}
    float getAres(){return (float)(getAscale()/32768.0);}

    void configureAccelerometer(){ // configure the accelerometer
      writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG6_XL, aodr << 5 | ascale << 3 | 0x04 |abw);
      delay(200);
    }
    void configureGyroscope(){ // configure the gyroscope
      writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG1_G, godr << 5 | gscale << 3 | gbw);
      delay(200);
    }

    bool accelAvailable(){
      return readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_STATUS_REG) & 0x01;
    }
    bool gyroAvailable(){
      return readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_STATUS_REG) & 0x02;
    }

    void setAscale(Ascale arange){
      ascale = arange;
      configureAccelerometer();
    }
    void setGscale(Gscale grange){
      gscale = grange;
      configureGyroscope();
    }
    void setArate(Aodr arate){
      aodr = arate;
      configureAccelerometer();
    }
    void setGrate(Godr grate){
      godr = grate;
      configureGyroscope();
    }
    void setAbandwidth(Abw aband){
      abw = aband;
      configureAccelerometer();
    }
    void setGbandwidth(Gbw gband){
      gbw = gband;
      configureGyroscope();
    }

    void readAccelint16(int16_t* x, int16_t* y, int16_t* z){
      uint8_t rawData[6];  // x/y/z accel register data stored here
      readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_XL, 6, &rawData[0]);  // Read the six raw data registers into data array
      *x = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
      *y = ((int16_t)rawData[3] << 8) | rawData[2] ;
      *z = ((int16_t)rawData[5] << 8) | rawData[4] ;
    }
    
    void readGyroint16(int16_t* x, int16_t* y, int16_t* z){
      uint8_t rawData[6];  // x/y/z gyro register data stored here
      readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_G, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
      *x = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
      *y = ((int16_t)rawData[3] << 8) | rawData[2];
      *z = ((int16_t)rawData[5] << 8) | rawData[4];
    }
    
    void initLSM9DS1(){
      writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG4, 0x38); // enable the 3-axes of the gyroscope
      configureGyroscope();
      writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG5_XL, 0x38); // enable the three axes of the accelerometer
      configureAccelerometer();
      writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG8, 0x44); // enable block data update, allow auto-increment during multiple byte read
    }
    
    void selftestLSM9DS1();
    //void accelgyrocalLSM9DS1(int16_t * dest1, int16_t * dest2);

  private:
    Ascale ascale = Ascale::AFS_4G; // accel full scale
    Aodr aodr = Aodr::AODR_238Hz; // accel data sample rate
    Abw abw = Abw::ABW_50Hz; // accel data bandwidth
    Gscale gscale = Gscale::GFS_500DPS; // gyro full scale
    Godr godr = Godr::GODR_238Hz; // gyro data sample rate
    Gbw gbw = Gbw::GBW_med; // gyro data bandwidth
    uint8_t readByte(uint8_t address, uint8_t subAddress)
    {
      uint8_t data; // `data` will store the register data   
      Wire1.beginTransmission(address);         // Initialize the Tx buffer
      Wire1.write(subAddress);                  // Put slave register address in Tx buffer
      Wire1.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
      Wire1.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
      data = Wire1.read();                      // Fill Rx buffer with result
      return data;                              // Return data read from slave register
    }
    void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
    {
      Wire1.beginTransmission(address);   // Initialize the Tx buffer
      Wire1.write(subAddress);            // Put slave register address in Tx buffer
      Wire1.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
      uint8_t i = 0;
      Wire1.requestFrom(address, count);  // Read bytes from slave register address 
      while (Wire1.available()){ 
        dest[i++] = Wire1.read(); 
        }         // Put read results in the Rx buffer
    }
    void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
    {
      Wire1.beginTransmission(address);  // Initialize the Tx buffer
      Wire1.write(subAddress);           // Put slave register address in Tx buffer
      Wire1.write(data);                 // Put data in Tx buffer
      Wire1.endTransmission();           // Send the Tx buffer
    }
};

//define UUID
#define BLE_UUID_ARDUINO_TIMESTAMP                "1805"
#define BLE_UUID_ARDUINO_MEASUREMENTS             "2a675dfb-a1b0-4c11-9ad1-031a84594196" //"1e0f9d07-42fe-4b48-b405-38374e5f2d97"
#define BLE_UUID_SENSOR_DATA                      "d80de551-8403-4bae-9f78-4d2af89ff17b"
#define BLE_DEVICE_NAME                           "Arduino Nano 33 BLE"
#define BLE_LOCAL_NAME                            "Arduino 2 (Nano 33 BLE)"
#define BLE_LED_PIN                               LED_BUILTIN

LSM9DS1IMUClass imu; //declare IMU



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

// Variable Initialization
//int16_t ax, ay, az, gx, gy, gz;
//Measurement Characteristics
BLEService Arduino_measurements(BLE_UUID_ARDUINO_MEASUREMENTS);
BLECharacteristic Sensor_data(BLE_UUID_SENSOR_DATA , BLERead|BLENotify, sizeof accel_gyro_data.bytes);

//function declarations:
bool setupBleMode();
void bleTask();
void bleConnectHandler(BLEDevice central);
void bleDisconnectHandler(BLEDevice central);
void initializeClock(void);

void setup() {
  //imu.initLSM9DS1();
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
  //read_Accel_Gyro();
  
  static uint32_t previousMillis = 0;
  uint32_t currentMillis = millis();

  if (currentMillis - previousMillis >= BLE_UPDATE_INTERVAL)
  {
    previousMillis = currentMillis;
    BLE.poll();
  }
}

void read_Accel_Gyro() {
  /*
  if (imu.accelAvailable()) {  // check if new accel data is ready  
    imu.readAccelint16(&accel_gyro_data.data_total.ax, &accel_gyro_data.data_total.ay, &accel_gyro_data.data_total.az);  // Read the x/y/z adc values
  } 

  if (imu.gyroAvailable()) {  // check if new gyro data is ready  
    imu.readGyroint16(&accel_gyro_data.data_total.gx, &accel_gyro_data.data_total.gy, &accel_gyro_data.data_total.gz);  // Read the x/y/z adc values 
  }

  Sensor_data.writeValue(accel_gyro_data.bytes, sizeof accel_gyro_data.bytes);
  
  //Serial Test prints 
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
  */
}
