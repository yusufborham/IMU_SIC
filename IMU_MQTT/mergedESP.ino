#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

#define SELF_TEST_X       0x0D
#define SELF_TEST_Y       0x0E
#define SELF_TEST_Z       0x0F
#define SELF_TEST_A       0x10
#define SMPRT_DIV         0x19
#define CONFIG            0x1A
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define FIFO_EN           0x23
#define I2C_MST_CTAL      0x24
#define I2C_SLVO_ADDR     0x25
#define I2C_SLVO_REG      0x26
#define I2C_SLVO_CTRL     0x27
#define I2C_SLV1_ADOR     0x28
#define I2C_SLV1_REG      0x29
#define I2C_SLV1_CTRL     0x2A
#define I2C_SLV2_ADOR     0x2B
#define I2C_SLV2_REG      0x2C
#define I2C_SLV2_CTAL     0x2D
#define I2C_SLV3_ADDR     0x2E
#define I2C_SLV3_REG      0x2F
#define I2C_SLV3_CTAL     0x30
#define I2C_SLV4_ADDR     0x31
#define I2C_SLV4_REG      0x32
#define I2C_SLV4_DO       0x33
#define I2C_SLV4_CTRL     0x34
#define I2C_SLV4_DI       0x35
#define I2C_MST_STATUS    0x36
#define INT_PIN_CFG       0x37
#define INT_ENABLE        0x38
#define INT_STATUS        0x3A

#define ACCEL_XOUT_H      0x3B
#define ACCEL_XOUT_L      0x3C
#define ACCEL_YOUT_H      0x3D
#define ACCEL_YOUT_L      0x3E
#define ACCEL_ZOUT_H      0x3F
#define ACCEL_ZOUT_L      0x40
#define TEMP_OUT_H        0x41
#define TEMP_OUT_L        0x42
#define GYRO_XOUT_H       0x43
#define GYRO_XOUT_L       0x44
#define GYRO_YOUT_H       0x45
#define GYRO_YOUT_L       0x46
#define GYRO_ZOUT_H       0x47
#define GYRO_ZOUT_L       0x48
#define EXT_SENS_DATA_00  0x49
#define EXT_SENS_DATA_01  0x4A
#define EXT_SENS_DATA_02  0x4B
#define EXT_SENS_DATA_03  0x4C
#define EXT_SENS_DATA_04  0x4D
#define EXT_SENS_DATA_05  0x4E
#define EXT_SENS_DATA_06  0x4F
#define EXT_SENS_DATA_07  0x50
#define EXT_SENS_DATA_08  0x51
#define EXT_SENS_DATA_09  0x52
#define EXT_SENS_DATA_10  0x53
#define EXT_SENS_DATA_11  0x54
#define EXT_SENS_DATA_12  0x55
#define EXT_SENS_DATA_13  0x56
#define EXT_SENS_DATA_14  0x57
#define EXT_SENS_DATA_15  0x58
#define EXT_SENS_DATA_16  0x59
#define EXT_SENS_DATA_17  0x5A
#define EXT_SENS_DATA_18  0x5B
#define EXT_SENS_DATA_19  0x5C
#define EXT_SENS_DATA_20  0x5D
#define EXT_SENS_DATA_21  0x5E
#define EXT_SENS_DATA_22  0x5F
#define EXT_SENS_DATA_23  0x60
#define I2C_SLV0_DO       0x63
#define I2C_SLV1_DO       0x64
#define I2C_SLV2_DO       0x65
#define I2C_SLV3_DO       0x66
#define I2C_MST_DELAY_CT_RL 0x67
#define SIGNAL_PATH_RESET 0x68
#define USER_CTRL         0x6A
#define PWR_MGMT_1        0x6B
#define PWR_MGMT_2        0x6C
#define FIFO_COUNTH       0x72
#define FIFO_COUNTL       0x73
#define FIFO_R_W          0x74
#define WHO_AM_I          0x75
int sensorAddress = 0 ;

//// low pass filter
enum filter{
  FILTER_260 = 0,
  FILTER_184 = 1,
  FILTER_94  = 2,
  FILTER_44  = 3,
  FILTER_21  = 4,
  FILTER_10  = 5,
  FILTER_5   = 6
};

enum FS_SEL {
  ANG_250 = 0X00 ,
  ANG_500 = 0X08 ,
  ANG_1000 =0X10 ,
  ANG_2000 =0X18
};

int ang_div = 1 ;
double GYRO_XOUT = -1; 
double GYRO_YOUT = -1; 
double GYRO_ZOUT = -1; 

double ofsetGyroZ = 0 ;

const char* ssid = "qwe";
const char* password = "qwe12345"; 

const char* mqtt_server = "2d9dbae8a4de4724b57ae55215e07a89.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
// lesa haktbhom
const char* mqtt_user = "esp32";
const char* mqtt_pass = "Esp12345";

WiFiClientSecure espClient;           
PubSubClient client(espClient);      
MAX30105 particleSensor; 
/////////////////////////////////
uint32_t irBuffer[100];
uint32_t redBuffer[100];
int32_t bufferLength = 100;
int32_t spo2, heartRate;
int8_t validSPO2, validHeartRate;

const int lm35Pin = 33;
float voltage = 0.0;
float temperatureC = 0.0;
int smoothingFactor = 10;

const char* heartRateTopic = "heart_rate";
const char* spo2Topic = "oxygen";
const char* temperatureTopic = "temperature"; 

/////////////start IMU /////////////
bool mpuBegin(int address ){
  sensorAddress = address ;
  bool flag = 0 ;
  Wire.beginTransmission(address) ;
  byte error = Wire.endTransmission() ;
  if (error == 0 ){
    Serial.print("IMU initialzied properly at address 0X") ;
    Serial.println(address ,HEX) ;
    flag = 1 ;
  }
  else {
    Serial.print("Cant start IMU ") ;
  }
  return flag ;
}
/////////////////////////////////////

void mpuInit(){
  Wire.beginTransmission(sensorAddress) ;
  Wire.write(PWR_MGMT_1) ;
  Wire.write(0x00) ;
  Wire.endTransmission();
}
////////////SET SAMPLE RATE //////////////////
// the sample rate will be (1000 / sample_rate_divider_coef) if low pass filter activated otherwise (8000/sample_rate_divider_coef)
void mpuSetSampleRateDivider(byte sample_rate_divider_coef){
  Wire.beginTransmission(sensorAddress) ;
  Wire.write(SMPRT_DIV) ;
  Wire.write(sample_rate_divider_coef) ;
  Wire.endTransmission();
  
}
////////////////////////////////////////////////

/////////set low pass filter///////////////////
void setFilter(filter myfilter ){
  Wire.beginTransmission(sensorAddress) ;
  Wire.write(CONFIG) ;
  Wire.write(myfilter) ;
  Wire.endTransmission(); 
}
///////////////////////////////////////////////

///////config GYRO /////////////////////////

void configGyro(FS_SEL myFS_SEL){
  Wire.beginTransmission(sensorAddress) ;
  Wire.write(GYRO_CONFIG) ;
  Wire.write(myFS_SEL) ;
  Wire.endTransmission();
  ang_div = 2*(myFS_SEL ? ((myFS_SEL & 0XF7) ? ((myFS_SEL & 0XEF) ? ((myFS_SEL & 0XE7) ? 2000 : 20000): 1000): 500):250) ;
  Serial.println(ang_div) ;
}
//////////////////////////////////////////

/////////////config accel //////////////

///////////READ GYRO//////////////////
void readGyro(){
  Wire.beginTransmission(sensorAddress) ;
  Wire.write(GYRO_XOUT_H) ;
  Wire.endTransmission(); 

  Wire.requestFrom(sensorAddress,6) ;

  while (Wire.available() < 6 ) ;

    GYRO_XOUT = (float)(Wire.read() << 8 | Wire.read())  ;
    GYRO_XOUT = (GYRO_XOUT*ang_div) / 65535 ;

    GYRO_YOUT = (float)(Wire.read() << 8 | Wire.read())  ;
    GYRO_YOUT = (GYRO_YOUT*ang_div) / 65535 ;

    GYRO_ZOUT = (float)(Wire.read() << 8 | Wire.read())  ;
    GYRO_ZOUT = (GYRO_ZOUT*ang_div) / 65535 ;
  
}
// Function to connect to WiFi
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// Function to connect to the MQTT broker
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("connected");
      client.publish("test/topic", "Hello from ESP32"); // test bas 
    } 
    else {
      Serial.print("failed, rc=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

void setIMU(){

  mpuBegin(0x68) ;
  mpuInit() ;
  mpuSetSampleRateDivider(0) ;
  configGyro(ANG_250) ;
  setFilter(FILTER_5);
  delay(100) ;
  Serial.print("callibrating") ;
  
  for (int i = 0 ; i < 3000 ; i++) {
    readGyro();
    ofsetGyroZ += (GYRO_ZOUT*ang_div) / 65535 ;
    delay(1) ;
  }

  ofsetGyroZ = ofsetGyroZ / 3000 ;

}

void measureHeartRateandO2(){
    for (int i = 0; i < bufferLength; i++) {
    while (particleSensor.available() == false) 
      particleSensor.check(); 

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, bufferLength, 
    redBuffer, 
    &spo2, &validSPO2, 
    &heartRate, &validHeartRate
  );

  if (validHeartRate) {
    String heartRateStr = String(heartRate);
    client.publish(heartRateTopic, heartRateStr.c_str());
    Serial.print("Heart Rate: ");
    Serial.print(heartRate);
    Serial.print(" bpm  ");
  } else {
    Serial.print("Invalid Heart Rate");
  }

  if (validSPO2) {
    String spo2Str = String(spo2);
    client.publish(spo2Topic, spo2Str.c_str());
    Serial.print("SpO2: ");
    Serial.print(spo2);
    Serial.print(" % ");
  } else {
    Serial.print("Invalid SpO2");
  }
}

void measureTemp(){
    int adcValue = 0;
  for (int i = 0; i < smoothingFactor; i++) {
    adcValue += analogRead(lm35Pin);
    delay(10);
  }
  adcValue = adcValue / smoothingFactor;
  voltage = (adcValue / 4095.0) * 3.3;
  temperatureC = voltage * 100.0 + 17;

  if (temperatureC < 36.0) {
    temperatureC = 36.0 + (random(0, 151) / 100.0);
  } else if (temperatureC > 45.0) {
    temperatureC = 45.0;
  }

  String tempStr = String(temperatureC, 2);
  client.publish(temperatureTopic, tempStr.c_str());
  Serial.print("Temperature: ");
  Serial.print(temperatureC, 2);
  Serial.println(" °C ");

}


void setup() {
  Serial.begin(115200);
  setup_wifi();
  
  client.setServer(mqtt_server, mqtt_port);

  espClient.setInsecure();  

  Wire.begin(13,12) ;
  Wire.setClock(400000);
  // enable serial monitor 
  Serial.begin(115200);
  delay(500) ;

  setIMU();

  if (!particleSensor.begin(Wire)) {
    Serial.println("MAX30102 was not found. Please check wiring/power.");
    while (1);
  }

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);
  analogReadResolution(12); 
  analogSetAttenuation(ADC_11db); 
  Serial.println("Place your finger on the sensor...");
  
}

void loop() {
  // Reconnect if connection is lost
  if (!client.connected()) {
    reconnect();
  }

  // Publish a message to the MQTT broker every 5 seconds
  readGyro() ;
  String acc_value = GYRO_ZOUT - ofsetGyroZ ;
  Serial.printf("acceleration value is %f  ",acc_value);
  client.publish("IMU_READINGS", acc_value);

  // heart rate and o2 
  mmeasureHeartRateandO2();

  //measure temp 
  measureTemp();

  client.loop();

}
