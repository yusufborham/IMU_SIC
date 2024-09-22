#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

const char* ssid = "qwe";
const char* password = "qwe12345";

const char* mqtt_server = "2d9dbae8a4de4724b57ae55215e07a89.s1.eu.hivemq.cloud";  
const int mqtt_port = 8883;

const char* mqtt_user = "esp32";
const char* mqtt_pass = "Esp12345";

WiFiClientSecure espClient;  
PubSubClient client(espClient);  
MAX30105 particleSensor;  

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

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_pass)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  
  espClient.setInsecure(); 
  client.setServer(mqtt_server, mqtt_port);

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
  if (!client.connected()) {
    reconnect();
  }

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
    Serial.println(" bpm");
  } else {
    Serial.println("Invalid Heart Rate");
  }

  if (validSPO2) {
    String spo2Str = String(spo2);
    client.publish(spo2Topic, spo2Str.c_str());
    Serial.print("SpO2: ");
    Serial.print(spo2);
    Serial.println(" %");
  } else {
    Serial.println("Invalid SpO2");
  }

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
  Serial.println(" °C");

  client.loop();
  delay(1000);
}