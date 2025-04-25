#include "certs.h"
#include "iot_utils.h"
#include <Wire.h>
#include <ESP32Servo.h>

// WiFi Credentials
const int MAX_NETWORKS = 2;
const char *wifi_ssid[MAX_NETWORKS] = {"V-Fi", "Mubashir's iPhone"};
const char *wifi_password[MAX_NETWORKS] = {"V3ttuv3dic#alil", "12345678"};

// ====== AWS IoT Core Settings ======
const char *mqtt_server = "a1g47hlvkrz0y5-ats.iot.ap-south-1.amazonaws.com";
const int mqtt_port = 8883;

// OTA Firmware for Movement Controll
const char *ota_firmware_url ="https://raw.githubusercontent.com/mubashir-v/robotics-learn/main/movement-controller/.pio/build/esp32dev/firmware.bin";

// MQTT Topics
const char *mqtt_lidar_topic = "esp32/lidar/distance";
const char *ota_topic = "esp32/update";
const char *control_topic = "esp32/lidar/control";

bool isReading = false;

// TF-Luna and Servo
#define SERVO_PIN 13
#define LUNA_ADDR 0x10
Servo myServo;
int servoAngle = 0;
bool servoIncreasing = true;

IoTUtils *iot;

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  payload[length] = '\0';
  String msg = String((char *)payload);
  if (String(topic) == ota_topic) {
    iot->performOTA(ota_firmware_url);
  } else if (String(topic) == control_topic) {
    if (msg == "start") isReading = true;
    else if (msg == "stop") isReading = false;
  }
}

int readLidarDistance() {
  Wire.beginTransmission(LUNA_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) return -1;
  Wire.requestFrom(LUNA_ADDR, 2);
  if (Wire.available() < 2) return -1;
  uint8_t low = Wire.read();
  uint8_t high = Wire.read();
  return (high << 8) | low;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN, 500, 2400);

  iot = new IoTUtils(
    wifi_ssid, wifi_password, MAX_NETWORKS,
    mqtt_server, mqtt_port,
    root_ca, client_cert, client_key,
    "ESP32_Client",
    mqttCallback
  );

  iot->connectWiFi();
  iot->connectMQTT();
  iot->mqttClient.subscribe(ota_topic);
  iot->mqttClient.subscribe(control_topic);
}

void loop() {
  iot->handleMQTTLoop();

  if (isReading) {
    servoAngle += servoIncreasing ? 1 : -1;
    if (servoAngle >= 180 || servoAngle <= 0) servoIncreasing = !servoIncreasing;
    myServo.write(servoAngle);

    int dist = readLidarDistance();
    if (dist > 0) {
      String payload = "{\"distance_cm\":" + String(dist) + ",\"angle_deg\":" + String(servoAngle) + "}";
      iot->mqttClient.publish(mqtt_lidar_topic, payload.c_str());
    }

    delay(5);
  }
}
