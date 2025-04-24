#include "certs.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <Update.h>
#include <Wire.h>
#include <ESP32Servo.h>

// ====== Wi-Fi Networks ======
const int MAX_NETWORKS = 2; // Number of WiFi networks to try
const char *ssid[MAX_NETWORKS] = {
    "V-Fi",             // Primary network
    "Mubashir's iPhone" // Secondary network
};
const char *password[MAX_NETWORKS] = {
    "V3ttuv3dic#alil", // Primary password
    "12345678"         // Secondary password
};

// ====== AWS IoT Core Settings ======
const char *mqtt_server = "a1g47hlvkrz0y5-ats.iot.ap-south-1.amazonaws.com";
const int mqtt_port = 8883;
const char *mqtt_lidar_topic = "esp32/lidar/distance";
const char *ota_topic = "esp32/update";
const char *control_topic = "esp32/lidar/control";
bool isReading = false;

// ====== TF-Luna I2C Settings ======
#define LUNA_ADDR 0x10

WiFiClientSecure secureClient;
PubSubClient mqttClient(secureClient);

// ====== Servo ========

#define SERVO_PIN 13
Servo myServo;
int servoAngle = 0;
bool servoIncreasing = true;

// ====== Connect to Wi-Fi with fallback ======
void connectWiFi()
{
  Serial.println("Attempting to connect to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(1000);

  for (int i = 0; i < MAX_NETWORKS; i++)
  {
    Serial.print("Trying network: ");
    Serial.println(ssid[i]);

    WiFi.begin(ssid[i], password[i]);

    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 20)
    { // Try for 10 seconds (20 * 500ms)
      delay(500);
      Serial.print(".");
      retries++;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("\nWiFi connected!");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      return;
    }

    Serial.println("\nConnection failed, trying next network...");
    WiFi.disconnect();
    delay(1000);
  }

  Serial.println("Failed to connect to any WiFi network");
  // You might want to add ESP.restart() here or other recovery logic
}

// ====== OTA Logic ======
void performOTA(String bin_url)
{
  WiFiClientSecure client;
  client.setInsecure(); // Optional for GitHub if not using custom cert
  HTTPClient https;

  Serial.println("Starting OTA: " + bin_url);
  if (https.begin(client, bin_url))
  {
    int httpCode = https.GET();
    if (httpCode == HTTP_CODE_OK)
    {
      int len = https.getSize();
      if (Update.begin(len))
      {
        WiFiClient *stream = https.getStreamPtr();
        size_t written = Update.writeStream(*stream);
        if (written == len)
        {
          Serial.println("Update successful, rebooting...");
          if (Update.end())
            ESP.restart();
        }
        else
        {
          Serial.println("Write failed");
        }
      }
      else
      {
        Serial.println("Not enough space");
      }
    }
    else
    {
      Serial.println("HTTP Error: " + String(httpCode));
    }
    https.end();
  }
  else
  {
    Serial.println("HTTPS begin failed");
  }
}

// ====== MQTT OTA Message Handler ======
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  payload[length] = '\0';
  if (String(topic) == ota_topic)
  {
    String binURL = String((char *)payload);
    Serial.println("Received OTA URL: " + binURL);
    performOTA(binURL);
  }
  else if (String(topic) == control_topic)
  {
    String message = String((char *)payload);
    if (message == "start")
    {
      isReading = true;
      Serial.println("Distance reading started.");
    }
    else if (message == "stop")
    {
      isReading = false;
      Serial.println("Distance reading stopped.");
    }
  }
}

// ====== Connect to AWS MQTT ======
void connectMQTT()
{
  secureClient.setCACert(root_ca);
  secureClient.setCertificate(client_cert);
  secureClient.setPrivateKey(client_key);

  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);

  while (!mqttClient.connected())
  {
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect("ESP32_Client"))
    {
      Serial.println(" connected.");
      mqttClient.subscribe(ota_topic);
      mqttClient.subscribe(control_topic);
    }
    else
    {
      Serial.print(" failed, rc=");
      Serial.println(mqttClient.state());
      delay(2000);
    }
  }
}

// ====== TF-Luna I2C Read ======
int readLidarDistance()
{
  Wire.beginTransmission(LUNA_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0)
  {
    Serial.println("TF-Luna I2C error");
    return -1;
  }

  Wire.requestFrom(LUNA_ADDR, 2);
  if (Wire.available() < 2)
  {
    Serial.println("TF-Luna: No data");
    return -1;
  }

  uint8_t low = Wire.read();
  uint8_t high = Wire.read();
  return (high << 8) | low;
}

// ====== Setup ======
void setup()
{
  Serial.begin(115200);
  Wire.begin();
  delay(1000);
  connectWiFi();
  connectMQTT();
  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN, 500, 2400);
}

// ====== Loop ======
void loop()
{
  if (!mqttClient.connected())
  {
    connectMQTT();
  }
  mqttClient.loop();

  if (isReading)
  {
    if (servoIncreasing)
    {
      servoAngle += 1;
      if (servoAngle >= 180)
      {
        servoAngle = 180;
        servoIncreasing = false;
      }
    }
    else
    {
      servoAngle -= 1;
      if (servoAngle <= 0)
      {
        servoAngle = 0;
        servoIncreasing = true;
      }
    }

    myServo.write(servoAngle);

    int distance = readLidarDistance();
    if (distance > 0)
    {
      String payload = "{\"distance_cm\":" + String(distance) + ",\"angle_deg\":" + String(servoAngle) + "}";
      Serial.println("Publishing: " + payload);
      mqttClient.publish(mqtt_lidar_topic, payload.c_str());
    }
    else
    {
      Serial.println("Failed to read TF-Luna");
    }

    delay(5); // Controls scan speed
  }
}
