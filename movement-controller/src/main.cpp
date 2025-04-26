#include "certs.h"
#include "iot_utils.h"
#include <Wire.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>  // Add ArduinoJson library for parsing JSON

// WiFi Credentials
const int MAX_NETWORKS = 2;
const char *wifi_ssid[MAX_NETWORKS] = {"V-Fi", "Mubashir's iPhone"};
const char *wifi_password[MAX_NETWORKS] = {"V3ttuv3dic#alil", "12345678"};

// ====== AWS IoT Core Settings ======
const char *mqtt_server = "a1g47hlvkrz0y5-ats.iot.ap-south-1.amazonaws.com";
const int mqtt_port = 8883;
IoTUtils *iot;

// ====== Project var and constants START ======
// OTA Firmware for Movement Control
const char *ota_firmware_url = "https://raw.githubusercontent.com/mubashir-v/robotics-learn/main/<PROJECT>/.pio/build/esp32dev/firmware.bin";
// MQTT Topics
const char *ota_topic = "esp32/update";
const char *control_topic = "esp32/motor/control";  // Topic for motor control

// Motor Pins for L298N
const int IN1 = 26;  // GPIO 26
const int IN2 = 27;  // GPIO 27
const int IN3 = 14;  // GPIO 14
const int IN4 = 12;  // GPIO 12
const int ENA = 25;  // GPIO 25 (PWM)
const int ENB = 33;  // GPIO 33 (PWM)

// ====== Project var and constants END ======


void controlMotor(String motor, String direction, int pwm)
{
  if (motor == "A")
  {
    // Control motor A
    ledcWrite(0, pwm);  // Set PWM for motor A
    if (direction == "forward")
    {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    }
    else if (direction == "reverse")
    {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    }
    else if (direction == "stop")
    {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    }
  }
  else if (motor == "B")
  {
    // Control motor B
    ledcWrite(1, pwm);  // Set PWM for motor B
    if (direction == "forward")
    {
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    }
    else if (direction == "reverse")
    {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    }
    else if (direction == "stop")
    {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    }
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  Serial.println("Received OTA Update Message");

  payload[length] = '\0';  // Null-terminate the payload string
  String msg = String((char *)payload);
  
  if (String(topic) == ota_topic)
  {
    iot->performOTA(ota_firmware_url);
  }
  else if (String(topic) == control_topic)
  {
    Serial.println("Received Motor Controll Topic");

    // Parse the incoming message
    StaticJsonDocument<200> doc;
    deserializeJson(doc, msg);
    String motor = doc["motor"];  // Motor A or B
    String direction = doc["direction"];  // forward or reverse
    int pwm = doc["pwm"];  // PWM value (0-255)
    
    // Call function to control the motor based on the parsed values
    controlMotor(motor, direction, pwm);
  }
  else
  {
    // Handle other Topics or messages
  }
}



void setup()
{
  Serial.begin(115200);
  Wire.begin();

  // Set motor control pins to outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Initialize PWM for motors
  ledcSetup(0, 5000, 8); // PWM frequency 5kHz, 8-bit resolution for motor A
  ledcAttachPin(ENA, 0);  // Attach motor A enable pin
  ledcSetup(1, 5000, 8); // PWM frequency 5kHz, 8-bit resolution for motor B
  ledcAttachPin(ENB, 1);  // Attach motor B enable pin

  // Setups Start
  // Edit Here
  // Setup end

  iot = new IoTUtils(
      wifi_ssid, wifi_password, MAX_NETWORKS,
      mqtt_server, mqtt_port,
      root_ca, client_cert, client_key,
      "ESP32_Client",
      mqttCallback);

  iot->connectWiFi();
  iot->connectMQTT();
  iot->mqttClient.subscribe(ota_topic);
  iot->mqttClient.subscribe(control_topic);  // Subscribe to the motor control topic
}

void loop()
{
  iot->handleMQTTLoop();
  // Add any additional logic for your robot's behavior here if needed
}
