#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <Update.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <LittleFS.h>

// ====== Wi-Fi Networks ======
const int MAX_NETWORKS = 2; // Number of WiFi networks to try
const char* ssid[MAX_NETWORKS] = {
  "V-Fi",          // Primary network
  "Mubashir's iPhone"    // Secondary network
};
const char* password[MAX_NETWORKS] = {
  "V3ttuv3dic#alil",  // Primary password
  "12345678"   // Secondary password
};


// ====== AWS IoT Core Settings ======
const char* mqtt_server = "a1g47hlvkrz0y5-ats.iot.ap-south-1.amazonaws.com";
const int mqtt_port = 8883;
const char* mqtt_lidar_topic = "esp32/lidar/distance";
const char* ota_topic = "esp32/update";
const char* control_topic = "esp32/lidar/control";
bool isReading = false;


// ====== Certificates ======

const char* client_cert = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDWjCCAkKgAwIBAgIVAJ1uOTbTr3vwfZVS9RriGnm3MocMMA0GCSqGSIb3DQEB
CwUAME0xSzBJBgNVBAsMQkFtYXpvbiBXZWIgU2VydmljZXMgTz1BbWF6b24uY29t
IEluYy4gTD1TZWF0dGxlIFNUPVdhc2hpbmd0b24gQz1VUzAeFw0yNTA0MjQwNzMw
MzdaFw00OTEyMzEyMzU5NTlaMB4xHDAaBgNVBAMME0FXUyBJb1QgQ2VydGlmaWNh
dGUwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDOR1H/7Zxp7Zqngv8E
znO0QnBoSS1cw8LXFV+PFCKSViCW3S0PhBqVll/acXAbZeqf76fzJ4rCU140x/pt
OxtCqSPhFXtwCNc6cdT9Pezi7glYtgC5v3V4XVNEGZdiRhtJFPsCMKXIfmV0Xl0Y
XtsjjpIIF3sgbrJs+VD26oijA4eYhY/Mhae+cvc5J1ATJ5vwHpNPY6CGTMKqiCH5
FhOjaYumm9E161QHydycChcdr1FM+Tnir0oPo5O7A1XRl/qUs/ptLKs1aVRY7D2o
7ExryjfvovF+G040sHsDY7vLQ1VCU8tdNf056BdQl6vSRixIW5a/1q2DQJ4s4UdO
8yvPAgMBAAGjYDBeMB8GA1UdIwQYMBaAFKcYly/86rc6OFxDbcOILXbt8dioMB0G
A1UdDgQWBBRsDTufWepbJNDEehrPYBg2izFYtjAMBgNVHRMBAf8EAjAAMA4GA1Ud
DwEB/wQEAwIHgDANBgkqhkiG9w0BAQsFAAOCAQEAD7vxkCR2EYPBPGg2fqbz27IA
qFwVAt1D2u8Irnob7IiYiNGtvDG3py6cA70Xn4B6RMoqE0sxSNrElEJS/QLCCnBx
vWdQZs6bK6EEEogZBQmuO17D546gakJ2DTe5Aom6cnl1eCUn7zLacXwzGRR68Vuy
rDYseRkiX8GkM2Svx1qKqDOn/sFsDU0JMvs35enxs4FOV6L8Kxbdz+pqRJ4oMrTA
4DxaByHY0MLKgeTD8R153BVCqYa5UsdH7d9+Iu39nMBsJ3pZr3zxufc7X+0z9c0Y
lsF0i6JI7llBxQfY39m3SaQX8NMmt4lS9RN7jg1/WKnhd6dW4bJQsRY1YS4GgA==
-----END CERTIFICATE-----
)EOF";

const char* client_key = R"EOF(
-----BEGIN RSA PRIVATE KEY-----
MIIEpAIBAAKCAQEAzkdR/+2cae2ap4L/BM5ztEJwaEktXMPC1xVfjxQiklYglt0t
D4QalZZf2nFwG2Xqn++n8yeKwlNeNMf6bTsbQqkj4RV7cAjXOnHU/T3s4u4JWLYA
ub91eF1TRBmXYkYbSRT7AjClyH5ldF5dGF7bI46SCBd7IG6ybPlQ9uqIowOHmIWP
zIWnvnL3OSdQEyeb8B6TT2OghkzCqogh+RYTo2mLppvRNetUB8ncnAoXHa9RTPk5
4q9KD6OTuwNV0Zf6lLP6bSyrNWlUWOw9qOxMa8o376LxfhtONLB7A2O7y0NVQlPL
XTX9OegXUJer0kYsSFuWv9atg0CeLOFHTvMrzwIDAQABAoIBAHLOOPIi0coW5ttS
ShLnwHfT/nHcQHX/siI5EA023KLrconkoV6gXJdisQiH2Cg3ieTn4mavXEgZVDY2
EsfWfK6WKpTset343aj2j98+JuiKKQOJkE/RIlUSQ/SD9cBJWohLOASZBLvYtSiW
GGQm7voUMGx2fR2o3n9OJ2S1VcS7g0wbxszq7Y+OwTSma/bqjmmexP7OSrcp9QEx
vXRf9KJrkgMcZxjSdkMKZ3wP5df2RMJATOv7ygVNtmQRmdPu05JE2YBEFKyhKExc
tEUHjgLQ3u9hORCluLRRpHaNBbSZQEnO/8gSJwoAWJhnPG4Gfaf4Y9fzTS86oK5U
ldfASTkCgYEA68U6ov3+9z2ZVAS6gxyx13ZErTlc6PVPqxQrcSx0xq5MXSiaEE8G
1fd7pqsNIikQI7ZYDParYwIECxAxYNJMVD7WxdzMb/Nbh+RzvWLqL/yzbnLR6AMI
8zTtVoao1ZWu2WyTbiADyaMDsDqm1F3f9Nvm/dzLKi6Q75Jw6cxQbLUCgYEA3/pL
Qza18t+5XjsoipYbOCzAaQa5UDVp2YdWVZYF8igLvNKeea0yTldiXwZAYmHO0lgb
uz2/bhMXq/jNhaUYmcHwaNnc0wydvd+2pT+zKNxx91oLtzQmtOflnKgfQUxR/b9B
TdcHNl/rqaMOWT0zBMvCaaZj2NOW1IR9fRkljPMCgYEA3PlNq9ZDUZb6TjTgWmJa
jIbcdz58RhslQEL3NGtmR5C1IdscCo0G6yO5UMyg0pOJ/O66N+w5VeJdx8C1hvmT
cKI0yd3X/lUoRIVptOcwdylxQuN6uOmfSdMhXyd2tjhEWKPayixXYHTY0CMomYIi
lWxFY0m9YRuj2+Z0zawg9MECgYEA3+Yr2pMUrCgt/B8TVgbcSvmhltwb4CylFcmO
6pvAceTKZwX9WhFjWqFMO0Xakv7Ha8qcXj9J4FXBsi5T9aemwf33wDkZ4PiZr7e1
s9fL16sXWndvgsLldq7jAAmoxPPwesL4G0I7eYXF5ZyUuWzOVZsDS+4DnjtWajfP
4oCC4zUCgYBkbipPeyGsov3pzFHMdqzAEHZJg84/3VpWGdGSsBP6E9sh76/m3tvq
GXcL7KM0ZGKNjLrbOukrjxCecUwMcJIws/cHXuZD+d7Hs4qTWD1h/CkZKLa7N5Zv
jWCCRPmMqVRqZll6+fBRayQ8OxQWoZAgpkXbqZHbwbAGghZT/NInrA==
-----END RSA PRIVATE KEY-----
)EOF";

const char* root_ca = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";


// ====== TF-Luna I2C Settings ======
#define LUNA_ADDR 0x10

WiFiClientSecure secureClient;
PubSubClient mqttClient(secureClient);

// Load certificate and File From Board
void loadCertFromFS(const char* path, char* buf, size_t max_len) {
  File file = LittleFS.open(path, "r");
  if (!file) {
    Serial.println("Failed to open " + String(path));
    return;
  }
  size_t len = file.readBytes(buf, max_len - 1);
  buf[len] = '\0';
  file.close();
}
// ====== Servo ========

#define SERVO_PIN 13
Servo myServo;
int servoAngle = 0;
bool servoIncreasing = true;

// ====== Connect to Wi-Fi with fallback ======
void connectWiFi() {
  Serial.println("Attempting to connect to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(1000);
  
  for(int i = 0; i < MAX_NETWORKS; i++) {
    Serial.print("Trying network: ");
    Serial.println(ssid[i]);
    
    WiFi.begin(ssid[i], password[i]);
    
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 20) { // Try for 10 seconds (20 * 500ms)
      delay(500);
      Serial.print(".");
      retries++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
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
void performOTA(String bin_url) {
  WiFiClientSecure client;
  client.setInsecure();  // Optional for GitHub if not using custom cert
  HTTPClient https;

  Serial.println("Starting OTA: " + bin_url);
  if (https.begin(client, bin_url)) {
    int httpCode = https.GET();
    if (httpCode == HTTP_CODE_OK) {
      int len = https.getSize();
      if (Update.begin(len)) {
        WiFiClient* stream = https.getStreamPtr();
        size_t written = Update.writeStream(*stream);
        if (written == len) {
          Serial.println("Update successful, rebooting...");
          if (Update.end()) ESP.restart();
        } else {
          Serial.println("Write failed");
        }
      } else {
        Serial.println("Not enough space");
      }
    } else {
      Serial.println("HTTP Error: " + String(httpCode));
    }
    https.end();
  } else {
    Serial.println("HTTPS begin failed");
  }
}

// ====== MQTT OTA Message Handler ======
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  if (String(topic) == ota_topic) {
    String binURL = String((char*)payload);
    Serial.println("Received OTA URL: " + binURL);
    performOTA(binURL);
  } else if (String(topic) == control_topic) {
    String message = String((char*)payload);
    if (message == "start") {
      isReading = true;
      Serial.println("Distance reading started.");
    } else if (message == "stop") {
      isReading = false;
      Serial.println("Distance reading stopped.");
    }
  }
}


// ====== Connect to AWS MQTT ======
void connectMQTT() {
  secureClient.setCACert(root_ca);
  secureClient.setCertificate(client_cert);
  secureClient.setPrivateKey(client_key);

  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);

  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect("ESP32_Client")) {
      Serial.println(" connected.");
      mqttClient.subscribe(ota_topic);
      mqttClient.subscribe(control_topic);
    } else {
      Serial.print(" failed, rc=");
      Serial.println(mqttClient.state());
      delay(2000);
    }
  }
}




// ====== TF-Luna I2C Read ======
int readLidarDistance() {
  Wire.beginTransmission(LUNA_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) {
    Serial.println("TF-Luna I2C error");
    return -1;
  }

  Wire.requestFrom(LUNA_ADDR, 2);
  if (Wire.available() < 2) {
    Serial.println("TF-Luna: No data");
    return -1;
  }

  uint8_t low = Wire.read();
  uint8_t high = Wire.read();
  return (high << 8) | low;
}

// ====== Setup ======
void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(1000);
  connectWiFi();
  connectMQTT();
  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN, 500, 2400);
}

// ====== Loop ======
void loop() {
  if (!mqttClient.connected()) {
    connectMQTT();
  }
  mqttClient.loop();

  if (isReading) {
    if (servoIncreasing) {
      servoAngle += 1;
      if (servoAngle >= 180) {
        servoAngle = 180;
        servoIncreasing = false;
      }
    } else {
      servoAngle -= 1;
      if (servoAngle <= 0) {
        servoAngle = 0;
        servoIncreasing = true;
      }
    }

    myServo.write(servoAngle);

    int distance = readLidarDistance();
    if (distance > 0) {
      String payload = "{\"distance_cm\":" + String(distance) + ",\"angle_deg\":" + String(servoAngle) + "}";
      Serial.println("Publishing: " + payload);
      mqttClient.publish(mqtt_lidar_topic, payload.c_str());
    } else {
      Serial.println("Failed to read TF-Luna");
    }

    delay(5);  // Controls scan speed
  }
}

