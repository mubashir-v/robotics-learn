#include "iot_utils.h"

IoTUtils::IoTUtils(
  const char **wifi_ssids, const char **wifi_passwords, int wifi_count,
  const char *mqtt_server, int mqtt_port,
  const char *root_ca, const char *client_cert, const char *client_key,
  const char *client_id,
  std::function<void(char *, byte *, unsigned int)> mqttCallback
) : mqttClient(_secureClient) {
  _wifi_ssids = wifi_ssids;
  _wifi_passwords = wifi_passwords;
  _wifi_count = wifi_count;
  _mqtt_server = mqtt_server;
  _mqtt_port = mqtt_port;
  _root_ca = root_ca;
  _client_cert = client_cert;
  _client_key = client_key;
  _client_id = client_id;

  _secureClient.setCACert(_root_ca);
  _secureClient.setCertificate(_client_cert);
  _secureClient.setPrivateKey(_client_key);

  mqttClient.setServer(_mqtt_server, _mqtt_port);
  mqttClient.setCallback(mqttCallback);
}

void IoTUtils::connectWiFi() {
  Serial.flush();
  Serial.println("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(1000);

  for (int i = 0; i < _wifi_count; i++) {
    Serial.print("Trying network: ");
    Serial.flush();
    Serial.println(_wifi_ssids[i]);

    WiFi.begin(_wifi_ssids[i], _wifi_passwords[i]);

    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 20) {
      delay(500);
      Serial.print(".");
      Serial.flush();
      retries++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nConnected to WiFi");
      Serial.println(WiFi.localIP());
      return;
    }

    Serial.println("\nFailed. Trying next...");
    WiFi.disconnect();
    delay(1000);
  }

  Serial.println("All WiFi attempts failed.");
}

void IoTUtils::connectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");
    Serial.flush();
    if (mqttClient.connect(_client_id)) {
      Serial.println(" connected.");
      Serial.flush();
    } else {
      Serial.print(" failed, rc=");
      Serial.flush();
      Serial.println(mqttClient.state());
      delay(2000);
    }
  }
}

void IoTUtils::handleMQTTLoop() {
  if (!mqttClient.connected()) {
    connectMQTT();
  }
  mqttClient.loop();
}

void IoTUtils::performOTA(String bin_url) {
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient https;

  Serial.println("Starting OTA from: " + bin_url);
  if (https.begin(client, bin_url)) {
    int httpCode = https.GET();
    if (httpCode == HTTP_CODE_OK) {
      int len = https.getSize();
      if (Update.begin(len)) {
        WiFiClient *stream = https.getStreamPtr();
        size_t written = Update.writeStream(*stream);
        if (written == len && Update.end()) {
          Serial.println("OTA Success. Rebooting...");
          ESP.restart();
        } else {
          Serial.println("OTA write failed");
        }
      } else {
        Serial.println("Not enough space for OTA");
      }
    } else {
      Serial.println("HTTP error: " + String(httpCode));
    }
    https.end();
  } else {
    Serial.println("Failed to begin OTA");
  }
}
