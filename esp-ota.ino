#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <Syslog.h>
#include <WiFiUdp.h>
#include <EEPROM.h>

#define POWER_PIN 5
#define BRIGHTNESS_PIN 4
#define POWER_TOPIC "shouse/cmnd/nodemcu-0/POWER"
#define BRIGHTNESS_TOPIC "shouse/cmnd/nodemcu-0/BRIGHTNESS"
#define STATUS_TOPIC "shouse/tele/nodemcu-0/STATUS"

#define WIFI_SSID "MARVEL"
#define WIFI_PASSWORD "sosishurup"
#define WIFI_HOSTNAME "nodemcu-0"

#define SYSLOG_SERVER "192.168.1.5"
#define SYSLOG_PORT 514
#define SYSLOG_DEVICE_HOSTNAME "nodemcu-0"
#define SYSLOG_APP_NAME "mirror"
#define OTA_PASSWORD "53263952360"

#define MQTT_SERVER "192.168.1.5"
#define MQTT_PORT 1883
#define SWITCH_PIN 12

WiFiClient espClient;
PubSubClient client(espClient);

WiFiUDP udpClient;
Syslog syslog(udpClient, SYSLOG_SERVER, SYSLOG_PORT, SYSLOG_DEVICE_HOSTNAME, SYSLOG_APP_NAME, LOG_KERN);

bool switchOn = false;

int readBrightness() {
  int res = EEPROM.read(0);
  byte v2 = EEPROM.read(1);
  res |= v2 << 8;
  return res;
}

void writeBrightness(int val) {
  EEPROM.write(0, (byte)val);
  EEPROM.write(1, val >> 8);
  EEPROM.commit();
}

void setup() {
  EEPROM.begin(512);
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  WiFi.hostname(WIFI_HOSTNAME);
  ArduinoOTA.setHostname(WIFI_HOSTNAME);

  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";    
    } else { // U_SPIFFS
      type = "filesystem";
    }

    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  syslog.logf(LOG_INFO, "Device ready, ip address: %s", WiFi.localIP().toString().c_str());

  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);

  Serial.setDebugOutput(0);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(POWER_PIN, OUTPUT);
  pinMode(BRIGHTNESS_PIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  int brightness = readBrightness();
  analogWrite(BRIGHTNESS_PIN, brightness);
  syslog.logf(LOG_INFO, "light brightness: %d", brightness);

  digitalWrite(POWER_PIN, HIGH);
  delay(1000);
  digitalWrite(POWER_PIN, LOW);
  delay(1000);
  digitalWrite(POWER_PIN, HIGH);
  delay(1000);
  digitalWrite(POWER_PIN, LOW);

  pinMode(SWITCH_PIN, INPUT);
  delay(500);
  if(digitalRead(SWITCH_PIN) == 1) {
    syslog.logf(LOG_INFO, "switch is on");
    switchOn = true;
  } else {
    syslog.logf(LOG_INFO, "switch is off");
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  char *msg = new char[length + 1];
  memcpy(msg, payload, length);
  msg[length] = '\0';

  syslog.logf(LOG_INFO, "mqtt message arrived, topic: %s, message: %s", topic, msg);
  
  if (strcmp(topic, POWER_TOPIC) == 0) {
    if(strcmp(msg, "ON") == 0) {
      digitalWrite(POWER_PIN, HIGH);
    } else if(strcmp(msg, "OFF") == 0 && !switchOn) {
      digitalWrite(POWER_PIN, LOW);
    }
  } else if (strcmp(topic, BRIGHTNESS_TOPIC) == 0) {
    int n = atoi(msg);
    analogWrite(BRIGHTNESS_PIN, n);
    writeBrightness(n);
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("smart-mirror")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(STATUS_TOPIC,"poweron");
      // ... and resubscribe
      client.subscribe(POWER_TOPIC);
      client.subscribe(BRIGHTNESS_TOPIC);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop() {
  ArduinoOTA.handle();

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (!switchOn && digitalRead(SWITCH_PIN) == 1) {
    switchOn = true;
    digitalWrite(POWER_PIN, HIGH);
    delay(100);
    Serial.println("switch on");
    syslog.logf(LOG_INFO, "switch on");
    //client.publish(POWER_TOPIC, "ON", 2);
  } else if(switchOn && digitalRead(SWITCH_PIN) == 0) {
    switchOn = false;
    digitalWrite(POWER_PIN, LOW);
    delay(100);
    Serial.println("switch off");
    syslog.logf(LOG_INFO, "switch off");
    //client.publish(POWER_TOPIC, "OFF", 3);
  }
}

