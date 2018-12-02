
#define RCSwitchMulti_DEBUG_PACKETS true

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "RCSwitchMulti.h"
#include "BresserReceiver.h"

#include "config.h"

const char* ssid = SSID;
const char* password = PASS;
const char* mqtt_server = MQTT_SERVER_HOST;
const int mqtt_port = MQTT_SERVER_PORT;

const int led = LED_BUILTIN; // 13
const int LED_ON = LOW;
const int LED_OFF = HIGH;

WiFiClient wifiClient;
PubSubClient client(wifiClient);

BresserReceiver bresser;
RCSwitchMulti rfreceiver;


long lastMsg = 0;
char msg[75];

void setupWifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  
  static int ledState = LED_ON;

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("done");

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  randomSeed(micros());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "RF433MQTTGateway";
    if (client.connect(clientId.c_str(), "/devices/rf433gateway/LWT", MQTTQOS1, true, "Offline")) {
      Serial.println("connected");
      publishMQTT("/devices/rf433gateway/LWT", "Online", true);
    } else {
      Serial.print("failed. Client state: ");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);   // Wait 5 seconds before retrying
    }
  }
}

void setup() {
 
  Serial.begin(115200);
  Serial.println("\n");
  Serial.println("==========================================\n");
  Serial.println("\nComplete RF433-MQTT Gateway initialized!");

  setupWifi();
  client.setServer(mqtt_server, mqtt_port);

  attachInterrupt(digitalPinToInterrupt(2), handleInterrupt, CHANGE);

  reconnect();
  
}

void handleInterrupt(void) {
  static word last;
  // determine the pulse length in microseconds, for either polarity
  word pulse = micros() - last;
  last += pulse;

  bresser.handlePulse(pulse);
  rfreceiver.handlePulse(pulse);
}

void publishMQTT(String topic, String message, bool retained) {
  Serial.println("Sending MQTT message: " + topic + ": " + message);

  char t[topic.length() + 1];
  topic.toCharArray(t, topic.length() + 1);

  char m[message.length() + 1];
  message.toCharArray(m, message.length() + 1);

  client.publish(t, m, retained);
}

void checkIntertechnoAndNatsenPacket() {
  static bool stateWaitingForSignal = true;
  static long silenceStartTime = 0;

  unsigned long value = rfreceiver.getReceivedValue();
  if (stateWaitingForSignal) {
    if (value != 0) {
      rfreceiver.resetAvailable();
      publishMQTT("/switches/" + String(value, HEX), "pressed", false);
      stateWaitingForSignal = false;
    }
  } else {
    // waiting for pause of adequate length

    if (value != 0) {
      rfreceiver.resetAvailable();
      silenceStartTime = 0;
    } else {
      // no signal
      if (silenceStartTime == 0) {
        silenceStartTime = millis();
      } else {
        long silenceDuration = millis() - silenceStartTime;
        if (silenceDuration > 200) {
          stateWaitingForSignal = true;
          silenceStartTime = 0;
//          Serial.println("Waiting for Intertechno/Natsen signal again");
        }
      }
    }
  }
}

void checkBresserPacket() {
  static byte previousPacket[BresserReceiver_NUM_BYTES];

  byte packet[BresserReceiver_NUM_BYTES];
  bool packetReceived = bresser.takePacket(packet);
  if (packetReceived) {
    if (!bresser.samePacket(previousPacket, packet)) {
      memcpy(previousPacket, packet, BresserReceiver_NUM_BYTES);

      int deviceId = bresser.deviceId(packet);

      publishMQTT(
        "/sensors/bresser/" + String(deviceId) + "/channel",
        String(bresser.channel(packet)),
        true);
      publishMQTT(
        "/sensors/bresser/" + String(deviceId) + "/temperature",
        String(bresser.temperature(packet)),
        true);
      publishMQTT(
        "/sensors/bresser/" + String(deviceId) + "/humidity",
        String(bresser.humidity(packet)),
        true);
    }
  }
}


// NOTE: enabling the LED breaks serial transmission, because TX and blue LED use the same GPIO1
void toggleStatusLedEverySecond() {
  static long lastLedToggleTime = 0;
  static int ledState = LED_OFF;

  long now = millis();
  if (now - lastLedToggleTime > 1000) {
    if (ledState == LED_OFF) {
      ledState = LED_ON;
    } else {
      ledState = LED_OFF;
    }
    digitalWrite(led, ledState); 
    lastLedToggleTime = now;
  }
}

void blinkStatusLedEverySecond() {
  static long lastLedToggleTime = 0;

  long now = millis();
  if (now - lastLedToggleTime > 1000) {
    Serial.println("-----------------------------------");  // this blinks the blue lead, because the TX and blue LED both use GPIO1
    lastLedToggleTime = now;
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  checkIntertechnoAndNatsenPacket();
  checkBresserPacket();
  //toggleStatusLedEverySecond();
  blinkStatusLedEverySecond();
}
