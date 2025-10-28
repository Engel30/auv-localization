#include <WiFi.h>
#include <M5StickCPlus.h>
#include <PubSubClient.h>
#include "Hat_8Servos.h"

Hat_8Servos drive;

// WiFi credentials
const char* ssid = "Redmi note 13 pro";
const char* password = "hotspot0202";

// MQTT broker configuration
const char* mqtt_server = "192.168.61.91";
const int mqtt_port = 1883;

// WiFi and MQTT client
WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  M5.begin();
  Serial.begin(115200);

  // Configure WiFi
  setup_wifi();

  if (drive.begin(&Wire, 0, 26, 0x36)) {
    M5.Lcd.print("8Servos HAT inizializzato correttamente");
  }
  drive.enableServoPower(1);  // Abilita l'alimentazione del servo sul canale 1
  vTaskDelay(1000); 
  
  // Configure MQTT client
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Attempt to connect to MQTT broker
  reconnect();
}

// WiFi Connection
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

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("M5StickCPlus")) {
      Serial.println("connected");
      client.subscribe("/servo_angle"); // Subscribe to topic for servo control
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}

// Callback function for MQTT messages
void callback(char* topic, byte* payload, unsigned int length) {
  if (String(topic) == "/servo_angle") {
    // Converti il payload in una stringa
    String angleStr = "";
    for (unsigned int i = 0; i < length; i++) {
      angleStr += (char)payload[i];
    }
    
    // Converti la stringa in un intero
    int angle = angleStr.toInt();
    Serial.print("Angle received: ");
    Serial.println(angle);

    if (angle >= 0 && angle <= 180) {
      drive.setServoAngle(1, angle);  // Imposta l'angolo del servo su 0 gradi
      vTaskDelay(2000); 
    }
  }
}


