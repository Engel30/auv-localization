#include <WiFi.h>
#include <PubSubClient.h>

// Configura le credenziali WiFi
const char* ssid = "RP-Net-WD03";
const char* password = "20190901";

// Configura il broker MQTT
const char* mqtt_server = "192.168.11.1";
const int mqtt_port = 1883;
const char* mqtt_topic = "/mqtt1";


WiFiClient espClient;
PubSubClient client(espClient);


void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
}

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

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Simula un messaggio da inviare
  String message = "Hello from ESP32S3";
  client.publish(mqtt_topic, message.c_str());
  delay(1000);
}
