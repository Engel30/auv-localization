#include <WiFi.h>
#include <M5StickCPlus.h>
#include <PubSubClient.h>

// WiFi credentials
const char* ssid = "Redmi note 13 pro";
const char* password = "hotspot0202";

// MQTT broker configuration
const char* mqtt_server = "192.168.61.91";
const int mqtt_port = 1883;

// WiFi and MQTT client
WiFiClient espClient;
PubSubClient client(espClient);

// Pin GPIO per il controllo del LED superiore
const int ledPin = 10; // Sostituisci con il pin corretto per il LED superiore

void setup() {
  M5.begin();
  Serial.begin(115200);

  // Configure WiFi
  setup_wifi();
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH); // Assicurati che il LED sia spento all'avvio

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
      client.subscribe("/accensione"); // Subscribe to topic for LED control
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
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println(message);

  // Check if the message is to control the LED
  if (String(topic) == "/accensione") {
    if (message.equals("accendi")) {
      digitalWrite(ledPin, LOW);
    } else if (message.equals("spegni")) {
      digitalWrite(ledPin, HIGH);
    }
  }
}
