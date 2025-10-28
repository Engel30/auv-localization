#include <M5StickCPlus.h>
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "Redmi note 13 pro";
const char* password = "hotspot0202";
const char* mqtt_server = "192.168.61.91";

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
    M5.begin();
    M5.Lcd.print("Connecting");

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        M5.Lcd.print(".");
    }

    client.setServer(mqtt_server, 1883);
    while (!client.connected()) {
        M5.Lcd.print("Connecting to MQTT...");
        if (client.connect("M5StickCPlus")) {
            M5.Lcd.print("Connected to MQTT");
        } else {
            M5.Lcd.print("Failed to connect, retrying in 5 seconds");
            delay(5000);
        }
    }

    M5.Lcd.print("Connected");
}

void loop() {
    M5.update();
    if (!client.connected()) {
        while (!client.connected()) {
            M5.Lcd.print("Reconnecting to MQTT...");
            if (client.connect("M5StickCPlus")) {
                M5.Lcd.print("Reconnected to MQTT");
            } else {
                M5.Lcd.print("Failed to reconnect, retrying in 5 seconds");
                delay(5000);
            }
        }
    }
    client.loop();
    if (M5.BtnA.wasPressed()) {
        client.publish("/m5stickc/button", "Button A pressed");
        M5.Lcd.print("\nButton A pressed");
    } else if (M5.BtnB.wasPressed()) {
        client.publish("/m5stickc/button", "Button B pressed");
        M5.Lcd.print("\nButton B pressed");
    }
}
