#!/bin/bash

# Sorgente delle variabili d'ambiente ROS 2
source /opt/ros/humble/setup.bash

# Configurazione MQTT
MQTT_HOST="192.168.26.91"
MQTT_PORT="1883"
MQTT_TOPIC_PUB="/accensione"
MQTT_TOPIC_SUB="/accensione"

# Configurazione ROS 2
ROS2_TOPIC_SUB="/accensione"
ROS2_MSG_TYPE="std_msgs/String"

# Funzione per pubblicare su MQTT
publish_to_mqtt() {
    local msg=$1
    echo "Pubblicazione su MQTT: $msg"
    mosquitto_pub -h $MQTT_HOST -p $MQTT_PORT -t $MQTT_TOPIC_PUB -m "$msg"
}

# Ascolta i messaggi ROS 2 e pubblicali su MQTT
echo "Avvio ascolto su ROS 2 e pubblicazione su MQTT..."
ros2 topic echo $ROS2_TOPIC_SUB --qos-profile sensor_data | while read -r line; do
    if [[ $line == data:* ]]; then
        msg=${line#data: }
        echo "Ricevuto messaggio ROS 2: $msg"
        
        # Pubblica il messaggio su MQTT
        publish_to_mqtt "$msg"
        
        # Se il messaggio ROS 2 è "accendi", rispondi con "acceso" su MQTT
        if [ "$msg" = "accendi" ]; then
            publish_to_mqtt "acceso"
        fi
    fi
done