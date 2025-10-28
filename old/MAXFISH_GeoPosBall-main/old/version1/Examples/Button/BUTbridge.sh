#!/bin/bash

# Sorgente delle variabili d'ambiente ROS 2
source /opt/ros/humble/setup.bash

# Configurazione MQTT
MQTT_HOST="192.168.26.91"
MQTT_PORT="1883"
MQTT_TOPIC_SUB="/m5stickc/button"

# Configurazione ROS 2
ROS2_TOPIC_PUB="/m5stickc/button"
ROS2_MSG_TYPE="std_msgs/String"

# Funzione per pubblicare su ROS 2
publish_to_ros2() {
    local msg=$1
    echo "Pubblicazione su ROS 2: $msg"
    ros2 topic pub --once $ROS2_TOPIC_PUB $ROS2_MSG_TYPE "{data: '$msg'}"
}

# Ascolta i messaggi MQTT e pubblicali su ROS 2
echo "Avvio ascolto su MQTT e pubblicazione su ROS 2..."
mosquitto_sub -h $MQTT_HOST -p $MQTT_PORT -t $MQTT_TOPIC_SUB | while read -r line; do
    echo "Ricevuto messaggio MQTT: $line"
    publish_to_ros2 "$line"
done
