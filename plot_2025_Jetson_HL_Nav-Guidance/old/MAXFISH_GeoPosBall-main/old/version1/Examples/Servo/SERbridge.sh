#!/bin/bash

# Sorgente delle variabili d'ambiente ROS 2
source /opt/ros/humble/setup.bash

# Configurazione MQTT
MQTT_HOST="192.168.61.91"
MQTT_PORT="1883"
MQTT_TOPIC_PUB="/servo_angle"
MQTT_TOPIC_SUB="/servo_angle"

# Configurazione ROS 2
ROS2_TOPIC_SUB="/servo_angle"
ROS2_MSG_TYPE="std_msgs/String"

# Verifica se il topic esiste e qual è il suo tipo
if ! ros2 topic info $ROS2_TOPIC_SUB; then
    echo "Topic $ROS2_TOPIC_SUB non esiste o il tipo non può essere determinato"
    exit 1
fi

# Funzione per pubblicare su MQTT
publish_to_mqtt() {
    local msg=$1
    echo "Pubblicazione su MQTT: $msg"
    mosquitto_pub -h $MQTT_HOST -p $MQTT_PORT -t $MQTT_TOPIC_PUB -m "$msg"
}

# Ascolta i messaggi ROS 2 e pubblicali su MQTT
echo "Avvio ascolto su ROS 2 e pubblicazione su MQTT..."
ros2 topic echo $ROS2_TOPIC_SUB --qos-profile sensor_data | while read -r line
do
    if [[ $line == *"data:"* ]]; then
        msg=${line#*data: }
        msg=$(echo $msg | xargs) # Rimuove eventuali spazi bianchi
        echo "Ricevuto messaggio ROS 2: $msg"

        # Controlla che l'angolo sia valido (tra 0 e 180 gradi)
        if [[ "$msg" =~ ^[0-9]+$ ]] && [ "$msg" -ge 0 ] && [ "$msg" -le 180 ]; then
            publish_to_mqtt "$msg"
        else
            echo "Angolo non valido ricevuto: $msg"
        fi
    fi
done