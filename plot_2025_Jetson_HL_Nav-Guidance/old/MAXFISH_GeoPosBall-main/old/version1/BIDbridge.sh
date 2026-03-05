source /opt/ros/humble/setup.bash

MQTT_HOST="192.168.11.1"
MQTT_PORT="1883"
MQTT_TOPIC_PUB="/mqtt2"
MQTT_TOPIC_SUB="/mqtt1"

ROS2_TOPIC_PUB="/locdata"
ROS2_TOPIC_SUB="/servocmd"
ROS2_MSG_TYPE="std_msgs/String"

publish_to_ros2() {
     local msg=$1
     ros2 topic pub --once $ROS2_TOPIC_PUB $ROS2_MSG_TYPE "data: '$msg'"
}

publish_to_mqtt() {
     local msg=$1
     mosquitto_pub -h $MQTT_HOST -p $MQTT_PORT -t $MQTT_TOPIC_PUB -m "$msg"
}


mosquitto_sub -h $MQTT_HOST -p $MQTT_PORT -t $MQTT_TOPIC_SUB | while read -r msg
do
     echo "Ricevuto messaggio MQTT: $msg"
     publish_to_ros2 "$msg"
done &


ros2 topic echo $ROS2_TOPIC_SUB --qos-profile sensor_data | while read -r line
do
   if [[ $line == data:* ]]; then
       msg=${line#data: }
       echo "Ricevuto messaggio ROS 2: $msg"
       publish_to_mqtt "$msg"
   fi
done
