% Connettersi al dispositivo ROS 2
ros2device('192.168.61.91', 'marco', 'Ubuntu');

% Imposta le variabili d'ambiente ROS 2
setenv('ROS_DOMAIN_ID', '0');  % Usa lo stesso numero di dominio DDS impostato in ROS 2
setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp');

% Creare un nodo ROS2
node = ros2node("/matlab_node");

% Creare un publisher per inviare l'angolo del servo
pub_angle = ros2publisher(node, "/servo_angle", "std_msgs/String");

% Creare un messaggio per l'angolo del servo
msg_angle = ros2message(pub_angle);


% Impostare l'angolo desiderato
desiredAngle = '180';
msg_angle.data = desiredAngle;

% Inviare il messaggio per impostare l'angolo del servo
send(pub_angle, msg_angle);

