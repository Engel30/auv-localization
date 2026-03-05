% Connetti al dispositivo ROS 2
ros2device('192.168.61.91', 'marco', 'Ubuntu');

% Imposta le variabili d'ambiente ROS 2
setenv('ROS_DOMAIN_ID', '0');  % Assicurati che questo sia lo stesso numero di dominio DDS impostato in ROS 2
setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'); % Usa 'rmw_fastrtps_cpp' o il valore appropriato

% Crea un nodo ROS 2
ros2Node = ros2node('/led_control');

% Nome del topic ROS 2
rosTopicName = '/accensione';

% Crea un publisher
ledPub = ros2publisher(ros2Node, rosTopicName, 'std_msgs/String');

% Messaggio da inviare
messageToSend = 'spegni';

% Crea un messaggio ROS
msg = ros2message(ledPub);
msg.data = messageToSend;

% Pubblica il messaggio
send(ledPub, msg);

