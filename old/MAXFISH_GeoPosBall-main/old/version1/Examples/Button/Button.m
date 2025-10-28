% Connetti al dispositivo ROS 2
device = ros2device('192.168.61.91', 'marco', 'Ubuntu');

% Imposta le variabili d'ambiente ROS 2
setenv('ROS_DOMAIN_ID', '0');  % Usa lo stesso numero di dominio DDS impostato in ROS 2
setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp');

% Crea un nodo ROS2
ros2Node = ros2node('/matlab_node');

% Sottoscrivi al topic ROS2 che riceve i messaggi MQTT
sub = ros2subscriber(ros2Node, '/m5stickc/button', @messageCallback);

% Funzione callback per gestire i messaggi ricevuti
function messageCallback(message)
    disp('Received message:');
    disp(message);
    % Verifica il contenuto del messaggio e stampa il messaggio appropriato su MATLAB
    if strcmp(message.data, 'Button A pressed')
        disp('Tasto centrale premuto');
    end
    if strcmp(message.data, 'Button B pressed')
        disp('Tasto a destra premuto');
    end
end
