ros2Node = ros2node('/matlab_node');
% Topic to subscribe
topicName = '/locdata';

% Create a publisher
rosTopicName = '/servocmd';
rosPub = ros2publisher(ros2Node, rosTopicName, 'std_msgs/String');
messageToSend = 'Hello from MATLAB';

% Subscribe to the topic
sub = ros2subscriber(ros2Node, topicName, @messageCallback);

disp('Subscribed to the topic, waiting for messages...');
disp('Ctrl+C to stop.');

while true
    % Send data to topic /servocmd
    msg = ros2message(rosPub);
    msg.data = messageToSend;
    send(rosPub, msg);

    pause(1);
end


% Read data from topic /locdata
function messageCallback(message)
    disp('Received message:');
    disp(message);
end