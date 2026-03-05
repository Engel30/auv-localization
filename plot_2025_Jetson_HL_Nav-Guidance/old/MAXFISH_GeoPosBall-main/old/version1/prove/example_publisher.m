node_1 = ros2node("/node_1")
chatterPub = ros2publisher(node_1,"/chatter","std_msgs/String");
msg = ros2message(chatterPub);
msg.data = 'hello world';
while 1
    send(chatterPub,msg);
end