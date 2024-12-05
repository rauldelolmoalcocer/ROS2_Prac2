% Declaración de SUBSCRIBERS (a la odometria y el laser)
sub_odom = ros2subscriber(node_matlab, "/odom", "Reliability","besteffort");
sub_laser = ros2subscriber(node_matlab, "/scan");

% Imagen
% sub_cam= ros2subscriber(node_sim, "/camera1");

% Declaración de PUBLISHERS (a la velocidad)
pub_vel =  ros2publisher(node_matlab, "/cmd_vel", "geometry_msgs/Twist");

% Creación del MENSAJE de velocidad
msg_vel = ros2message(pub_vel);

% PERIODICIDAD del timer
r = rateControl(10);


giro_laser = pi/2;