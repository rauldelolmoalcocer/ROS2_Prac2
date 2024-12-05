% Establecer conexion con el robot
% Limpiamos los espacios de trabajo

% Nos conectamos al dominio
setenv("ROS_DOMAIN_ID", "65");

% Generamos el nodo
node_matlab = ros2node("/node_matlabRauld"); %Cambiado la parte de Rauld.
pause(12)

% Listar nodos activos
disp("Nodos activos:");
ros2 node list

% Listar topics
disp("Topics:");
ros2 topic list