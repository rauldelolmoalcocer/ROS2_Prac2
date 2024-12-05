close all;
%tftree = ros2tf(node_matlab);
map = occupancyMap(45,45,20);
map.GridLocationInWorld = [-10,-10];

figureHandle = figure('Name', 'Map');
axesHandle = axes('Parent', figureHandle);
mapHandle = show(map, 'Parent', axesHandle);
title(axesHandle, 'OccupancyGrid: Update 0');

updateCounter = 1;
while(1)
    
    % Leer en msg_odom el último mensaje de odometría
    msg_odom = sub_odom.LatestMessage;
    
    % Lee en msg_laser el último mensaje del láser
    msg_laser = sub_laser.LatestMessage;

     
    % Obtener del mensaje de odometría la posición del robot [x y yaw]
    odomQuat = [msg_odom.pose.pose.orientation.w, msg_odom.pose.pose.orientation.x, ...
        msg_odom.pose.pose.orientation.y, msg_odom.pose.pose.orientation.z];
    odomRotation = quat2eul(odomQuat);
        % Establecer la pose actual [x, y, theta] del robot
    pose_robot = [msg_odom.pose.pose.position.x msg_odom.pose.pose.position.y odomRotation(1)];

    % Procesar los datos del láser para eliminar datos fuera de rango y
    % girar en el caso de que sea necesario (robot real)
    scans=rosReadLidarScan(msg_laser);
    scans = removeInvalidData(scans,'RangeLimits',[0 11.5]);
    scans = transformScan(scans,[0 0 giro_laser]);
    
    % Insertar la medida del laser en el mapa utilizando 'insertRay',
    % pasandole los datos apropiados obtenidos anteriormente
    insertRay(map,pose_robot,scans,10);

    % Visualizamos el mapa cada 50 actualizaciones.
    if ~mod(updateCounter,20)
        mapHandle.CData = occupancyMatrix(map);
        title(axesHandle, ['OccupancyGrid: Update ' num2str(updateCounter)]);
    end
    updateCounter = updateCounter+1;
        
    % Wait for control rate to ensure 10 Hz rate
    waitfor(r);
end

show(map, 'Parent', axesHandle);
title(axesHandle, 'OccupancyGrid: Final Map');

save mapa_posiciones_conocidasProfeRosbagFINAL.mat map
