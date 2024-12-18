close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Crear figuras distintas para el láser y el visualizador del VFH
% NOTA: para dibujar sobre una figura concreta, antes de llamar a la
% correspondiente función de dibujo debe convertirse en la figura activa
% utilizando figure(fig_laser) o figure(fig_vfh) respectivamente.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
fig_laser=figure; title('LASER')
fig_vfh=figure; title('VFH')
fig_mapa=figure;

%Definir la posicion de destino
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%startLocation = [0 0];
endLocation = [-5 12];

XnumNOdes = 200;
Xmetros = 20;

%Inicializar valores X, Y, Z
umbralx=0.05
umbraly=0.05
umbralyaw=0.05

%Cargar el mapa
load mapa_pos_real_SLAMONLineRosbagPropioFINAL.mat
show(map);

% Crear el objeto VFH…
VFH=controllerVFH;

% Ajustar propiedades del objeto VFH
VFH.NumAngularSectors=180;
VFH.DistanceLimits=[0.05 1.5];
VFH.RobotRadius=0.1;
VFH.SafetyDistance=0.1;
VFH.MinTurningRadius=0.1;
VFH.TargetDirectionWeight=50;
VFH.CurrentDirectionWeight=2;
VFH.PreviousDirectionWeight=2;
VFH.HistogramThresholds=[3 10];
VFH.UseLidarScan=[true]; %para permitir utilizar la notación del scan

%Inicializar el localizador AMCL (práctica 1) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


odometryModel = odometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];

rangeFinderModel = likelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0 11.8];
rangeFinderModel.Map = map;

rangeFinderModel.SensorPose = [0.005 0 0];

amcl = monteCarloLocalization;
amcl.UseLidarScan = true;

amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;

amcl.UpdateThresholds = [0.2,0.2,0.2];
amcl.ResamplingInterval = 1;

amcl.ParticleLimits = [500 50000];           % Minimum and maximum number of particles
amcl.GlobalLocalization = false;      % global = true      local=false
amcl.InitialPose = [4 2 0];              % Initial pose of vehicle   
amcl.InitialCovariance = diag([1 1 1])*3; % Covariance of initial pose


visualizationHelper = ExampleHelperAMCLVisualization(map);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Crear el objeto PurePursuit y ajustar sus propiedades
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
controller=controllerPurePursuit;
controller.LookaheadDistance =2
controller.DesiredLinearVelocity=0.1
controller.MaxAngularVelocity=0.3

% Rellenar los campos por defecto de la velocidad del robot, para que la lineal
%sea siempre 0.1 m/s
msg_vel.linear.x=0.1;
msg_vel.linear.y=0;
msg_vel.linear.z=0;

msg_vel.angular.x=0;
msg_vel.angular.y=0;
msg_vel.angular.z=0.1;
%Bucle de control infinito
while(1)
    %Leer y dibujar los datos del láser en la figura ‘fig_laser’
    msg_laser = sub_laser.LatestMessage;
    scans= rosReadLidarScan(msg_laser); %Crear objeto lidarScan
    figure(fig_laser);
    plot(scans); %Visualizar datos del láser

    %Leer la odometría

    % Receive laser scan and odometry message.
    msg_laser = receive(sub_laser);
    msg_odom = sub_odom.LatestMessage;
    
    %Crear objeto para almacenar el escaneo LiDAR 2-D
    scans = rosReadLidarScan(msg_laser);  % Extraer datos del mensaje del laser
    scans = removeInvalidData(scans, 'RangeLimits', [0, 11.5]);  % Quitar datos fuera de rango
    scans = transformScan(scans, [0 0 giro_laser]);  % Girar datos laser si es necesario
    
    
    % Compute robot's pose [x,y,yaw] from odometry message.
    
   % Obtener del mensaje de odometría la posición del robot [x y yaw]
    odomQuat = [msg_odom.pose.pose.orientation.w, msg_odom.pose.pose.orientation.x, ...
        msg_odom.pose.pose.orientation.y, msg_odom.pose.pose.orientation.z];
    odomRotation = quat2eul(odomQuat);
        % Establecer la pose actual [x, y, theta] del robot
    pose_robot = [msg_odom.pose.pose.position.x msg_odom.pose.pose.position.y odomRotation(1)];

    % Update estimated robot's pose and covariance using new odometry and
    % sensor readings.
    [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose_robot, scans);
    
    %Mostrar los datos por pantalla
    estimatedCovariance
    xprint=estimatedCovariance(1,1)
    yprint=estimatedCovariance(2,2)
    orientprint=estimatedCovariance(3,3)

    % Drive robot to next pose.
    %wander(wanderHelper);
    if (estimatedCovariance(1,1)<umbralx && estimatedCovariance(2,2)<umbraly && estimatedCovariance(3,3)<umbralyaw) 
        disp("Robot Localizado");
        break; 
    end
     %Dibujar los resultados del localizador con el visualizationHelper 
        % Plot the robot's estimated pose, particles and laser scans on the map.
    if isUpdated
        i = i + 1
        plotStep(visualizationHelper, amcl, estimatedPose, scans, i)
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5



%Llamar al objeto VFH para obtener la dirección a seguir por el robot para
%evitar los obstáculos. Mostrar los resultados del algoritmo (histogramas)
%en la figura ‘fig_vfh’
 % Obtener dirección de navegación
%Rellenar el campo de la velocidad angular del mensaje de velocidad con un
%valor proporcional a la dirección anterior (K=0.1)
targetDir = 0;
K = 0.1;
steeringDir = VFH(scans, targetDir);
v_ang = K * steeringDir;
figure(fig_vfh);
show(VFH);

%Publicar el mensaje de velocidad
send(pub_vel, msg_vel);
%Esperar al siguiente periodo de muestreo
waitfor(r);
end


%%%%%%%%%%% AL SALIR DE ESTE BUCLE EL ROBOT YA SE HA LOCALIZADO %%%%%%%%%%
%%%%%%%%%%% COMIENZA LA PLANIFICACIÓN GLOBAL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Paramos el robot, para que no avance mientras planificamos
msg_vel.linear.x=0;
msg_vel.linear.y=0;
msg_vel.linear.z=0;

msg_vel.angular.x=0;
msg_vel.angular.y=0;
msg_vel.angular.z=0;
%Publicar el mensaje de velocidad
send(pub_vel, msg_vel);
%Hacemos una copia del mapa, para “inflarlo” antes de planificar
cpMap= copy(map);
inflate(cpMap,0.25);
%Crear el objeto PRM y ajustar sus parámetros
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
planner = mobileRobotPRM;
planner.Map= cpMap;
planner.NumNodes= 500;
planner.ConnectionDistance = 3;
%Obtener la ruta hacia el destino desde la posición actual del robot y mostrarla
%en una figura
startLocation = estimatedPose(1:2);

ruta=findpath(planner,startLocation,endLocation);
figure; 
show(planner);

%%%%%%%%%%% COMIENZA EL BUCLE DE CONTROL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Indicamos al controlador la lista de waypoints a recorrer (ruta)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
controller.Waypoints=ruta
%Bucle de control
%%%%%%%%%%%%%%%%%
while(1)
%Leer el láser y la odometría
    msg_laser = receive(sub_laser);
    msg_odom = sub_odom.LatestMessage;

    scans = rosReadLidarScan(msg_laser);  % Extraer datos del mensaje del laser
    scans = removeInvalidData(scans, 'RangeLimits', [0, 11.5]);  % Quitar datos fuera de rango
    scans = transformScan(scans, [0 0 giro_laser]);  % Girar datos laser si es necesario
    
%Obtener la posición pose=[x,y,yaw] a partir de la odometría anterior
    odomQuat = [msg_odom.pose.pose.orientation.w, msg_odom.pose.pose.orientation.x, ...
        msg_odom.pose.pose.orientation.y, msg_odom.pose.pose.orientation.z];
    odomRotation = quat2eul(odomQuat);
        % Establecer la pose actual [x, y, theta] del robot
    pose_robot = [msg_odom.pose.pose.position.x msg_odom.pose.pose.position.y odomRotation(1)];

%Ejecutar amcl para obtener la posición estimada estimatedPose
    [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose_robot, scans);


    if isUpdated
        i = i + 1
        plotStep(visualizationHelper, amcl, estimatedPose, scans, i)
    end
    
%Ejecutar el controlador PurePursuit para obtener las velocidades lineal
%y angular

[lin_vel,ang_vel]=controller(estimatedPose);
%Rellenar los campos del mensaje de velocidad
msg_vel.linear.x=lin_vel;

steeringDir = VFH(scans, 0.75*ang_vel);
v_ang = 0.35 * steeringDir;
figure(fig_vfh);
show(VFH);

msg_vel.angular.z=v_ang;
%Publicar el mensaje de velocidad
send(pub_vel, msg_vel);

%Comprobar si hemos llegado al destino, calculando la distancia euclidea
%y estableciendo un umbral
Goal=endLocation

if (sqrt((Goal(1)-estimatedPose(1))^2+(Goal(2)-estimatedPose(2))^2) < 0.2)
%Parar el robot
break;
end
%Esperar al siguiente periodo de muestreo
waitfor(r);

end