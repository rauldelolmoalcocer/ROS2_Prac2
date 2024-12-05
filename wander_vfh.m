%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Crear figuras distintas para el láser y el visualizador del VFH
% NOTA: para dibujar sobre una figura concreta, antes de llamar a la
% correspondiente función de dibujo debe convertirse en la figura activa
% utilizando figure(fig_laser) o figure(fig_vfh) respectivamente.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
fig_laser=figure; title('LASER')
fig_vfh=figure; title('VFH')


% Crear el objeto VFH…
VFH=controllerVFH;

% Ajustar propiedades del objeto VFH
VFH.NumAngularSectors=180;
VFH.DistanceLimits=[0.05 2];
VFH.RobotRadius=0.1;
VFH.SafetyDistance=0.1;
VFH.MinTurningRadius=0.1;
VFH.TargetDirectionWeight=5;
VFH.CurrentDirectionWeight=2;
VFH.PreviousDirectionWeight=2;
VFH.HistogramThresholds=[3 10];
VFH.UseLidarScan=[true]; %para permitir utilizar la notación del scan

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