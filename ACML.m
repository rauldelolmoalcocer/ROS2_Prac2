load mapa_pos_real_SLAMONLineProfeRosBagFINAL.mat
show(map);

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
amcl.GlobalLocalization = true;      % global = true      local=false
amcl.InitialPose = [0 0 0];              % Initial pose of vehicle   
amcl.InitialCovariance = diag([1 1 1])*0.5; % Covariance of initial pose

visualizationHelper = ExampleHelperAMCLVisualization(map);

i=1;
while (1)
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
    
    % Drive robot to next pose.
    %wander(wanderHelper);
    
    % Plot the robot's estimated pose, particles and laser scans on the map.
    if isUpdated
        i = i + 1
        plotStep(visualizationHelper, amcl, estimatedPose, scans, i)
    end

    % Wait for control rate to ensure 10 Hz rate
    waitfor(r);
end