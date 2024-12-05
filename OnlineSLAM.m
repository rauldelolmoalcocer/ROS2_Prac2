close all;


maxLidarRange = 11.5 ;
mapResolution = 12 ;
slamObj = lidarSLAM(mapResolution, maxLidarRange)

slamObj.LoopClosureThreshold = 200 ;
slamObj.LoopClosureSearchRadius = 3 ;

firstLoopClosure = false;

figure
while (1)
    msg_laser = sub_laser.LatestMessage;
    % Create lidarScan object from scan message
    scans= rosReadLidarScan(msg_laser);

    scans=transformScan(scans, [0 0 pi/2]); %cambiar transformada solo a robot real comentar la linea
      
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamObj, scans);
    if isScanAccepted
        % Visualize how scans plot and poses are updated as robot navigates
        % through virtual scene
        show(slamObj);
        
        % Visualize the first detected loop closure
        % firstLoopClosure flag is used to capture the first loop closure event
        if optimizationInfo.IsPerformed && ~firstLoopClosure
            firstLoopClosure = true;
            show(slamObj, 'Poses', 'off');
            hold on;
            show(slamObj.PoseGraph);
            hold off;
            title('First loop closure');
            snapnow
        end
    end

    waitfor(r);
end

[scans, optimizedPoses] = scansAndPoses(slamObj);
map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);


save mapa_pos_real_SLAMONLineProfeRosBagFINAL.mat map




% Plot the final built map after all scans are added to the |slamObj|
% object.
%show(slamObj, 'Poses', 'off'); 
%hold on;
%show(slamObj.PoseGraph); 
%hold off;
%title({'Final Built Map of the Environment', 'Trajectory of the Robot'});

