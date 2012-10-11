function [...
    bicycleSteeringAngle,...
    bicycleRollAngle,...
    bicycleSpeed,...
    bicycleCadence,...
    ant_t,...
    figures] = ...
    ...
    getRollAndSteeringAngles(figures,...
    filename,...
    runName,...
    Fs_plot,...
    Fs_ImuMeasured,...
    steeringImuNodeId,...
    rollingImuNodeId,...
    callibrateStart,...
    plotRuns,...
    synchroniseTheImus,...
    imuOrAdams)
%GETBICYCLEANGLES Processes a run in an experiment and calculates the
% steering and roll angles with the plots as well.
% RETURNS:
% steering_angle - The Steering angle for the run.
% roll_angle - The roll angle for the run.
% speed - The forward velocity for the run.
% cadence - The pedaling cadence for the run.
% ant_t - The time vector for the speed and cadence measurements.
% figures - A list of the figures handles, in case you want to export
% by using laprint or something.
finalPlotStyle = '--r.'
display(['%%%%%%%%%%%%%%Processing RUN:' runName ' %%%%%%%%%%%%%%'])
display('CREATING FIGURES')
if isempty(figures)
    figFinalAngles=figure('visible','on','WindowStyle','docked',...
        'Name','Roll, Pitch and Steering Angle of the Bicycle.');
    figSync = figure('visible','on','WindowStyle','docked',...
        'Name','Synchronisation');
    figSpeed = figure('visible','on','WindowStyle','docked',...
        'Name','Forward velocity.');
    figures = [figFinalAngles,figSync,figSpeed];
else
    delete(figures(1));
    figFinalAngles=figures(1);
    delete(figures(2));
    figSync = figures(2);
    delete(figures(5));
    figRollSteerSync = figures(5);
    delete(figures(6));
    figSpeed = figures(6);
end
display('FIGURES CREATED')

runNamePrint = strrep(runName,'/','-');
runNamePrint = strrep(runNamePrint,'_','-');
processedFilename=['.' runName '-getbicycleangles-processed.mat'];
%Get the data
display('GETTING DATA');
if exist(processedFilename,'file')
    load(processedFilename);
else
    save(processedFilename,'runName');
    if imuOrAdams>0
        [steering_t,steering_sync_t] = ...
            Quat3D.readDataPromove(filename,runName,...
            steeringImuNodeId,Fs_plot,Fs_ImuMeasured);
        [roll_t,roll_sync_t] = ...
            Quat3D.readDataPromove(filename,runName,...
            rollingImuNodeId,Fs_plot,Fs_ImuMeasured);
        theAntReader = antReader(filename,runName);
        [bicycleSpeed,bicycleCadence,ant_t,ant_sync,ant_sync_t] = ...
            theAntReader.readVelocityCadence();
        save(processedFilename,'bicycleSpeed','-append');
        save(processedFilename,'bicycleCadence','-append');
        save(processedFilename,'ant_t','-append');
        save(processedFilename,'ant_sync','-append');
        save(processedFilename,'ant_sync_t','-append');
    else
        [steering_t,steering_sync_t] = ...
            Markers3D.readDataAdams(filename,runName,...
            'RBO','LBO','FON');;
        [roll_t,roll_sync_t] = ...
            Markers3D.readDataAdams(filename,runName,...
            'RBT','LBT','FTN');;
    end
    
    save(processedFilename,'steering_t','-append');
    save(processedFilename,'steering_sync_t','-append');
    save(processedFilename,'roll_t','-append');
    save(processedFilename,'roll_sync_t','-append');
    
end
display('DATA RECEIVED');
display('PLOTTING FORWARD INFORMATION');
if imuOrAdams>0
    %Synchronise:
    if any(ant_sync_t)
        bicycleSpeed =  bicycleSpeed(ant_t>=ant_sync_t(1));
        bicycleCadence =  bicycleCadence(ant_t>=ant_sync_t(1));
        ant_t =  ant_t(ant_t>=ant_sync_t(1));
    end
    ant_t =ant_t-repmat(ant_t(1),size(ant_t));
    set(0,'CurrentFigure',figSpeed)
    subplot(2,1,1)
    plot(ant_t,bicycleSpeed);
    title('Forward Velocity: ');
    subplot(2,1,2)
    plot(ant_t,bicycleCadence);
    title('Pedaling Cadence: ');
    display('FORWARD VELOCITY PLOTTED');
end
if imuOrAdams>0
    if any(roll_sync_t)
        display('SYNCING SENSORS ON MANUAL VALUES');
        if any(steering_sync_t)
            [roll_t,steering_t] = synchronise(roll_sync_t,...
                steering_sync_t,roll_t,steering_t,...
                Fs_plot,1,1);
        end
        display('FINISHED MANUAL SYNC');
    end
    if synchroniseTheImus
        display('SYNCING SENSORS');
        set(0,'CurrentFigure',figSync);
        hold on;
        [roll_t,steering_t,rollMax] = synchronise(roll_sync_t,...
            steering_sync_t,roll_t,steering_t,...
            Fs_plot,1,1);
        %     [roll_t_r,steering_t_r,rollMax] = synchronise(roll_r,...
        %         roll_s,roll_t,steering_t,...
        %         Fs_plot,1,1);
        %     [roll_t_p,steering_t_p,pitchMax] = synchronise(pitch_r,...
        %         pitch_s,roll_t,steering_t,...
        %         Fs_plot,1,1);
        %     [roll_t_y,steering_t_y,yawMax] = synchronise(yaw_r,...
        %         yaw_s,roll_t,steering_t,...
        %         Fs_plot,1,1);
        %     totalMax = max([rollMax pitchMax yawMax]);
        %     if rollMax == totalMax
        %         display('SYNCING ON ROLL ANGLE');
        %         roll_t = roll_t_r;
        %         steering_t = steering_t_r;
        %     elseif pitchMax == totalMax
        %         display('SYNCING ON PITCH ANGLE');
        %         roll_t = roll_t_p;
        %         steering_t = steering_t_p;
        %     else
        %         display('SYNCING ON YAW ANGLE');
        %         roll_t = roll_t_y;
        %         steering_t = steering_t_y;
        %     end
        %     display('FINISHED');
        
        display('PLOTTING ROLL PITCH YAW: SENSORS SYNCHRONISED');
        [frameSensorRollAngle,pitch_r,yaw_r]=ThreeD.getRPYt(roll_t,true);
        [roll_s,pitch_s,yaw_s]=ThreeD.getRPYt(steering_t,true);
        set(0,'CurrentFigure',figRollSteerSync)
        hold on;
        ThreeD.plotRPY(...
            frameSensorRollAngle,pitch_r,yaw_r,true,Fs_plot);
        ThreeD.plotRPY(...
            roll_s,pitch_s,yaw_s,true,Fs_ImuMeasured);
        display('FINISHED PLOTTING ROLL PITCH YAW: SENSORS SYNC');
    end
end

[timestamps,steeringJoint_t,...
    steeringJointRollAngle,...
    steeringJointPitchAngle,...
    steeringJointYawAngle,...
    frameSensorRollAngle,frameSensorPitchAngle, frameSensorYawAngle] = ...
    getjointangles(roll_t,steering_t,'Steering Angle',callibrateStart);


bicycleSteeringAngle = steeringJointYawAngle;
bicycleRollAngle = frameSensorRollAngle;
set(0,'CurrentFigure',figFinalAngles)
ThreeD.plotRPY(bicycleRollAngle,frameSensorPitchAngle,bicycleSteeringAngle,...
    timestamps,true,'timeseries',finalPlotStyle);

if plotRuns==1
    display('PLOT STEERING QUAT');
    minSize = min(size(steering_t,2),size(roll_t,2));
    figure
    ThreeD.plotRun([roll_t(1:minSize);steering_t(1:minSize);
        diff_t(1:minSize)
        ]);
    display('STEERING QUAT PLOTTED');
end