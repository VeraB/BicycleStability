tic
clear all;
close all;
format compact;
format long;
initialiseprocessing();


filename = 'D:\Sofie-HDF-Format\win32build\Visualeyze_tests.h5'


VisualEyezRun = '/AdrianSlow29';

StaticRun = '/AdrianZeroMeasurement04012013_2';

Track = '/track2'

MovingLeg = '/AdrianMovingLeg04012013';

% Pre processing the raw data and create 3D objects (interpolation and filtering)
% The following variables can be set: 
% - lengthThreshold (14 mm): threshold for the difference between the lengths of the triangle)
% - maxGap(10): max gap to interpolate
% - interpMethod ('linear'): interpolation method
% - maxVelJump (1000 mm): max velocity jump between to samples
% - Fs (100 Hz): sample frequency
% - freqLowPass (15): frequency low pass filter
% - orderLowPass (4): order low pass filter

plotOrNot = 0; % true when preprocessing is plotted

[Roll,t_Roll] = BicycleStability.visualeyzePreProcessing(filename,VisualEyezRun,...
                'Roll','plotOrNot',true);
            
                         [roll_Roll,pitch_Roll,yaw_Roll,t_Roll]=ThreeD.getAndPlotRPYt(Roll,'Roll',...
     false,'timeseries','*b');
         

% create resampling vector   
Fs = 100;
t_Roll = [t_Roll(1):1/Fs:t_Roll(length(t_Roll))];

 Roll_resampled = ThreeD.resample(Roll,t_Roll);

% Get and plot the Roll, Pitch, Yaw 
% After the resampling

ResampledPlotStyle = '--b*'

 [roll_RollR,pitch_RollR,yaw_RollR,t_RollR]=ThreeD.getAndPlotRPYt(Roll_resampled,'Roll Resampled',...
     false,'timeseries',ResampledPlotStyle);

% pre-process the Roll sensor of the static run

[RollStatic,t_RollStatic] = BicycleStability.visualeyzePreProcessing(filename,StaticRun,...
                'Roll','plotOrNot',false);

  
            % plot track
             BicycleStability.plotExperimentPath(filename,Track, 'YesOrNoTrack', true);
            
            % pre process the track run
             [rawTrack] = RawMarkers.readFromFile(filename,Track,...
                             'Channel103','Channel104','Channel101');           
            [TrackAll,TrackSome,TrackOutliers] = ...
                RawMarkers.areMarkersDropped(rawTrack,'yesOrNoPlot',plotOrNot,...
                'sensorName','sensorName','maxVelJump', 1000, 'Fs', 100);
            [gapTrack] = RawMarkers.findGaps(rawTrack);
            [TrackWellSpaced,distances]=...
                RawMarkers.areTheMarkersWellSpaced(gapTrack,'lengthThreshold',14,...
                'plotTheDistances',plotOrNot,'plotBoxPlot',plotOrNot,'sensorName', ['sensorName' ' - rawData'],...
                'withRespectToMean',true);
            [spacedTrack] = RawMarkers.removeNotWellSpaced(gapTrack,...
                TrackWellSpaced);
            [interpTrack] = RawMarkers.interpolateGaps(spacedTrack,rawTrack,'plotOrNot',plotOrNot,...
                'sensorName','sensorName');
            [filtTrack] = RawMarkers.filterRawData(interpTrack,'plotOrNot',plotOrNot,'sensorName','sensorName');
            [filtTrackN,tTrackOriginal] = RawMarkers.eraseNan(filtTrack);
            Track = Markers3D.create3DMarkersFromRawData(filtTrackN);
            [rollTrack,pitchTrack,yawTrack,tTrack] = ...
                ThreeD.getAndPlotRPYt(Track,'Track',false,'timeseries','*b',...
                'plotDropped', true, 'yesOrNoAll', TrackAll, 'yesOrNoSome', TrackSome,...
                'yesOrNoOutlier', TrackOutliers,'yesOrNoWellSpaced',TrackWellSpaced,...
                't_Original',tTrackOriginal);

             % Calibrate the track to obtain the global reference frame and plot RPY
             % (these should be zero for the first part, then going into a 90 degree corner)
             % so this will show the yaw angle
             [TrackCalibrated,tm_estGRF] = ...
                ThreeD.zeroTheRun(Track,100,100);
            [roll_track,pitch_track,yaw_track,t_track]=ThreeD.getAndPlotRPYt(TrackCalibrated,'Track Callibrated',...
    false,'timeseries',ResampledPlotStyle);

            % calibration of the static roll angle to the GRF
            
             [roll_Roll,pitch_Roll,yaw_Roll,t_Roll]=ThreeD.getAndPlotRPYt(RollStatic,'Roll static',...
     false,'timeseries',ResampledPlotStyle);
%             [Roll_callibratedS,tm_est] = ...
%                 ThreeD.zeroTheRun(RollStatic,100,100,'optionStaticRun',true,'tm_tStatic',...
%                 Track);
                 parfor i =  1:length(RollStatic)
                Roll_St = tm_estGRF.*RollStatic{i};
                Roll_Static_GRF{i} = Roll_St;
                 end

             [roll_Roll_Static_GRF,pitch_Roll_Static_GRF,yaw_Roll_Static_GRF,t_Roll_Static_GRF]=ThreeD.getAndPlotRPYt(Roll_Static_GRF,'Roll static callibrated to GRF',...
     false,'timeseries',ResampledPlotStyle);  
 
 % calibrate static_GRF to zero
              [Roll_Static_calibrated,tm_staticGRF] = ...
                ThreeD.zeroTheRun(Roll_Static_GRF,200,300);
            
             [roll_Roll,pitch_Roll,yaw_Roll,t_Roll]=ThreeD.getAndPlotRPYt(Roll_Static_calibrated,'Roll static callibrated to itself',...
     false,'timeseries',ResampledPlotStyle);              
 
% calibration of the  roll angle to the GRF
            


                 parfor i =  1:length(Roll_resampled)
                Roll_C = tm_estGRF.*Roll_resampled{i};
                Roll_Cal_GRF{i} = Roll_C;
                 end

             [roll_Roll_Cal_GRF,pitch_Roll_Cal_GRF,yaw_Roll_Cal_GRF,t_Roll_Cal_GRF]=ThreeD.getAndPlotRPYt(Roll_Cal_GRF,'Roll callibrated to GRF',...
     false,'timeseries',ResampledPlotStyle);  
 
 % To obtain the roll angle, subtract the roll angle of the static
 % measurement. (in this case the roll angle is around the x-axis, so is
 % called pitch...)
 Roll_angle = pitch_Roll_Cal_GRF - mean(pitch_Roll_Static_GRF(300:400));
%   Yaw_angle = roll_Roll_Cal_GRF - mean(roll_Roll_Static_GRF(300:400));
%   Pitch_angle = yaw_Roll_Cal_GRF - mean(yaw_Roll_Static_GRF(300:400));
%   
%   
%  figure('visible','on','WindowStyle','docked',...
%                     'Name', 'Calibrated roll angle');
%                 plot(t_RollR,Roll_angle,'-*');
%                  figure('visible','on','WindowStyle','docked',...
%                     'Name', 'Calibrated yaw angle');
%                 plot(t_RollR,Yaw_angle,'-*');
%                  figure('visible','on','WindowStyle','docked',...
%                     'Name', 'Calibrated Pitch angle');
%                 plot(t_RollR,Pitch_angle,'-*');

 %%
 % Calculate Steering Angle
 %Preprocess Steer sensor
  [Steer,t_Steer] = BicycleStability.visualeyzePreProcessing(filename,VisualEyezRun,...
                 'Steer','plotOrNot',plotOrNot);
 % Resample Steer sensor   
         % create resampling vector   
t_Steer = [t_Steer(1):1/Fs:t_Steer(length(t_Steer))];
Steer_resampled = ThreeD.resample(Steer,t_Steer);
 [roll_SteerR,pitch_SteerR,yaw_SteerR,t_SteerR]=ThreeD.getAndPlotRPYt(Steer_resampled,'Steer Resampled',...
     false,'timeseries',ResampledPlotStyle);

 %Get the steering angle
           
 [SteeringAngle] = ...
                ThreeD.getjointangles(Roll_resampled,Steer_resampled,'Steering Angle',100); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
%% 
%Get the Upper Body Lean angle
    % Pre process the Upper Body sensor
  [UpperBody,t_UpperBody] = BicycleStability.visualeyzePreProcessing(filename,VisualEyezRun,...
                 'UpperBody','plotOrNot',plotOrNot);
    % Resample UpperBody sensor       
    t_UpperBody = [t_UpperBody(1):1/Fs:t_UpperBody(length(t_UpperBody))]; % create resampling vector  
    UpperBody_resampled = ThreeD.resample(UpperBody,t_UpperBody); % resample
    ThreeD.getAndPlotRPYt(UpperBody_resampled,'UpperBody Resampled',...
     false,'timeseries',ResampledPlotStyle); % Get and plot RPY
    % Calculate Upper body lean angle with respect to the bicycle frame
             [UpperbodyFrameAngle] = ...
                ThreeD.getjointangles(Roll_resampled,UpperBody_resampled,'Upper Body Angle (wrt Frame)',100);
    % Calculate Upper body lean angle with respect to the pelvis
        % Pre process the Upper Body sensor
          [Pelvis,t_Pelvis] = BicycleStability.visualeyzePreProcessing(filename,VisualEyezRun,...
                 'Pelvis','plotOrNot',plotOrNot);
        % Resample UpperBody sensor       
        t_Pelvis = [t_Pelvis(1):1/Fs:t_Pelvis(length(t_Pelvis))]; % create resampling vector  
        Pelvis_resampled = ThreeD.resample(Pelvis,t_Pelvis); % resample
        ThreeD.getAndPlotRPYt(Pelvis_resampled,'Pelvis Resampled',...
        false,'timeseries',ResampledPlotStyle); % Get and plot RPY
        % Calculate the joint angle (Upper Body to Pelvis)
         [UpperbodyPelvisAngle] = ...
                ThreeD.getjointangles(Pelvis_resampled,UpperBody_resampled,'Upper body Angle (wrt pelvis)',100);     

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%             
%  % calibration of the roll movement to callibrated static
% 
% %  
%                 parfor i = 1:length(Roll_Cal_GRF)
%                      Roll_to_GRF = tm_staticGRF.*Roll_Cal_GRF{i};
%                      Roll_{i} = Roll_to_GRF;
%                  end 
% 
% % 
%            [roll_Roll,pitch_Roll,yaw_Roll,t_Roll]=ThreeD.getAndPlotRPYt(Roll_,'Roll callibrated to GRF',...
%      false,'timeseries',ResampledPlotStyle);  
%             [roll_Roll,pitch_Roll,yaw_Roll,t_Roll]=ThreeD.getAndPlotRPYt(Roll_GRF,'Roll callibrated to GRF',...
%      false,'timeseries',ResampledPlotStyle);

                 
%                  
%           [roll_Roll,pitch_Roll,yaw_Roll,t_Roll]=ThreeD.getAndPlotRPYt(Roll_GRF,'Roll callibrated to GRF final',...
%      false,'timeseries',ResampledPlotStyle);  

% 
% % calibrate of the callibrate roll angle to the global frame
%             [Roll_callibratedG] = ...
%                 ThreeD.zeroTheRun(Roll_callibratedS,100,100,'optionStaticRun',true,'tm_tStatic',...
%                 Track);
%               [roll_staticG,pitch_staticG,yaw_staticG,t_staticG]=ThreeD.getAndPlotRPYt(Roll_callibratedG,'roll callibrated  to G',...
%     false,'timeseries',ResampledPlotStyle);
          
%    % calibrate of the roll to the global frame
%             [Roll_calibratedG] = ...
%                 ThreeD.zeroTheRun(Roll,100,100,'optionStaticRun',true,'tm_tStatic',...
%                 TrackCalibrated);     
%                [roll_Roll_calibratedG,pitch_Roll_calibratedG,yaw_Roll_calibratedG,t_Roll_calibratedG]=ThreeD.getAndPlotRPYt(Roll_callibratedG,'Roll Callibrated to V',...
%     false,'timeseries',ResampledPlotStyle);           
         
            
%             % calibrate the roll sensor to the calibrated track sensor
%                  [Roll_callibratedG] = ...
%                 ThreeD.zeroTheRun(Roll,300,200,'optionStaticRun',true,'tm_tStatic',...
%                 TrackCalibrated);       
%            [roll_Roll,pitch_Roll,yaw_Roll,t_Roll]=ThreeD.getAndPlotRPYt(Roll_callibratedG,'Roll Resampled Callibrated global',...
%     false,'timeseries',ResampledPlotStyle);

% % calibrate the roll to the static run
%              [RollCalibrated] = ...
%                 ThreeD.zeroTheRun(Roll,300,100,'optionStaticRun',true,'tm_tStatic',...
%                 RollStatic);
%             [roll_Roll,pitch_Roll,yaw_Roll,t_Roll]=ThreeD.getAndPlotRPYt(RollCalibrated,'Roll Callibrated',...
%     false,'timeseries',ResampledPlotStyle); 


