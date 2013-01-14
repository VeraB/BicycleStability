% Loads a Visualeyze run, checks for gaps and does the processing

tic
clear all;
close all;
format compact;
format long;
initialiseprocessing();
filename = 'D:\Sofie-HDF-Format\win32build\Visualeyze_tests.h5'


VisualEyezRun = '/AdrianSlow30';

speed = 14 /3.6 *1000;
SwitchTime = 160e-6;
SwitchTimeWorst = 1/100;
inaccuracyOfSwitchting = speed * SwitchTime;
inaccuracyOfSwitchtingWorstCase = speed * SwitchTimeWorst;
%return
% Read in the 3 markers for all sensors
[rawSteer] = RawMarkers.readFromFile(filename,VisualEyezRun,...
    'RollRB','RollLB','RollFT');

% Check if there are NAN's, zero's or outliers in the data and plot them
[steerAll,steerSome,steerOutliers] = ...
    RawMarkers.areMarkersDropped(rawSteer,'yesOrNoPlot',true,...
    'sensorName','Steer','maxVelJump', 1000, 'Fs', 100);

% Remove the NaN's, zero's and outliers from the data
[gapSteer] = RawMarkers.findGaps(rawSteer);

% Check if the markers are well spaced

[steerWellSpaced,distances]=...
    RawMarkers.areTheMarkersWellSpaced(gapSteer,'lengthThreshold',14.0,...
    'plotTheDistances',true,'plotBoxPlot',true,'sensorName', 'Steer - rawData',...
    'withRespectToMean',true);

%   Plot path in 3D
BicycleStability.plotExperimentPath(filename,VisualEyezRun, 'sensorName', 'Steer')
% Remove if not well spaced
[spacedSteer] = RawMarkers.removeNotWellSpaced(gapSteer,...
    steerWellSpaced);

% interpolate the gaps
[interpSteer] = RawMarkers.interpolateGaps(spacedSteer,rawSteer,'plotOrNot',true,...
    'sensorName','Steer');

% % Check if the markers are well spaced, after interpolation
% [yesOrNoIntp,distancesIntp]=...
%     RawMarkers.areTheMarkersWellSpaced(interpSteer,'lengthThreshold',14.0,...
%     'plotTheDistances',true,'plotBoxPlot',true,'sensorName', 'Steer - interpData',...
%     'withRespectToMean',true);
% % Remove if not well spaced
% [InterpspacedSteer] = RawMarkers.removeNotWellSpaced(interpSteer,...
%     yesOrNoIntp);

% filtering the interpolated data
[filtSteer] = RawMarkers.filterRawData(interpSteer,'plotOrNot',true,'sensorName','Steer');

% Check if the markers are well spaced, after filtering
[yesOrNoFilt,distancesFilt]=...
    RawMarkers.areTheMarkersWellSpaced(filtSteer,'lengthThreshold',14.0,...
    'plotTheDistances',true,'plotBoxPlot',true,'sensorName', 'Steer - filtData',...
    'withRespectToMean',true);
% Remove if not well spaced
[filtspacedSteer] = RawMarkers.removeNotWellSpaced(filtSteer,...
    yesOrNoFilt);


%Erase NaN, to be able to create the 3D object
[filtSteerN,tSteerOriginal] = RawMarkers.eraseNan(filtspacedSteer);

% create 3D object
Steer = Markers3D.create3DMarkersFromRawData(filtSteerN);



% calculate and plot Roll Pitch Yaw
[rollSteer,pitchSteer,yawSteer,tSteer] = ...
    ThreeD.getAndPlotRPYt(Steer,'Steer',false,'timeseries','*b',...
    'plotDropped', true, 'yesOrNoAll', steerAll, 'yesOrNoSome', steerSome,...
     'yesOrNoOutlier', steerOutliers,'yesOrNoWellSpaced',yesOrNoFilt,...
     't_Original', tSteerOriginal);
% 
% % remove not well spaced
% [Steer_Dropped,gapArray] = ThreeD.dropQuaternions(Steer, 'yesOrNoAll', steerAll, 'yesOrNoSome', steerSome,...
%     'yesOrNoOutlier', steerOutliers,'yesOrNoWellSpaced',steerWellSpaced);
% 
% % calculate and plot Roll Pitch Yaw - after dropped
% % quaternions
% [roll,pitch,yaw,t] = ...
%     ThreeD.getAndPlotRPYt(Steer_Dropped,'Steer',false,'timeseries','*b',...
%     'plotDropped', true, 'yesOrNoAll', steerAll, 'yesOrNoSome', steerSome,...
%     'yesOrNoOutlier', steerOutliers,'yesOrNoWellSpaced',steerWellSpaced);

