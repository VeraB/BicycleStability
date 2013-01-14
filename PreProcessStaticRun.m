tic
clear all;
close all;
format compact;
format long;
initialiseprocessing();


filename = 'D:\Sofie-HDF-Format\win32build\Visualeyze_tests.h5'


VisualEyezRun = '/AdrianSlow29';

StaticRun = '/AdrianZeroMeasurement04012013_2';

plotOrNot = 0; % true when preprocessing is plotted

[Steer,rollSteer,pitchSteer,yawSteer,t_Steer] = BicycleStability.visualeyzePreProcessing(filename,StaticRun,...
                'Steer','plotOrNot',plotOrNot);
[Roll,rollRoll,pitchRoll,yawRoll,t_Roll] = BicycleStability.visualeyzePreProcessing(filename,StaticRun,...
                'Roll','plotOrNot',plotOrNot);
[Pelvis,rollPelvis,pitchPelvis,yawPelvis,t_Pelvis] = BicycleStability.visualeyzePreProcessing(filename,StaticRun,...
                'Pelvis','plotOrNot',plotOrNot);
[UpperBody,rollUpperBody,pitchUpperBody,yawUpperBody,t_UpperBody] = BicycleStability.visualeyzePreProcessing(filename,StaticRun,...
                'UpperBody','plotOrNot',plotOrNot);
[LeftThigh,rollLeftThigh,pitchLeftThigh,yawLeftThigh,t_LeftThigh] = BicycleStability.visualeyzePreProcessing(filename,StaticRun,...
                'LeftThigh','plotOrNot',plotOrNot);
[RightThigh,rollRightThigh,pitchRightThigh,yawRightThigh,t_RightThigh] = BicycleStability.visualeyzePreProcessing(filename,StaticRun,...
                'RightThigh','plotOrNot',plotOrNot);    
