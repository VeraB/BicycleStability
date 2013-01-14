tic
clear all;
close all;
format compact;
format long;
initialiseprocessing();
filename = 'D:\Sofie-HDF-Format\win32build\Visualeyze_tests.h5'


VisualEyezRun = '/AdrianFast40';
            %plotExperimentPath, plots the path used in a visualeyze
            %experiment
            
            % Read in the 3 markers for all sensors
            [rawData] = RawMarkers.readFromFile(filename,VisualEyezRun,...
                'RollRB','RollLB','RollFT');                
    %            'Channel101','Channel102','Channel103'); 
            
            %plot the track in 3D
            figure;
            plot3(rawData(:,1)',rawData(:,2)',rawData(:,3)', '*m');
            xlabel('x coordinate');
            ylabel('y coordinate');
            zlabel('z coordinate');
            
            N = size(rawData(:,10),1);
            for i = 1:50:N
                text(rawData(i,1)',rawData(i,2)',rawData(i,3)',['',int2str(i)]);
            end
            title('Path in 3D');
