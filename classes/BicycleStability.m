classdef BicycleStability < ThreeD
    %
    properties% (Access = protected)
        f_length;
    end
    
    methods (Static)
        
        
        
        
        
        
        %%
        function [] = plotExperimentPath(filename,VisualEyezRun, varargin)
            %plotExperimentPath, plots the path used in a visualeyze
            %experiment
            p = inputParser;
            addOptional(p,'sensorName','sensorName');
            addOptional(p,'YesOrNoTrack',false);
            parse(p,varargin{:});
            sensorName = p.Results.sensorName;
            YesOrNoTrack = p.Results.YesOrNoTrack;
            
            % Read in the 3 markers for all sensors
            
            if YesOrNoTrack
                [rawData] = RawMarkers.readFromFile(filename,VisualEyezRun,...
                    'Channel103','Channel104','Channel101');
            else
                [rawData] = RawMarkers.readFromFile(filename,VisualEyezRun,...
                    [sensorName 'RB'],[sensorName 'LB'],[sensorName 'FT']);
            end
            %plot the track in 3D
            figure('visible','on','WindowStyle','docked',...
                'Name',[sensorName ' - Path in 3D']);
            plot3(rawData(:,1)',rawData(:,2)',rawData(:,3)', '*g');
            grid on
            
            xlabel('x coordinate');
            ylabel('y coordinate');
            zlabel('z coordinate');
            
            N = size(rawData(:,10),1);
            for i = 1:50:N
                text(rawData(i,1)',rawData(i,2)',rawData(i,3)',['',int2str(i)]);
            end
            hold on
            plot3(rawData(:,4)',rawData(:,5)',rawData(:,6)', '*m');
            xlabel('x coordinate');
            ylabel('y coordinate');
            zlabel('z coordinate');
            
            N = size(rawData(:,10),1);
            for i = 1:50:N
                text(rawData(i,4)',rawData(i,5)',rawData(i,6)',['',int2str(i)]);
            end
            plot3(rawData(:,7)',rawData(:,8)',rawData(:,9)', '*b');
            xlabel('x coordinate');
            ylabel('y coordinate');
            zlabel('z coordinate');
            
            N = size(rawData(:,10),1);
            for i = 1:50:N
                text(rawData(i,7)',rawData(i,8)',rawData(i,9)',['',int2str(i)]);
            end
            legend('RB','LB','FO');
            title([sensorName 'Path in 3D']);
        end
        
        
        %%
        function [rawData] = visualeyzePreProcessing(filename,VisualEyezRun,...
                sensorName,rightBackName,leftBackName,frontName,varargin)
            % Loads a Visualeyze run, checks for gaps and does the
            % pre-processing (interpolation and filtering), creates a 3D object
            % and gets and plots the roll pitch yaw
            p = inputParser;
            
            addOptional(p,'lengthThreshold',14.0);
            addOptional(p,'maxGap',10); %max gap to interpolate
            addOptional(p,'interpMethod', 'linear');
            addOptional(p,'maxVelJump',1000); %max velocity jump in mm/sec
            addOptional(p,'Fs',100);
            addOptional(p,'freqLowPass',15);
            addOptional(p,'orderLowPass',4);
            addOptional(p,'plotOrNot',true);
            parse(p,varargin{:});
            
            lengthThreshold = p.Results.lengthThreshold
            maxGap = p.Results.maxGap
            interpMethod = p.Results.interpMethod
            maxVelJump = p.Results.maxVelJump
            Fs = p.Results.Fs
            freqLowPass = p.Results.freqLowPass
            orderLowPass = p.Results.orderLowPass;
            plotOrNot = p.Results.plotOrNot
            
            % Read in the 3 markers for all sensors
            [rawData] = RawMarkers.readFromFile(filename,VisualEyezRun,...
                rightBackName,leftBackName,frontName);
            
            % Check if there are NAN's, zero's or outliers in the data and plot them
            [dataAll,dataSome,dataOutliers] = ...
                RawMarkers.areMarkersDropped(rawData,'yesOrNoPlot',plotOrNot,...
                'sensorName',sensorName,'maxVelJump', maxVelJump, 'Fs', Fs);
            
            % Remove the NaN's, zero's and outliers from the data
            [gapData] = RawMarkers.findGaps(rawData);
            
            % Check if the markers are well spaced
            
            [dataWellSpaced,distances]=...
                RawMarkers.areTheMarkersWellSpaced(gapData,'lengthThreshold',lengthThreshold,...
                'plotTheDistances',plotOrNot,'plotBoxPlot',plotOrNot,'sensorName', [sensorName ' - rawData'],...
                'withRespectToMean',true);
            
            %   Plot path in 3D
            if plotOrNot
                BicycleStability.plotExperimentPath(filename,VisualEyezRun, 'sensorName', sensorName) ;
            end
            
            % Remove if not well spaced
            [spacedData] = RawMarkers.removeNotWellSpaced(gapData,...
                dataWellSpaced);
            
            % interpolate the gaps
            [interpData] = RawMarkers.interpolateGaps(spacedData,rawData,'plotOrNot',plotOrNot,...
                'sensorName',sensorName);
            
            % filtering the interpolated data
            [filtData] = RawMarkers.filterRawData(interpData,'plotOrNot',plotOrNot,'sensorName',sensorName);
            
            % Check if the markers are well spaced, after filtering
            [yesOrNoFilt]=...
                RawMarkers.areTheMarkersWellSpaced(filtData,'lengthThreshold',lengthThreshold,...
                'plotTheDistances',false,'plotBoxPlot',false,'sensorName', [sensorName ' - filtData'],...
                'withRespectToMean',true);
            % Remove if not well spaced
            [filtSpacedData] = RawMarkers.removeNotWellSpaced(filtData,...
                yesOrNoFilt);
            
            %Erase NaN, to be able to create the 3D object
            [filtDataN, t_Original] = RawMarkers.eraseNan(filtSpacedData);
            
            % create 3D object
            rawData = filtDataN;
%             tm_t = Markers3D.create3DMarkersFromRawData(filtDataN);
%             
%             % calculate and plot Roll Pitch Yaw
%             [roll,pitch,yaw,t] = ...
%                 ThreeD.getAndPlotRPYt(tm_t,sensorName,false,'timeseries','*b',...
%                 'plotDropped', true, 'yesOrNoAll', dataAll, 'yesOrNoSome', dataSome,...
%                 'yesOrNoOutlier', dataOutliers,'yesOrNoWellSpaced',dataWellSpaced,...
%                 't_Original',t_Original);
            
        end
        %%
        function [SM_t] = calculateStabilityMargin_t(btm_t,COM_0)
            SM_t = zeros(1,length(btm_t));
            parfor i = 1: length(btm_t)
                tm = btm_t{i};
                H_f_to_h = tm.getH_f_to_h();
                H_f_0 = btm_t{i}.getH;
                COM_h = (H_f_to_h \(H_f_0)) *  COM_0;
                SM_t(i) = COM_h(2);
            end
        end
        
        %%
    end
    
    methods
        function btm = BicycleStability(tm,f_length)
            % The Constructor
            %
            quat = tm.getQ;
            btm@ThreeD(quat);
            btm.f_length = f_length;
        end
        %%
        function lengthAdams = getlength(btm)
            % this function can be used to calculate the length of the rear
            % frame sensor to the ground, when processing Adams data.
            % length = f_z0/sin(rollangle)
            currentRPY = btm.getRPY(false);
            roll_angle = currentRPY(1);
            %TO be done: get the z-coordinate of the origin of the rear
            %frame sensor from the Adams input data...
        end
        
        %%
        function [H_f_to_h] = getH_f_to_h (btm)
            % Calculate the H matrix from the rear frame sensor to the heading frame (projected coordinate system of the rear frame on the ground)
            % input are the length (origin of rear frame coordinate system to the ground) and the roll angle
            currentRPY = btm.getRPY(false);
            roll_angle = currentRPY(1);
            %             if (sin(roll_angle) == 0)
            %                 if roll_angle == pi
            %                     translation_f_to_h = [0,0,-btm.l];
            %                 else
            %                     translation_f_to_h = [0,0,btm.l];
            %                 end
            %             elseif cos(roll_angle) == 0
            %                 if roll_angle == pi/2
            %                    translation_f_to_h = [0,btm.l,0];
            %                 else
            %                    translation_f_to_h = [0,-btm.l,0];
            %                 end
            %             else
            
            translation_f_to_h = [0, btm.f_length*sin(roll_angle), btm.f_length*cos(roll_angle)];
            H_f_to_h = rotx(roll_angle);
            H_f_to_h(1:3,4)=translation_f_to_h';
        end
    end
    
end



