classdef Experiment < handle
    %EXPERIMENT A class to handle the processing and configuration of
    %experiment runs
    properties
        %Configuration
        cacheTheReadData=true
        Fs_IMU=200;
        callibrateStart=200;
        plotTheSteeringAndRollAngleRuns=false;
        filename='';
        synchroniseIMUsOnRPY=false;
        processIMUorAdams=true;
        runName='';
        steeringImuNodeId=1;
        rollingImuNodeId=2;
        %RawData
        ant_t = [];
        figures = [];
        ant_sync = [];
        ant_sync_t = [];
        steeringColumnRun = [];
        steeringColumnSyncs = [];
        frameRun= [];
        frameRunSyncs = [];
        %Processed Data.
        bicycleSteeringAngle=[];
        bicycleRollAngle = [];
        bicycleSpeed = [];
        bicycleCadence = [];
        
        
    end
    methods(Static)
        function [runs] = listRuns(filename)
            runs = h5Reader.listRuns(filename);
        end
    end
    methods
        function ex = Experiment(filename)
            % The Constructor
            ex.filename =filename;
        end
        
        function ex = processRun(ex,runName)
            %Process a single RUN.
            ex.runName=runName;
            ex = ex.getVelocityRollAndSteeringAngles();
        end
        
        function [ex] = getVelocityRollAndSteeringAngles(ex)
            %getVelocityRollAndSteeringAngles 
            %Processes a run in an experiment and calculates the
            %steering and roll angles with the plots as well.
            % RETURNS:
            % bicycleSteeringAngle - The Steering angle for the run.
            % bicycleRollAngle - The roll angle for the run.
            % bicycleSpeed - The forward velocity for the run.
            % cadence - The pedaling cadence for the run.
            % ant_t - The time vector for the speed and cadence measurements.
            % figures - A list of the figures handles, in case you want to export
            % by using laprint or something.
            finalPlotStyle = '--r.';
            display(['%%%%%%%%%%%%%%Processing RUN:' ex.runName ' %%%%%%%%%%%%%%'])
            display('CREATING FIGURES')
            if isempty(ex.figures)
                figFinalAngles=figure('visible','on','WindowStyle','docked',...
                    'Name','Roll, Pitch and Steering Angle of the Bicycle.');
                figSync = figure('visible','on','WindowStyle','docked',...
                    'Name','Synchronisation');
                figSpeed = figure('visible','on','WindowStyle','docked',...
                    'Name','Forward velocity.');
                ex.figures = [figFinalAngles,figSync,figSpeed];
            else
                delete(ex.figures(1));
                figFinalAngles=ex.figures(1);
                delete(ex.figures(2));
                figSync = ex.figures(2);
                delete(ex.figures(5));
                figRollSteerSync = ex.figures(5);
                delete(ex.figures(6));
                figSpeed = ex.figures(6);
            end
            display('FIGURES CREATED')
            
            runNamePrint = strrep(ex.runName,'/','-');
            runNamePrint = strrep(runNamePrint,'_','-');
            filenamePrint = strrep(ex.filename,'/','-');
            filenamePrint = strrep(filenamePrint,'_','-');
            loadedDataFilename=['./dataloaded' runNamePrint filenamePrint...
                '-getbicycleangles.mat'];
            %Get the data
            display(['GETTING DATA FROM FILE:' loadedDataFilename]);
            %Only load from file if caching is enabled.
            if ((ex.cacheTheReadData)&&(exist(loadedDataFilename,'file')))
                load(loadedDataFilename);
            else
                if ex.processIMUorAdams==true
                    %Read promove with no resampling
                    [ex.steeringColumnRun,ex.steeringColumnSyncs] = ...
                        Quat3D.readDataPromove(ex.filename,ex.runName,...
                        ex.steeringImuNodeId,ex.Fs_IMU,ex.Fs_IMU);
                    [ex.frameRun,ex.frameRunSyncs] = ...
                        Quat3D.readDataPromove(ex.filename,ex.runName,...
                        ex.rollingImuNodeId,ex.Fs_IMU,ex.Fs_IMU);
                    theAntReader = antReader(ex.filename,ex.runName);
                    [ex.bicycleSpeed,ex.bicycleCadence,ex.ant_t,...
                        ex.ant_sync,ex.ant_sync_t] = ...
                        theAntReader.readVelocityCadence();
                else
                    [ex.steeringColumnRun,ex.steeringColumnSyncs] = ...
                        Markers3D.readDataAdams(ex.filename,ex.runName,...
                        'RBO','LBO','FON');
                    [ex.frameRun,ex.frameRunSyncs] = ...
                        Markers3D.readDataAdams(ex.filename,ex.runName,...
                        'RBT','LBT','FTN');
                end
                
                save(loadedDataFilename,'ex');
            end
            display('DATA RECEIVED');
            display('PLOTTING FORWARD INFORMATION');
            if ex.processIMUorAdams==true
                %Synchronise:
                if any(ex.ant_sync_t)
                    ex.bicycleSpeed =...
                        ex.bicycleSpeed(ex.ant_t>=ex.ant_sync_t(1));
                    ex.bicycleCadence =...
                        ex.bicycleCadence(ex.ant_t>=ex.ant_sync_t(1));
                    ex.ant_t =  ex.ant_t(ex.ant_t>=ex.ant_sync_t(1));
                end
                ex.ant_t =ex.ant_t-repmat(ex.ant_t(1),size(ex.ant_t));
                set(0,'CurrentFigure',figSpeed)
                subplot(2,1,1)
                plot(ex.ant_t,ex.bicycleSpeed);
                title('Forward Velocity: ');
                subplot(2,1,2)
                plot(ex.ant_t,ex.bicycleCadence);
                title('Pedaling Cadence: ');
                display('FORWARD VELOCITY PLOTTED');
                if any(ex.frameRunSyncs)
                    display('SYNCING SENSORS ON MANUAL VALUES');
                    if any(ex.steeringColumnSyncs)
                        [ex.frameRun,ex.steeringColumnRun] = synchronise(ex.frameRunSyncs,...
                            ex.steeringColumnSyncs,ex.frameRun,ex.steeringColumnRun,...
                            Fs_plot,1,1);
                    end
                    display('FINISHED MANUAL SYNC');
                end
                if ex.synchroniseIMUsOnRPY
                    display('SYNCING SENSORS');
                    set(0,'CurrentFigure',figSync);
                    hold on;
                    display('PLOTTING ROLL PITCH YAW: SENSORS SYNCHRONISED');
                    [frameSensorRollAngle,...
                        frameSensorPitchAngle,...
                        frameSensorYawAngle]=ThreeD.getRPYt(ex.frameRun,true);
                    [steerColumnRollAngle,...
                        steerColumnPitchAngle,...
                        steerColumnYawAngle]=ThreeD.getRPYt(ex.steeringColumnRun,true);
                    [ex.frameRun,ex.steeringColumnRun]=...
                        synchroniseWithRespectToRPY(...
                            frameSensorRollAngle,...
                        frameSensorPitchAngle,...
                        frameSensorYawAngle,...
                        ex.frameRun,...
                        steerColumnRollAngle,...
                        steerColumnPitchAngle,...
                        steerColumnYawAngle,...
                        ex.steeringColumnRun);
                    display('FINISHED PLOTTING ROLL PITCH YAW: SENSORS SYNC');
                end
            end
            
            [timestamps,steeringJoint_t,...
                steeringJointRollAngle,...
                steeringJointPitchAngle,...
                steeringJointYawAngle,...
                frameSensorRollAngle,...
                frameSensorPitchAngle,...
                frameSensorYawAngle,...
                figuresJoint] = ...
                getjointangles(ex.frameRun,ex.steeringColumnRun,...
                'Steering Angle',ex.callibrateStart);
            
            ex.figures = [ex.figures figuresJoint];
            ex.bicycleSteeringAngle = steeringJointYawAngle;
            ex.bicycleRollAngle = frameSensorRollAngle;
            set(0,'CurrentFigure',figFinalAngles)
            ThreeD.plotRPY(ex.bicycleRollAngle,...
                frameSensorPitchAngle,...
            ex.bicycleSteeringAngle,...
                timestamps,true,'timeseries',finalPlotStyle);
            
            if ex.plotTheSteeringAndRollAngleRuns==1
                display('PLOT STEERING QUAT');
                minSize = min(size(ex.steeringColumnRun,2),size(ex.frameRun,2));
                figure
                ThreeD.plotRun([ex.frameRun(1:minSize);ex.steeringColumnRun(1:minSize);
                    diff_t(1:minSize)
                    ]);
                display('STEERING QUAT PLOTTED');
            end
        end
        
    end
    
end

