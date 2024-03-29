classdef uavIMU < uav.SensorAdaptor
    %uavIMU Adaptor that specifies behavior of imuSensor in uavScenario
    
    %   Copyright 2020 The MathWorks, Inc.
    
    properties
        %UpdateRate Sensor Update Rate
        UpdateRate
    end
    
    methods
        function s = get.UpdateRate(obj)
            s = obj.SensorModel.SampleRate;
        end
        
        function set.UpdateRate(obj, s)
            obj.SensorModel.SampleRate = s;
        end
    end

    methods
        function obj = uavIMU(sensorModel)
            %uavIMU
            
            obj@uav.SensorAdaptor(sensorModel);
        end
        
        function setup(~, ~, ~)
            %setup the sensor
        end
        
        function [acc, gyro, mag] = read(obj, scene, platform, sensor, t)
            %read the accelerometer and gyroscope readings
            motion = uav.SensorAdaptor.getMotion(scene, platform, sensor, t);
            tformPR2R = scene.TransformTree.getTransform(obj.SensorModel.ReferenceFrame, platform.ReferenceFrame, t);
            rotmPR2R = tform2rotm(tformPR2R);        
            
            % use imu sensor to gather readings
            [acc, gyro, mag] = obj.SensorModel((rotmPR2R*motion(7:9)')', (rotmPR2R*motion(14:16)')', ...
                quaternion(rotm2quat(rotmPR2R*(quat2rotm(motion(10:13))))));
        end
        
        function out = getEmptyOutputs(~)
            %getEmptyOutputs
            out = {nan(1,3), nan(1,3), nan(1, 3)};
        end
        
        function reset(obj)
            %reset the sensor
            obj.SensorModel.reset();
        end
    end
end

