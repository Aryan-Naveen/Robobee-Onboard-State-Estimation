clear all; close all;

addpath('simData');

config = load('simData/config.mat');

folder = 'simData/flightData/';
outputfolder = 'simData/flightSensorData/';
DirList = dir(fullfile(folder, '*.mat'));


for i = 4:size(DirList, 1)
    disp(DirList(i));
    data = load(fullfile(folder, DirList(i).name));
    simData = runSimulation(data, config);
    save(fullfile(outputfolder, DirList(i).name), '-struct', 'simData');
end

function simData = runSimulation(data, config)

    yout = data.yout;
    scenario = uavScenario("StopTime", 20, "UpdateRate", config.sampling_f);
    
    plat = uavPlatform("UAV", scenario, 'ReferenceFrame', 'NED');
    updateMesh(plat,"quadrotor", {0.025}, [0 0 1], [0 0 0], [1 0 0 0]);
        
    color.Gray = 0.651*ones(1,3);
    addMesh(scenario,"polygon",{[-1 -1; 1 -1; 1 1; -1 1],[-0.001 0]},color.Gray)
    
    lidarModel = uavLidarPointCloudGenerator("AzimuthLimits", [-5, 5], "ElevationLimits", [-90, 90], "HasNoise", true, 'UpdateRate', config.sampling_f);
    
    accRmsNoise = 1e-6*190 * config.g;
    gyroRmsNoise = 1.9199e-04;
    aParams = accelparams("NoiseDensity", accRmsNoise);
    gParams = gyroparams("NoiseDensity", gyroRmsNoise);
    
    imuModel = uavIMU(imuSensor('accel-gyro-mag', 'SampleRate', config.sampling_f, 'Accelerometer', aParams, 'Gyroscope', gParams, 'ReferenceFrame','ENU'));
    
    imu = uavSensor("IMU", plat, imuModel, "MountingLocation", [0, 0, 0]);
    tof = uavSensor("Lidar",plat,lidarModel,"MountingLocation",[0,0, -0.01],"MountingAngles",[0 90 0]);
    
    
    
    
    vicon = ViconData(yout, config);
    
    setup(scenario);
    
    
    simData = struct;
    simData.time = zeros(0, 1);
    simData.Accelerometer = zeros(0,3);
    simData.Gyroscope = zeros(0, 3);
    simData.Magnetometer = zeros(0, 3);
    simData.Thetas = zeros(0, 3);
    simData.Omega = zeros(0, 3);
    simData.TOF = zeros(0, 1);
    simData.U = zeros(0, 4);
    simData.trueZ = zeros(0, 1);
    
    t_duration = vicon.getEnd() - vicon.getStart();
    
    for i = vicon.getStart() : vicon.getStart() + t_duration
        move(plat, vicon.getMotionData(i));
    
        isRunning = advance(scenario);
        
        u = vicon.getDesiredControlInput(i);
    
        updateSensors(scenario);
        [isUpdated_imu, t_imu, acc, gyro, mag] = read(imu);
        [isUpdated_tof, t_tof, ptCloud_lidar] = read(tof);
        pt = proximitySensor(ptCloud_lidar);
        
        simData.Accelerometer = [simData.Accelerometer; acc];
        simData.Gyroscope = [simData.Gyroscope; gyro];
        simData.Magnetometer = [simData.Magnetometer; mag];
        simData.time = [simData.time; vicon.getTime(i)];
        simData.Thetas = [simData.Thetas; vicon.getThetasAtTimet(i)];
        simData.Omega = [simData.Omega; vicon.getOmegaAtTimet(i)];
        simData.TOF = [simData.TOF; pt];
    
        simData.U = [simData.U; u];
        simData.trueZ = [simData.trueZ; vicon.getZ(i)];
    
        % Exit loop when scenario is finished.
        if ~isRunning 
            break; 
        end
    end

end








