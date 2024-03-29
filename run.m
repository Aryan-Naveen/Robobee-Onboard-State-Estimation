close all; clear all;

addpath('CCF');      % include quaternion library
addpath('EKF');
addpath('CKF');

config = load('simData/config.mat');

folder = 'simData/flightSensorData/';
DirList = dir(fullfile(folder, '*.mat'));
RMSE = [];


alpha = [0.5987 0.5659 0.3524];
KP = [4.1395 28.0558 996.3669];
KI = [15.1802 10.3947 273.3739];

Q = [0.0022 2.6866 16.3006 1.3115 1.3307 1.3321 1.3309 0.0640 0.0002 1.3309];
R = [3.1346 0.5184 0.0098 1.3309];

CCFparams = [alpha KP KI];
EKFparams = [Q R];


%%
for i = 1:size(DirList, 1)
    disp(i);
    data = load(fullfile(folder, DirList(i).name));
    ccf = Robobee_CCF(CCFparams, config.sampling_time);
    ekf = RobobeeEKF(EKFparams, config);
    ckf = Robobee_CKF(ccf, ekf);
    traj = getEstimatedTrajectory(ckf, data, config);
    RMSE = [RMSE;getRMSE([data.Thetas data.trueZ-0.015], traj)];
end
mean(RMSE)
%%
%data = load(fullfile(folder, DirList(55).name));
data = load('simData/flightSensorData/OL_PBee_20230927_12.mat');
ccf = Robobee_CCF(CCFparams, config.sampling_time);
ekf = RobobeeEKF(EKFparams, config);
ckf = Robobee_CKF(ccf, ekf);
traj = getEstimatedTrajectory(ckf, data, config);
getRMSE([data.Thetas data.trueZ-0.015], traj)


start = 1;
n_time = size(data.time, 1)
figure('Name', 'Complimentary Extended Kalman Filter');
axis(1) = subplot(3,1,1);
hold on;
plot(data.time(start: n_time), traj(start: n_time, 1), 'r');
plot(data.time(start: n_time), data.Thetas(start: n_time, 1), 'r', 'LineStyle','--');
legend('EKF \phi', 'Vicon \phi', 'Error');
xlabel('Time (s)');
ylabel('Radians');
title('Roll');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(data.time(start: n_time), traj(start: n_time, 2), 'g');
plot(data.time(start: n_time), data.Thetas(start: n_time, 2), 'g', 'LineStyle','--');
legend('EKF \theta', 'Vicon \theta', 'Error');
xlabel('Time (s)');
ylabel('Radians');
title('Pitch');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(data.time(start: n_time), traj(start: n_time, 3), 'b');
plot(data.time(start: n_time), data.Thetas(start : n_time, 3), 'b', 'LineStyle','--');
legend('EKF \psi', 'Vicon \psi', 'Error');
xlabel('Time (s)');
ylabel('Radians');
title('Yaw');
hold off;

linkaxes(axis, 'x');

%%
figure('Name', 'Complimentary Extended Kalman Filter Altitude Estimation');
axis(1) = subplot(1,1,1);
hold on;
plot(data.time(start: n_time), traj(start: n_time, 4), 'b');
plot(data.time(start: n_time), data.trueZ(start: n_time)-0.015, 'b', 'LineStyle','--');
legend('CEKF z', 'Vicon z');
xlabel('Time (s)');
ylabel('Meters');
ylim(axis(1), [0 0.2]);
hold off;



function RMSE = getRMSE(true, traj)
    error = wrapToPi(wrapToPi(true) - wrapToPi(traj));
    RMSE = sqrt(mean(error.^2));
end

function traj = getEstimatedTrajectory(filter, data, config)

    traj = [];
    for t = 1:size(data.time)
        filter.update(data.Accelerometer(t, :), data.Gyroscope(t, :), data.Magnetometer(t, :), data.TOF(t), data.U(t, :));
        traj = [traj; filter.X.'];
    end

end
