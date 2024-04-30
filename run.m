close all; clear all;

addpath('CCF');      % include quaternion library
addpath('EKF');
addpath('CKF');
addpath('utils');

config = load('simData/config.mat');

folder = 'simData/flightSensorData/';
DirList = dir(fullfile(folder, '*.mat'));
RMSE = [];

%alpha = [0.5342 0.5525 0.2000];
%KP = [4.9076 27.4835 76.2950];
%KI = [12.0692 2.2215 94.4390];
alpha = [0.5987 0.5659 0.3524];
KP = [4.1395 28.0558 996.3669];
KI = [15.1802 10.3947 273.3739];

Q = [0.0022 2.6866 16.3006 1.3115 1.3307 1.3321 1.3309 0.0640 0.0002 1.3309];
R = [3.1346 0.5184 0.0098 1.3309];%
%Q = [0.0000 0.0009 110.9023 0.0000 0.0001 4.3754 0.3037 0.0000 0.0000 0.3037];
%R = [3.8426 16.6584 0.0387 0.3037];
CCFparams = [alpha KP KI];
EKFparams = [Q R];


%%
for i = 1:size(DirList, 1)
    disp(i);
    data = load(fullfile(folder, DirList(i).name));
    ccf = Robobee_CCF(CCFparams, config.sampling_time);
    ekf = RobobeeEKF(EKFparams, config);
    ckf = Robobee_CKF(ccf, ekf);
    [traj, conf] = getEstimatedTrajectory(ckf, data, config);
    [trueTraj, t] = getTrueTraj(data, 1e-4, 1e-3);
    RMSE = [RMSE; getRMSE(trueTraj, traj)];
end
mean(RMSE)
%%
%data = load(fullfile(folder, DirList(16).name));
data = load('simData/data_/20231224_BBee_compliant_leg_119mg_300Vpp_155Hz_leaf_hop_untethered_2.mat');

[trueTraj, t] = getTrueTraj(data, 1e-4, 1e-3);

ccf = Robobee_CCF(CCFparams, 1e-3);
ekf = RobobeeEKF(EKFparams, config);
ckf = Robobee_CKF(ccf, ekf);
[traj, conf] = getEstimatedTrajectory(ckf, data, config);
getRMSE(trueTraj, traj)

alpha = 0.5;  
figure('Name', 'Complimentary Extended Kalman Filter');


axis(1) = subplot(4,1,1);
hold on;
shadedErrorBar(t, traj(:, 1), conf(:, 1));
plot(t, traj(:, 1), 'r');
plot(t, trueTraj(:, 1), 'r', 'LineStyle','--');
legend('CKF \phi', 'Vicon \phi', 'Error');
xlabel('Time (s)');
ylabel('Radians');
title('Roll');
hold off;

axis(2) = subplot(4,1,2);
hold on;
shadedErrorBar(t, traj(:, 2), conf(:, 2));
plot(t, traj(:, 2), 'g');
plot(t, trueTraj(:, 2), 'g', 'LineStyle','--');
legend('CKF \theta', 'Vicon \theta', 'Error');
xlabel('Time (s)');
ylabel('Radians');
title('Pitch');
hold off;


axis(3) = subplot(4,1,3);
hold on;
shadedErrorBar(t, traj(:, 3), conf(:, 3));
plot(t, traj(:, 3), 'b');
plot(t, trueTraj(:, 3), 'b', 'LineStyle','--');
legend('CKF \psi', 'Vicon \psi', 'Error');
xlabel('Time (s)');
ylabel('Radians');
title('Yaw');
hold off;


axis(4) = subplot(4,1,4);
hold on;
shadedErrorBar(t, traj(:, 4), conf(:, 4));
plot(t, traj(start: n_time, 4), 'b');
plot(t, trueTraj(:, 4), 'b', 'LineStyle','--');
legend('CKF z', 'Vicon z', 'Error');
xlabel('Time (s)');
ylabel('Meters');
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

function [traj, confidence] = getEstimatedTrajectory(filter, data)

    traj = [];
    confidence = [];
    for t = 1:10:size(data.time)
        filter.update(data.Accelerometer(t, :), data.Gyroscope(t, :), data.Magnetometer(t, :), data.TOF(t), data.U(t, :));
        traj = [traj; filter.X.'];
        confidence = [confidence; filter.P];
    end

end


function [trueTraj, t] = getTrueTraj(data, original_dt, new_dt)
    s = new_dt/original_dt;
    trueTraj = [data.Thetas(1:s:end, :) data.trueZ(1:s:end, :) - 0.015];
    t = data.time(1:s:end);
end
