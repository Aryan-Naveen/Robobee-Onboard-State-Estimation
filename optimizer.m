clear all; close all;
addpath('CCF');      % include quaternion library
addpath('EKF');
addpath('CKF');

config = load('config.mat');


n_training_files = 40;
training_datasets = generateDataset(n_training_files);

% gs = GlobalSearch('Display','iter', 'PlotFcn','gsplotbestf','StartPointsToRun','bounds-ineqs');

num_minutes = 180;
ms = MultiStart('Display','iter','PlotFcn','gsplotbestf', 'StartPointsToRun','bounds', 'MaxTime',60*num_minutes);

alpha = optimvar('alpha', 3, 'LowerBound',  [0 0 0], 'UpperBound', [1 1 1]);
KP = optimvar('KP', 3, 'LowerBound',  [0 0 0], 'UpperBound', [1e3 1e3 1e3]);
KI = optimvar('KI', 3, 'LowerBound',  [0 0 0], 'UpperBound', [1e3 1e3 1e3]);

x0_ccf.alpha = [0.5 0.5 0.5];
x0_ccf.KP = ones(1, 3);
x0_ccf.KI = ones(1, 3);

objCCF = fcn2optimexpr(@calculateCostCCF, [alpha KP KI], training_datasets, config); 
probCCF = optimproblem("Objective",objCCF);

[solCCF, fvalCCF] = solve(probCCF, x0_ccf, ms);
disp('The optimal found alpha, KP, and KI for CCF:')
disp(solCCF.alpha);
disp(solCCF.KP);
disp(solCCF.KI);
disp('The minimal cost function is:')
disp(fvalCCF)



Q = optimvar('Q', 10, 'LowerBound',  [0 0 0 0 0 0 0 0 0 0], 'UpperBound', 1e3*ones(1, 10));
R = optimvar('R', 4, 'LowerBound',  [0 0 0 0], 'UpperBound', [1e3 1e3 1e3 1e3]);

x0_ckf.Q = ones(1, 10);
x0_ckf.R = ones(1, 4);

CCFparams = [solCCF.alpha; solCCF.KP; solCCF.KI].';

objCKF = fcn2optimexpr(@calculateCostCKF, [Q; R], CCFparams, training_datasets, config); 
probCKF = optimproblem("Objective",objCKF);

[solCKF, fvalCKF] = solve(probCKF, x0_ckf, ms);
disp('The optimal found Q and R for CKF:')
disp(solCKF.Q)
disp(solCKF.R)
disp('The minimal cost function is:')
disp(fvalCKF)



function cost = calculateCostCKF(ekfParams, CCF_optimal, dataset, config)
    cost = 0;
    for data = dataset
        ccf = Robobee_CCF(CCF_optimal, config.sampling_time);
        ekf = RobobeeEKF(ekfParams, config);
        ckf = Robobee_CKF(ccf, ekf);
        traj = getEstimatedTrajectory(ckf, data, config);
        error = wrapToPi(wrapToPi(data.Thetas) - wrapToPi(traj(:, 1:3)));
        cost = cost + sum((error).^2, "all");
    end
    cost = cost + norm(ekfParams(:));
end


function cost = calculateCostCCF(params, dataset, config)
    cost = 0;
    for data = dataset
        ccf = Robobee_CCF(params, config.sampling_time);
        traj = getEstimatedTrajectory(ccf, data, config);
        error = wrapToPi(wrapToPi(data.Thetas) - wrapToPi(traj(:, 1:3)));
        cost = cost + sum((error).^2, "all");
    end
    cost = cost + norm(params(:));
end

function traj = getEstimatedTrajectory(filter, data, config)
    traj = [];
    for t = 1:size(data.time) 
        filter.update(data.Accelerometer(t, :), data.Gyroscope(t, :), data.Magnetometer(t, :), data.TOF(t, :), data.U(t, :));    
        traj = [traj; filter.X.'];
    end
end

function dataset = generateDataset(n_files)
    folder = 'simsensor/';
    DirList = dir(fullfile(folder, '*.mat'));

    files_indices = randperm(numel(DirList), n_files);
    dataset = [];
    disp("Randomly selected files to fit parameters to: ");
    for ind = files_indices
        disp(DirList(ind).name);
        data = load(fullfile(folder, DirList(ind).name));
        dataset = [dataset, data];
    end
end
