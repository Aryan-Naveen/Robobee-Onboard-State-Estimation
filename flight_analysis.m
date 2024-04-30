clear all; close all;

data = load('simData/data/20231224_BBee_compliant_leg_119mg_300Vpp_155Hz_leaf_hop_untethered_2.mat');


yout = data.yout;

positions = yout(:,44:46);
rpy = yout(:, 9:11);

time = yout(:,1); 
poses = [positions rpy];

writematrix([time poses], 'leaf_landing.csv');
