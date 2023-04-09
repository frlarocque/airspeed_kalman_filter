%%
clear all
close all
clc

format long
% Add all paths
addpath('/home/frederic/Documents/thesis/tools/airspeed_estimation/functions/');

%% Load single filedata
[file,path] = uigetfile({'*.mat'},'Select a file');

load(fullfile(path,file))

%% Plot wind

figure

plot(ac_data.NPS_WIND.timestamp,ac_data.NPS_WIND.vx,'-')
hold on
plot(ac_data.NPS_WIND.timestamp,ac_data.NPS_WIND.vy,'-')
plot(ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.timestamp,ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.mu_N,'--')
plot(ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.timestamp,ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.mu_E,'--')
xlabel('Time [s]')
ylabel('Wind speed [m/s]')
legend('True N','True E','Estimation N','Estimation E')


