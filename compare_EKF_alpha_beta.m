%%
clear all
close all
clc

% Add all paths
addpath('functions/');

%% Load mat file
log_path = '/home/frederic/Documents/thesis/tools/kalman_airspeed_estimation/mat_files';

file_list = dir(fullfile(log_path, 'OUTDOOR*.mat'));

%[file,path] = uigetfile({'*.mat'},'Select a file');
for i=1:1%length(file_list)
clearvars -except file_list i
file = file_list(i).name;
path = file_list(i).folder;

load(fullfile(path,file))

%% Setup
% Setup Options
graph = 0;
beta_est = 0;
recalculate_variance = false;

% Run setup
wind_triangle_setup

%% Common EKF parameters
% 𝑢, 𝑣, 𝑤 = Velocity in the Body Frame
% 𝜇𝑥, 𝜇𝑦, 𝜇𝑧 = Wind Velocity in Earth Fixed Frame
% 𝑝, 𝑞, 𝑟 = Angular Rates Measured by Gyroscopes
% 𝑎x, 𝑎𝑦, 𝑎𝑧 = Accelerations Measured by Accelerometers
% 𝑉𝑥 , 𝑉𝑦, 𝑉𝑧 = Velocity in Earth Fixed Frame
% alpha, beta = angle of attack and sideslip angle
%x = [u v w mu_x mu_y mu_z];
%u = [a_x a_y a_z p q r phi theta psi];

%w_w = [0;0;0]; %noise angular rate
%w_a = [0;0;0]; %noise accelerations
%w_mu =[0;0;0]; %noise wind

epsi = 1E-2;
wind_var = [1 1 1]*7.8E-7;
t = airspeed_pitot.flight.time;
dt = mean(t(2:end)-t(1:end-1));
x_0 = [0 0 0 0 0 0]';
u_list = [IMU_accel.flight.data IMU_rate.flight.data IMU_angle.flight.data]';
Q = diag([IMU_accel.var,wind_var]); %process noise
P_0 = diag([1 1 1 wind_var]); %covariance

%% EKF with alpha and beta
%z = [V_x V_y V_z alpha beta];

f_fh = str2func('f_2');
g_fh = str2func('g_2');

z_list = [Vg_NED.flight.data alpha.flight.data beta.flight.data]'; %measurement
R = diag([Vg_NED.var IMU_angle.var(2) 1E-6]); %measurement noise

kalman_res_alpha_beta = run_EKF(epsi,t,Q,R,P_0,x_0,u_list,z_list,f_fh,g_fh);
kalman_res_alpha_beta.error = error_quantification(kalman_res_alpha_beta.x(1,:)',airspeed_pitot.flight.data);
kalman_res_alpha_beta.wind.vect = mean(kalman_res_alpha_beta.x([4:6],:)');
kalman_res_alpha_beta.wind.direction = atan2(kalman_res_alpha_beta.wind.vect(2),kalman_res_alpha_beta.wind.vect(1));

plot_EKF_result(kalman_res_alpha_beta,airspeed_estimation,airspeed_pitot,wind)

%% EKF without beta
%z = [V_x V_y V_z alpha];

f_fh = str2func('f_2');
g_fh = str2func('g_3');

z_list = [Vg_NED.flight.data alpha.flight.data ]'; %measurement
R = diag([Vg_NED.var IMU_angle.var(2)]); %measurement noise

kalman_res_alpha = run_EKF(epsi,t,Q,R,P_0,x_0,u_list,z_list,f_fh,g_fh);
kalman_res_alpha.error = error_quantification(kalman_res_alpha.x(1,:)',airspeed_pitot.flight.data);
kalman_res_alpha.wind.vect = mean(kalman_res_alpha.x([4:6],:)');
kalman_res_alpha.wind.direction = atan2(kalman_res_alpha.wind.vect(2),kalman_res_alpha.wind.vect(1));

plot_EKF_result(kalman_res_alpha,airspeed_estimation,airspeed_pitot,wind)
%% EKF without alpha nor beta
%z = [V_x V_y V_z];

f_fh = str2func('f_2');
g_fh = str2func('g_4');

z_list = [Vg_NED.flight.data]'; %measurement
R = diag([Vg_NED.var]); %measurement noise

kalman_res = run_EKF(epsi,t,Q,R,P_0,x_0,u_list,z_list,f_fh,g_fh);
kalman_res.error = error_quantification(kalman_res.x(1,:)',airspeed_pitot.flight.data);
kalman_res.wind.vect = mean(kalman_res.x([4:6],:)');
kalman_res.wind.direction = atan2(kalman_res.wind.vect(2),kalman_res.wind.vect(1));

plot_EKF_result(kalman_res,airspeed_estimation,airspeed_pitot,wind)

%% Print results
fprintf('----------------------------------------------\n')
fprintf('EKF TYPE              | RMS Error | Mean Error\n')
fprintf('WITH ALPHA AND BETA   |   %2.2f        %2.2f\n',kalman_res_alpha_beta.error.error_RMS,kalman_res_alpha_beta.error.error_mean)
fprintf('WITHOUT BETA          |   %2.2f        %2.2f\n',kalman_res_alpha.error.error_RMS,kalman_res_alpha.error.error_mean)
fprintf('WITHOUT ALPHA AND BETA|   %2.2f        %2.2f\n',kalman_res.error.error_RMS,kalman_res.error.error_mean)
fprintf('----------------------------------------------\n')

fprintf('--------------------------------------------------------------\n')
fprintf('EKF TYPE              | Wing Magnitude [m/s] | Direction [deg]\n')
fprintf('WITH ALPHA AND BETA   |            %2.2f        %2.2f\n',norm(kalman_res_alpha_beta.wind.vect),rad2deg(kalman_res_alpha_beta.wind.direction))
fprintf('WITHOUT BETA          |            %2.2f        %2.2f\n',norm(kalman_res_alpha.wind.vect),rad2deg(kalman_res_alpha.wind.direction))
fprintf('WITHOUT ALPHA AND BETA|            %2.2f        %2.2f\n',norm(kalman_res.wind.vect),rad2deg(kalman_res.wind.direction))
fprintf('--------------------------------------------------------------\n'),


end