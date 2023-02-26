%%
clear all
close all
clc

% Add all paths
addpath('/home/frederic/Documents/thesis/tools/airspeed_estimation/functions/');

%% Load single filedata
[file,path] = uigetfile({'*.mat'},'Select a file');

load(fullfile(path,file))

%% Setup
% Setup Options
graph = 0;
beta_est = 0;
recalculate_variance = false;
pitot_correction = 1.1;

% Run setup
wind_triangle_setup

%% Kalman Filter Without Euler angle estimation
% 𝑢, 𝑣, 𝑤 = Velocity in the Body Frame
% 𝜇𝑥, 𝜇𝑦, 𝜇𝑧 = Wind Velocity in Earth Fixed Frame
% 𝑝, 𝑞, 𝑟 = Angular Rates Measured by Gyroscopes
% 𝑎x, 𝑎𝑦, 𝑎𝑧 = Accelerations Measured by Accelerometers
% 𝑉𝑥 , 𝑉𝑦, 𝑉𝑧 = Velocity in Earth Fixed Frame
% alpha, beta = angle of attack and sideslip angle

%x = [u v w mu_x mu_y mu_z];
%u = [a_x a_y a_z p q r phi theta psi];
%z = [V_x V_y V_z alpha beta];

%w_w = [0;0;0]; %noise angular rate
%w_a = [0;0;0]; %noise accelerations
%w_mu =[0;0;0]; %noise wind

epsi = 1E-2;

t = airspeed_pitot.flight.time;
dt = mean(t(2:end)-t(1:end-1));

f_fh = str2func('f_2');
g_fh = str2func('g_4');

x_0 = [0 0 0 0 0 0]';
u_list = [IMU_accel.flight.data IMU_rate.flight.data IMU_angle.flight.data]';
z_list = [Vg_NED.flight.data]'; %measurement


cov_list =  1.43E-6; %7.8E-8; ;
kalman_res = {};

    
wind_var = [1 1 1]*cov_list(1);
Q = diag([IMU_accel.var,IMU_rate.var,wind_var]); %process noise
P_0 = diag([1 1 1 wind_var]); %covariance
R = diag([Vg_NED.var]); %measurement noise

kalman_res{1} = run_EKF(epsi,t,Q,R,P_0,x_0,u_list,z_list,f_fh,g_fh);
kalman_res{1}.error = error_quantification(kalman_res{1}.x(1,:)',airspeed_pitot.flight.data);

%% Plot
plot_EKF_result(kalman_res{1},airspeed_estimation,airspeed_pitot,wind)
fprintf('Estimated wind (using Kalman Filter) is %0.2f m/s going %0.2f deg\n',mean(sqrt(kalman_res{1}.x(4,:).^2+kalman_res{1}.x(5,:).^2)),rad2deg(atan2(mean(kalman_res{select}.x(4,:)),mean(kalman_res{select}.x(5,:)))))

%% Plot covariance
select = 1;

P_temp = zeros(length(kalman_res{select}.P),length(kalman_res{select}.P{1}));
for k=1:length(kalman_res{select}.P)
    P_temp(k,:) = kalman_res{select}.P{k}(sub2ind(size(kalman_res{1}.P{k}),1:size(kalman_res{select}.P{k},1),1:size(kalman_res{select}.P{k},2))); %get diagonal elements
end

figure
ax1 = subplot(2,1,1);
semilogy(kalman_res{select}.t,P_temp(:,[1:3]))
xlabel('Time [s]')
ylabel('Covariance')
title('Body Velocity')
legend('u','v','w')
grid on

ax2 = subplot(2,1,2);
semilogy(kalman_res{select}.t,P_temp(:,[4:6]))
xlabel('Time [s]')
ylabel('Covariance')
title('Wind Velocity')
legend('mu_x','mu_y','mu_z')
grid on

linkaxes([ax1,ax2],'x')
sgtitle(sprintf('Wind Covariance %.1d | RMS error %.2f',cov_list(select),kalman_res{select}.error.error_RMS))