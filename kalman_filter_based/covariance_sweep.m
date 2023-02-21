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

% Run setup
wind_triangle_setup

%% Kalman Filter Without euler angle estimation
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
g_fh = str2func('g_2');

x_0 = [0 0 0 0 0 0]';
u_list = [IMU_accel.flight.data IMU_rate.flight.data IMU_angle.flight.data]';
z_list = [Vg_NED.flight.data alpha.flight.data beta.flight.data]'; %measurement


cov_list =  logspace(-9,-4,20); %1.43E-6 %7.8E-8; ;
kalman_res = cell(1,length(cov_list));

% Loop for all wind variances
for i=1:length(cov_list)
    fprintf('RUNNING %2.2e\n',cov_list(i))
    clear Q P_0 R
    
    wind_var = [1 1 1]*cov_list(i);
    Q = diag([IMU_accel.var,IMU_rate.var,wind_var]); %process noise
    P_0 = diag([1 1 1 wind_var]); %covariance
    R = diag([Vg_NED.var IMU_angle.var(2) 1E-6]); %measurement noise

    kalman_res{i} = run_EKF(epsi,t,Q,R,P_0,x_0,u_list,z_list,f_fh,g_fh);
    kalman_res{i}.error = error_quantification(kalman_res{i}.x(1,:)',airspeed_pitot.flight.data);
    error{i} = kalman_res{i}.error;
end

%% Covariance sweep Graph
figure
subplot(2,1,1)
for i=1:length(error)
    semilogx(cov_list(i),error{i}.error_RMS,'*','MarkerSize',10)
    hold on
    grid on
end
xlabel('Wind Covariance')
ylabel('Airspeed RMS Error [m/s]')
grid on
yline(error_airspeed.error_RMS)

subplot(2,1,2)
for i=1:length(error)
    semilogx(cov_list(i),error{i}.error_mean,'*','MarkerSize',10)
    hold on
    grid on
end
xlabel('Wind Covariance')
ylabel('Airspeed Mean Error [m/s]')
grid on
yline(error_airspeed.error_mean)
sgtitle(['Covariance Sweep: ',file(1:end-4)])

%% Save data

% Light data
save(['covariance_sweep_',file(1:end-4),'.mat'],'error','file','error_airspeed','cov_list')

% Heavy data
%save(['covariance_sweep_',file(1:end-4),'.mat'],'kalman_res','file','error_airspeed','ac_data','cov_list')

