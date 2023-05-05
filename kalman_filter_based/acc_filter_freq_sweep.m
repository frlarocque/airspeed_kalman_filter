%%
clear all
close all
clc

format long
% Add all paths
addpath('/home/frederic/Documents/thesis/tools/airspeed_estimation/functions/');
%% Load single filedata
clear ac_data
log_path = '/home/frederic/Documents/thesis/tools/airspeed_estimation/mat_files/log';
[file,path] = uigetfile({'*.mat'},'Select a file',log_path);

load(fullfile(path,file))
%% Select relevant data
% Setup Options
graph = 0;
beta_est = 1;
alpha_est = 1;
recalculate_variance = false;
pitot_correction = 1.0;

% Run setup
wind_triangle_setup
%% Kalman Filter Settings
kalman_filter_settings

%% Filtering and resampling
% Comment for filter accel sweep
% t = airspeed_pitot.flight.time;
% dt = mean(diff(t));
%
%
% % Get filter accel agressively
% [b,a] = butter(2,2*filter_low_freq*dt,'low');
% 
% a_x_filt = filter(b,a,IMU_accel.flight.data(:,1));
% a_y_filt = filter(b,a,IMU_accel.flight.data(:,2));
% a_z_filt = filter(b,a,IMU_accel.flight.data(:,3));
% pusher_prop_rpm_filt = filter(b,a,pusher_prop_rpm.flight.data);%filter(b,a,pusher_prop_rpm.flight.data);
% hover_prop_rpm_filt = filter(b,a,mean(hover_prop_rpm.flight.data,2));%filter(b,a,mean(hover_prop_rpm.flight.data,2));
% skew_filt = filter(b,a,skew.flight.data);
% elevator_pprz_filt = filter(b,a,control_surface_pprz.flight.data(:,4));
% 
% u_list = [IMU_accel.flight.data IMU_rate.flight.data IMU_angle.flight.data ...
%             pusher_prop_rpm_filt hover_prop_rpm_filt skew_filt elevator_pprz_filt]';
% z_list = [Vg_NED.flight.data a_x_filt a_y_filt a_z_filt airspeed_pitot.flight.data]'; %measurement
% 
% % Filter Data coming in
% [b,a] = butter(2,2*filter_high_freq*dt,'low');
% u_list(1:6,:) = filtfilt(b,a,u_list(1:6,:)')';%filter(b,a,u_list(1:6,:),[],2);
% z_list(1:3,:) = filtfilt(b,a,z_list(1:3,:)')';%filter(b,a,z_list(1:3,:),[],2);


Q = diag([[1 1 1].*EKF_AW_Q_accel,[1 1 1].*EKF_AW_Q_gyro,[EKF_AW_Q_mu 1E-6 1E-9],[1 1 1].*EKF_AW_Q_offset]); %process noise
P_0 = diag([[1 1 1].*EKF_AW_P0_V_body [1 1 1].*EKF_AW_P0_mu [1 1 1].*EKF_AW_P0_offset]); %covariance
R = diag([[1 1 1E-3].*EKF_AW_R_V_gnd EKF_AW_R_accel_filt_x EKF_AW_R_accel_filt_y EKF_AW_R_accel_filt_z EKF_AW_R_V_pitot]); %measurement noise

% Commented because freq sweep
% Resample to different sample time
%u_list = resample(u_list',t,f_EKF)';
%z_list = resample(z_list',t,f_EKF)';
%t = [t(1):1/f_EKF:t(end)]';
%dt = 1/f_EKF;
%airspeed = resample(airspeed_pitot.flight.data,airspeed_pitot.flight.time,f_EKF);

%% Frequency Sweep
freq_list =  [0.1:0.1:1 2:1:24];
kalman_res = cell(1,length(freq_list));

% Loop for all wind variances
for i=1:length(freq_list)
    fprintf('RUNNING %2.2f\n',freq_list(i))
    
    filter_low_freq = freq_list(i);
    
    t = airspeed_pitot.flight.time;
    dt = mean(diff(t));
    
    % Get filter accel agressively
    [b,a] = butter(2,2*filter_low_freq*dt,'low');
    
    a_x_filt = filter(b,a,IMU_accel.flight.data(:,1));
    a_y_filt = filter(b,a,IMU_accel.flight.data(:,2));
    a_z_filt = filter(b,a,IMU_accel.flight.data(:,3));
    pusher_prop_pwm_filt = filtfilt(b,a,pusher_prop_pwm.flight.data);%filter(b,a,pusher_prop_rpm.flight.data);
    hover_prop_pwm_filt = filtfilt(b,a,mean(hover_prop_pwm.flight.data,2));%filter(b,a,mean(hover_prop_rpm.flight.data,2));skew_filt = filter(b,a,skew.flight.data);
    skew_filt = filter(b,a,skew.flight.data);
    elevator_pwm_filt = filter(b,a,control_surface_pwm.flight.data(:,4));
    
    u_list = [IMU_accel.flight.data IMU_rate.flight.data IMU_angle.flight.data ...
            pusher_prop_pwm_filt hover_prop_pwm_filt skew_filt elevator_pwm_filt]';
    z_list = [Vg_NED.flight.data a_x_filt a_y_filt a_z_filt airspeed_pitot.flight.data]'; %measurement
    
    % Filter Data coming in
    [b,a] = butter(2,2*filter_high_freq*dt,'low');
    u_list(1:6,:) = filtfilt(b,a,u_list(1:6,:)')';%filter(b,a,u_list(1:6,:),[],2);
    z_list(1:3,:) = filtfilt(b,a,z_list(1:3,:)')';%filter(b,a,z_list(1:3,:),[],2);

    % Resample to different sample time
    u_list_resampled = resample(u_list',t,f_EKF)';
    z_list_resampled = resample(z_list',t,f_EKF)';
    t_resampled = [t(1):1/f_EKF:t(end)]';
    dt = 1/f_EKF;
    airspeed = resample(airspeed_pitot.flight.data,airspeed_pitot.flight.time,f_EKF);

    % Run filter
    kalman_res{i} = run_EKF(epsi,t_resampled,Q,R,P_0,x_0,u_list_resampled,z_list_resampled,f_fh,g_fh);
    kalman_res{i}.error = error_quantification(kalman_res{i}.x(1,logical(interp1(airspeed_pitot.flight.time,double(airspeed_pitot.flight.valid),t_resampled,'nearest')))',airspeed(logical(interp1(airspeed_pitot.flight.time,double(airspeed_pitot.flight.valid),t_resampled,'nearest'))),kalman_res{i}.u(12,:));
    error{i} = kalman_res{i}.error;
end

%% Freq sweep Graph

error_RMS = zeros(length(freq_list),4);
error_mean = zeros(length(freq_list),4);
for i=1:length(freq_list)
    error_RMS(i,:)  = [error{i}.error_RMS error{i}.hover.error_RMS error{i}.transition.error_RMS error{i}.ff.error_RMS];
    error_mean(i,:) = [error{i}.error_mean error{i}.hover.error_mean error{i}.transition.error_mean error{i}.ff.error_mean];
end

figure
subplot(2,1,1)
plot(freq_list,error_RMS,'*','MarkerSize',10)
xlim([min(freq_list) max(freq_list)]);
hold on
grid on
xlabel('Accel Filt Frequency [Hz]')
ylabel('Airspeed RMS Error [m/s]')
legend('Overall','Hover','Transition','Forward Flight')
insetAxes = axes('Position', [0.6 0.65 0.1 0.1]); % Position of the inset window
plot(freq_list,error_RMS,'*','MarkerSize',10)
grid on
% Set the x-axis limits for the inset window
xlim([0 1]);
grid on

subplot(2,1,2)
plot(freq_list,error_mean,'*','MarkerSize',10)
xlim([min(freq_list) max(freq_list)]);
hold on
grid on
xlabel('Accel Filt Frequency [Hz]')
ylabel('Airspeed Mean Error [m/s]')
legend('Hover','Transition','Forward Flight','Overall')
insetAxes = axes('Position', [0.6 0.25 0.1 0.1]); % Position of the inset window
plot(freq_list,error_mean,'*','MarkerSize',10)
grid on
% Set the x-axis limits for the inset window
xlim([0 1]);
grid on
sgtitle(['Accel Filt Frequency Sweep: ',file(1:end-4)])

%% Save data

% Light data
save(['acc_filt_frequency_sweep_',file(1:end-4),'.mat'],'error','file','error_airspeed','freq_list')

% Heavy data
%save(['covariance_sweep_',file(1:end-4),'.mat'],'kalman_res','file','error_airspeed','ac_data','cov_list')

