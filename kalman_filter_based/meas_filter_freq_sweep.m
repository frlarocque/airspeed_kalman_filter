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

% Commented because freq sweep
% Resample to different sample time
%u_list = resample(u_list',t,f_EKF)';
%z_list = resample(z_list',t,f_EKF)';
%t = [t(1):1/f_EKF:t(end)]';
%dt = 1/f_EKF;
%airspeed = resample(airspeed_pitot.flight.data,airspeed_pitot.flight.time,f_EKF);

%% Frequency Sweep
freq_list =  [0.1:0.1:1 2:2:24];
kalman_res = cell(1,length(freq_list));

% Loop for all wind variances
for i=1:length(freq_list)
    fprintf('RUNNING %2.2f\n',freq_list(i))
    
    EKF_AW_MEAS_FILTERING = freq_list(i);
    
    t = airspeed_pitot.flight.time;
    dt = mean(diff(t));

    u_list = [IMU_accel.flight.data IMU_rate.flight.data IMU_angle.flight.data ...
            pusher_prop_rpm.flight.data mean(hover_prop_rpm.flight.data,2) skew.flight.data control_surface_pprz.flight.data(:,4)]';
    z_list = [Vg_NED.flight.data IMU_accel.flight.data airspeed_pitot.flight.data]'; %measurement

    % Filter Data coming in
    [b,a] = butter(2,2*EKF_AW_MEAS_FILTERING*dt,'low');
    u_list = filter(b,a,u_list,[],2);
    z_list = filter(b,a,z_list,[],2);
    
    % Resample to different sample time
    u_list_resampled = resample(u_list',t,f_EKF)';
    z_list_resampled = resample(z_list',t,f_EKF)';
    airspeed_pitot_resampled.flight.data = resample(airspeed_pitot.flight.data,t,f_EKF);
    t_resampled = [t(1):1/f_EKF:t(end)]';
    dt = 1/f_EKF;
    airspeed_pitot_resampled.flight.time = t_resampled;
    airspeed_pitot_resampled.flight.valid = logical(interp1(airspeed_pitot.flight.time,double(airspeed_pitot.flight.valid),t_resampled));
    
    % Wrap back heading to -180 to 180
    u_list_resampled(9,:) = wrapToPi(u_list_resampled(9,:));

    % Run filter
    kalman_res{i} = run_EKF(epsi,t_resampled,Q,R,P_0,x_0,u_list_resampled,z_list_resampled,f_fh,g_fh);
    kalman_res{i}.error = error_quantification_full(kalman_res{i}.x(1,:)',airspeed_pitot_resampled.flight.data,airspeed_pitot_resampled.flight.valid,kalman_res{1}.u(12,:)');
    kalman_res{i}.error.constant_wind = error_quantification(kalman_res{i}.x(1,:)',interp1(airspeed_estimation.time,airspeed_estimation.data,kalman_res{1}.t));

    error{i} = kalman_res{i}.error;
end

%% Extract data out
error_RMS_list = zeros(1,length(error));
error_mean_list = zeros(1,length(error));
% Get values out
for i=1:length(error)
    error_RMS_list(i) = error{i}.all.error_RMS;
    error_mean_list(i) = error{i}.all.error_mean;
end

%% Freq sweep Graph

error_RMS_list = zeros(1,length(freq_list));
error_mean_list = zeros(1,length(freq_list));
for i=1:length(freq_list)
    error_RMS_list(i) = error{i}.all.error_RMS;
    error_mean_list(i) = error{i}.all.error_mean;
end

figure
subplot(2,1,1)
plot(freq_list,error_RMS_list,'*','MarkerSize',10)
xlim([min(freq_list) max(freq_list)]);
hold on
grid on
xlabel('Accel Filt Frequency [Hz]')
ylabel('Airspeed RMS Error [m/s]')
insetAxes = axes('Position', [0.6 0.65 0.1 0.1]); % Position of the inset window
plot(freq_list,error_RMS_list,'*','MarkerSize',10)
grid on
% Set the x-axis limits for the inset window
xlim([0 1]);
grid on

subplot(2,1,2)
plot(freq_list,error_mean_list,'*','MarkerSize',10)
xlim([min(freq_list) max(freq_list)]);
hold on
grid on
xlabel('Accel Filt Frequency [Hz]')
ylabel('Airspeed Mean Error [m/s]')
insetAxes = axes('Position', [0.6 0.25 0.1 0.1]); % Position of the inset window
plot(freq_list,error_mean_list,'*','MarkerSize',10)
grid on
% Set the x-axis limits for the inset window
xlim([0 1]);
grid on
sgtitle(['Accel Filt Frequency Sweep: ',file(1:end-4)])

%% Nice frequency plot
AR = 4;
size = 500;

fig_height = size;
fig_width = fig_height*AR;

screen = get(0, 'ScreenSize');

if fig_width>screen(3)
    fig_width = screen(3);
    fig_height = fig_width/AR;
end
fprintf('Exporting as %.0fx%.0f \n',fig_width,fig_height);

% Get the current date and time
nowDateTime = datetime('now');

% Format the date and time in the "MM_DD_HH_MM" format
formattedDateTime = datestr(nowDateTime,'mm_dd_HH_MM');

fig = figure('position',[0 0 fig_width fig_height]);

% Store the default line width value
origLineWidth = get(groot, 'DefaultLineLineWidth');

% Set a new default line width value
set(groot, 'DefaultLineLineWidth', 2);

% Set colors and line styles
mycolors = linspecer(3,'qualitative');
mylinestyles = {'-', '--', ':'};
set(gcf,'DefaultAxesColorOrder',mycolors, ...
        'DefaultAxesLineStyleOrder',mylinestyles)

% RMS
subplot(1,1,1)
p1 = plot(freq_list,error_RMS_list);

xlabel('Measurement Filter Cutoff Frequency [Hz]')
ylabel('Airspeed Estimation RMS Error [m/s]')

grid on

% Export figure
fig_name = ['accel_freq_sweep_RMS_',formattedDateTime,'.eps'];
exportgraphics(fig,fig_name,'BackgroundColor','none','ContentType','vector')

% Mean
subplot(1,1,1)
p1 = plot(freq_list,error_mean_list);

xlabel('Measurement Filter Cutoff Frequency [Hz]')
ylabel('Airspeed Estimation Mean Error [m/s]')

grid on

% Export figure
fig_name = ['accel_freq_sweep_Mean_',formattedDateTime,'.eps'];
exportgraphics(fig,fig_name,'BackgroundColor','none','ContentType','vector')


%% Save data

% Light data
save(['acc_filt_frequency_sweep_',file(1:end-4),'.mat'],'error','file','error_airspeed','freq_list')

% Heavy data
%save(['covariance_sweep_',file(1:end-4),'.mat'],'kalman_res','file','error_airspeed','ac_data','cov_list')

