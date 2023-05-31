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
t = airspeed_pitot.flight.time;
dt = mean(diff(t));

u_list = [IMU_accel.flight.data IMU_rate.flight.data IMU_angle.flight.data ...
            pusher_prop_rpm.flight.data mean(hover_prop_rpm.flight.data,2) skew.flight.data control_surface_pprz.flight.data(:,4)]';
z_list = [Vg_NED.flight.data IMU_accel.flight.data airspeed_pitot.flight.data]'; %measurement

% Filter Data coming in
[b,a] = butter(2,2*EKF_AW_MEAS_FILTERING*dt,'low');
u_list = filter(b,a,u_list,[],2);
z_list = filter(b,a,z_list,[],2);

% Commented because covariance sweep
%Q = diag([[1 1 1].*EKF_AW_Q_accel,[1 1 1].*EKF_AW_Q_gyro,[1 1 1E-2].*EKF_AW_Q_mu,[1 1 1].*EKF_AW_Q_offset]); %process noise
%P_0 = diag([[1 1 1].*EKF_AW_P0_V_body [1 1 1].*EKF_AW_P0_mu [1 1 1].*EKF_AW_P0_offset]); %covariance
%R = diag([[1 1 1].*EKF_AW_R_V_gnd EKF_AW_R_accel_filt_x EKF_AW_R_accel_filt_y EKF_AW_R_accel_filt_z EKF_AW_R_V_pitot]); %measurement noise

% Resample to different sample time
u_list_resampled = resample(u_list',t,f_EKF)';
z_list_resampled = resample(z_list',t,f_EKF)';
airspeed_pitot_resampled.flight.data = resample(airspeed_pitot.flight.data,t,f_EKF);
t = [t(1):1/f_EKF:t(end)]';
dt = 1/f_EKF;
airspeed_pitot_resampled.flight.time = t;
airspeed_pitot_resampled.flight.valid = logical(interp1(airspeed_pitot.flight.time,double(airspeed_pitot.flight.valid),t));

% Wrap back heading to -180 to 180
u_list_resampled(9,:) = wrapToPi(u_list_resampled(9,:));

%% Covariance Sweep
cov_list =  logspace(-9,-2,40); %1.43E-6 %7.8E-8; ;
kalman_res = cell(1,length(cov_list));

% Loop for all wind variances
for i=1:length(cov_list)
    fprintf('RUNNING %2.2e\n',cov_list(i))
    clear Q P_0 R
    
    EKF_AW_Q_mu_x = cov_list(i);
    EKF_AW_Q_mu_y = cov_list(i);
    EKF_AW_Q_mu_z = 1E-1.*cov_list(i);
    
    Q = diag([EKF_AW_Q_accel_x  EKF_AW_Q_accel_y EKF_AW_Q_accel_z ...
         EKF_AW_Q_gyro_x EKF_AW_Q_gyro_y EKF_AW_Q_gyro_z ...
         EKF_AW_Q_mu_x EKF_AW_Q_mu_y EKF_AW_Q_mu_z ...
         EKF_AW_Q_offset_x EKF_AW_Q_offset_y EKF_AW_Q_offset_z]); %process noise
    
    % Run filter
    kalman_res{i} = run_EKF(epsi,t,Q,R,P_0,x_0,u_list_resampled,z_list_resampled,f_fh,g_fh,false);
    kalman_res{i}.error = error_quantification_full(kalman_res{i}.x(1,:)',airspeed_pitot_resampled.flight.data,airspeed_pitot_resampled.flight.valid,kalman_res{i}.u(12,:)');
    kalman_res{i}.error.constant_wind = error_quantification(kalman_res{i}.x(1,:)',interp1(airspeed_estimation.time,airspeed_estimation.data,kalman_res{i}.t));
    error{i} = kalman_res{i}.error;
end

%% Extract error out
error_RMS_list = zeros(1,length(error));
error_mean_list = zeros(1,length(error));
% Get values out
for i=1:length(error)
    error_RMS_list(i) = error{i}.all.error_RMS;
    error_mean_list(i) = error{i}.all.error_mean;
end

%% Nice frequency plot
set(gcf, 'Renderer', 'Painters');
AR = 2;
size = 500;
font_size = 20;

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
p1 = semilogx(cov_list,error_RMS_list);

xlabel('Wind Covariance [(m/s)²]')
ylabel('Airspeed Estimation RMS Error [m/s]')
axis([1E-9 1E-2 -inf inf])

grid on
%grid minor

% Change font size
set(findall(gcf,'-property','FontSize'),'FontSize',font_size)

% Export figure
fig_name = ['wind_cov_sweep_RMS_',formattedDateTime,'.eps'];
exportgraphics(fig,fig_name,'BackgroundColor','none','ContentType','vector')

% Mean
subplot(1,1,1)
p1 = semilogx(cov_list,error_mean_list);

xlabel('Wind Covariance [(m/s)²]')
ylabel('Airspeed Estimation Mean Error [m/s]')

grid on
%grid minor

% Change font size
set(findall(gcf,'-property','FontSize'),'FontSize',font_size)

% Export figure
fig_name = ['wind_cov_sweep_Mean_',formattedDateTime,'.eps'];
exportgraphics(fig,fig_name,'BackgroundColor','none','ContentType','vector')

%% Save data

% Light data
save(['covariance_sweep_',file(1:end-4),'.mat'],'error','file','error_airspeed','cov_list')

% Heavy data
%save(['covariance_sweep_',file(1:end-4),'.mat'],'kalman_res','file','error_airspeed','ac_data','cov_list')

