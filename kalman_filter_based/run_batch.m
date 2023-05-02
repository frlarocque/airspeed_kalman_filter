%%
clear all
close all
clc

format long
% Add all paths
addpath('/home/frederic/Documents/thesis/tools/airspeed_estimation/functions/');
%% Output file info
% Specify the file name and sheet name
excel_filename = sprintf('batch_%s.xlsx',datestr(datetime, 'mm_dd_HH_MM'));
excel_sheet_params = 'Params';
excel_sheet_results = 'Results';

%% Load file list
log_path = '/home/frederic/Documents/thesis/tools/airspeed_estimation/mat_files/log';
[root_path] = uigetdir(log_path,'Select a folder');

fileList = findMATFiles(root_path);

kalman_res = cell(1,length(fileList));

%% Kalman Filter Settings
kalman_filter_settings

%% Write Kalman filter settings in excel file
% Retrieve global variable names
globalVars = who('global');

% Create a cell array to store the variable names and values
variableData = {'Variable Name', 'Value'};
for i = 1:numel(globalVars)
    varName = globalVars{i};
    varValue = evalin('base', varName);
    variableData = [variableData; {varName, varValue}];
end

% Write the variable data to the Excel file
writecell(variableData, excel_filename, 'Sheet', excel_sheet_params);

%% Main loop

variableData = {'File Name','Path','Mean Pusher RPM [RPM]','Mean Hover RPM [RPM]','Mean Skew [deg]','Mean Airspeed [m/s]','Mean Groundspeed [m/s]','Mean Wind [m/s]','Error RMS [m/s]','Error Mean [m/s]','Error Max [m/s]','Error Min [m/s]','Error Std Dev [m^2/s^2]'};

for iii=1:length(fileList)
    clearvars -except iii variableData fileList kalman_res excel_filename excel_sheet_params excel_sheet_results
    fprintf('Loading %d/%d %s\n',iii,length(fileList),fileList{iii})
    clear ac_data
    load(fileList{iii})
    [~, file, ~] = fileparts(fileList{iii});

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
    
    % Get filter accel agressively
    [b,a] = butter(2,2*filter_low_freq*dt,'low');
    
    a_x_filt = filter(b,a,IMU_accel.flight.data(:,1));
    a_y_filt = filter(b,a,IMU_accel.flight.data(:,2));
    a_z_filt = filter(b,a,IMU_accel.flight.data(:,3));
    pusher_prop_rpm_filt = filter(b,a,pusher_prop_rpm.flight.data);%filter(b,a,pusher_prop_rpm.flight.data);
    hover_prop_rpm_filt = filter(b,a,mean(hover_prop_rpm.flight.data,2));%filter(b,a,mean(hover_prop_rpm.flight.data,2));
    skew_filt = filter(b,a,skew.flight.data);
    elevator_pprz_filt = filter(b,a,control_surface_pprz.flight.data(:,4));
    
    u_list = [IMU_accel.flight.data IMU_rate.flight.data IMU_angle.flight.data ...
                pusher_prop_rpm_filt hover_prop_rpm_filt skew_filt elevator_pprz_filt]';
    z_list = [Vg_NED.flight.data a_x_filt a_y_filt a_z_filt airspeed_pitot.flight.data]'; %measurement
    
    % Filter Data coming in
    [b,a] = butter(2,2*filter_high_freq*dt,'low');
    u_list(1:6,:) = filtfilt(b,a,u_list(1:6,:)')';%filter(b,a,u_list(1:6,:),[],2);
    z_list(1:3,:) = filtfilt(b,a,z_list(1:3,:)')';%filter(b,a,z_list(1:3,:),[],2);
    
    Q = diag([[1 1 1].*EKF_AW_Q_accel,[1 1 1].*EKF_AW_Q_gyro,[1 1 1E-2].*EKF_AW_Q_mu,[1 1 1].*EKF_AW_Q_offset]); %process noise
    P_0 = diag([[1 1 1].*EKF_AW_P0_V_body [1 1 1].*EKF_AW_P0_mu [1 1 1].*EKF_AW_P0_offset]); %covariance
    R = diag([[1 1 1].*EKF_AW_R_V_gnd EKF_AW_R_accel_filt_x EKF_AW_R_accel_filt_y EKF_AW_R_accel_filt_z EKF_AW_R_V_pitot]); %measurement noise
    
    % Resample to different sample time
    u_list = resample(u_list',t,f_EKF)';
    z_list = resample(z_list',t,f_EKF)';
    t = [t(1):1/f_EKF:t(end)]';
    dt = 1/f_EKF;
    airspeed = resample(airspeed_pitot.flight.data,airspeed_pitot.flight.time,f_EKF);

    %% Run filter
    
    kalman_res{iii} = run_EKF(epsi,t,Q,R,P_0,x_0,u_list,z_list,f_fh,g_fh);
    kalman_res{iii}.error = error_quantification(kalman_res{iii}.x(1,logical(interp1(airspeed_pitot.flight.time,double(airspeed_pitot.flight.valid),t,'nearest')))',airspeed(logical(interp1(airspeed_pitot.flight.time,double(airspeed_pitot.flight.valid),t,'nearest'))));
    
    %% Get data for excel file
                %variableData = {'File Name','Path','Mean Pusher RPM','Mean Hover RPM','Mean Skew','Mean Airspeed','Mean Wind','Error RMS','Error Mean','Error Max','Error Min','Error Std Dev'};
    
    variableData = [variableData; {file, fileList{iii}, mean(kalman_res{iii}.u(10,:)), mean(kalman_res{iii}.u(11,:)), rad2deg(mean(kalman_res{iii}.u(12,:))), mean(airspeed_pitot.flight.data(airspeed_pitot.flight.valid)),mean(vecnorm(Vg_NED.flight.data,2,2)) ,wind.norm ,kalman_res{iii}.error.error_RMS, kalman_res{iii}.error.error_mean, kalman_res{iii}.error.error_max, kalman_res{iii}.error.error_min, kalman_res{iii}.error.std_dev}];
end

%% To excel file
writecell(variableData, excel_filename, 'Sheet', excel_sheet_results);
