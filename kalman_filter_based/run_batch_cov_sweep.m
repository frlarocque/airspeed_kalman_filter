%%
clear all
close all
clc

format long
% Add all paths
addpath('/home/frederic/Documents/thesis/tools/airspeed_estimation/functions/');
%% Output file info
% Specify the file name and sheet name
%
excel_sheet_params = 'Params';
excel_sheet_results = 'Results';

%% Load file list
log_path = '/home/frederic/Documents/thesis/tools/airspeed_estimation/mat_files/log';
[root_path] = uigetdir(log_path,'Select a folder');

fileList = findMATFiles(root_path);

kalman_res = cell(1,length(fileList));

%% Loop through wind cov values
cov_list = logspace(-7,-1,20);

for cov_index=1:length(cov_list)
fprintf('RUNNING %2.2e\n',cov_list(cov_index))

%% Main loop
variableData = {'File Name','Path',...
                'Duration Flight [s]','Duration Hover Flight [s]','Duration Transition [s]','Duration Forward Flight [s]',...
                'Mean Pusher RPM [RPM]','Mean Hover RPM [RPM]',...
                'Mean Skew [deg]','Max Skew [deg]',...
                'Mean Airspeed [m/s]','Mean Groundspeed [m/s]','Mean Wind [m/s]',...
                'Error Overall RMS [m/s]','Error Overall Mean [m/s]','Error Overall Max [m/s]','Error Overall Min [m/s]','Error Overall Std Dev [m^2/s^2]',...
                'Error Constant Wind RMS [m/s]','Error Constant Wind  Mean [m/s]','Error Constant Wind  Max [m/s]','Error Constant Wind  Min [m/s]','Error Constant Wind  Std Dev [m^2/s^2]',...
                'Error RMS Hover [m/s]','Error RMS Transition [m/s]','Error RMS Forward Flight [m/s]'};

analyzed_files = {};
warning ('off','all');

%% Write Kalman filter settings in excel file
    excel_filename = sprintf('batch_%s_%2.2e.xlsx',datestr(datetime, 'mm_dd_HH_MM'),cov_list(cov_index));
    % Retrieve global variable names
    globalVars = who('global');
    
    % Create a cell array to store the variable names and values
    variableData_2 = {'Variable Name', 'Value'};
    for ii = 1:numel(globalVars)
        varName = globalVars{ii};
        varValue = evalin('base', varName);
        variableData_2 = [variableData_2; {varName, varValue}];
    end
    
    % Write the variable data to the Excel file
    writecell(variableData_2, excel_filename, 'Sheet', excel_sheet_params);

    %% Main loop
for iii=1:length(fileList)
    clearvars -except cov_index iii variableData fileList kalman_res excel_filename excel_sheet_params excel_sheet_results analyzed_files cov_list
    fprintf('Loading %d/%d %s\n',iii,length(fileList),fileList{iii})
    clear ac_data
    load(fileList{iii})
    [~, file, ~] = fileparts(fileList{iii});
    
    if any(strcmp(analyzed_files,file))
        fprintf('Skipped file as already analyzed\n')
    else
        
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

    %% Cov selection
    EKF_AW_Q_mu_x = cov_list(cov_index);
    EKF_AW_Q_mu_y = cov_list(cov_index);
    EKF_AW_Q_mu_z = 1E-1.*cov_list(cov_index);
    
    Q = diag([EKF_AW_Q_accel_x  EKF_AW_Q_accel_y EKF_AW_Q_accel_z ...
         EKF_AW_Q_gyro_x EKF_AW_Q_gyro_y EKF_AW_Q_gyro_z ...
         EKF_AW_Q_mu_x EKF_AW_Q_mu_y EKF_AW_Q_mu_z ...
         EKF_AW_Q_offset_x EKF_AW_Q_offset_y EKF_AW_Q_offset_z]); %process noise
    
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
    %% Run filter
    
    kalman_res{iii} = run_EKF(epsi,t,Q,R,P_0,x_0,u_list_resampled,z_list_resampled,f_fh,g_fh);
    kalman_res{iii}.error = error_quantification_full(kalman_res{iii}.x(1,:)',airspeed_pitot_resampled.flight.data,airspeed_pitot_resampled.flight.valid,kalman_res{iii}.u(12,:)');
    kalman_res{iii}.error.constant_wind = error_quantification(kalman_res{iii}.x(1,:)',interp1(airspeed_estimation.time,airspeed_estimation.data,kalman_res{iii}.t));

    %% Calculate time in different flight phases
    [bool_hover,bool_transition,bool_ff] = identify_hover_transition_ff(kalman_res{iii}.u(12,:));
    
    hover_duration = sum(bool_hover).*dt;
    transition_duration = sum(bool_transition).*dt;
    ff_duration = sum(bool_ff).*dt;

    %% Get data for excel file
%                 {'File Name','Path',...
%                 'Duration Flight [s]','Mean Pusher RPM [RPM]','Mean Hover RPM [RPM]',...
%                 'Mean Skew [deg]','Max Skew [deg]',...
%                 'Mean Airspeed [m/s]','Mean Groundspeed [m/s]','Mean Wind [m/s]',...
%                 'Error Overall RMS [m/s]','Error Overall Mean [m/s]','Error Overall Max [m/s]','Error Overall Min [m/s]','Error Overall Std Dev [m^2/s^2]',...
%                 'Error Constant Wind RMS [m/s]','Error Constant Wind  Mean [m/s]','Error Constant Wind  Max [m/s]','Error Constant Wind  Min [m/s]','Error Constant Wind  Std Dev [m^2/s^2]',...
%                 'Error RMS Hover [m/s]','Error RMS Transition [m/s]','Error RMS Forward Flight [m/s]'};
    
    variableData = [variableData; {file, fileList{iii},...
                    kalman_res{iii}.t(end)-kalman_res{iii}.t(1),hover_duration,transition_duration,ff_duration...
                    mean(kalman_res{iii}.u(10,:)), mean(kalman_res{iii}.u(11,:)),...
                    rad2deg(mean(kalman_res{iii}.u(12,:))),rad2deg(max(kalman_res{iii}.u(12,:))),...
                    mean(airspeed_pitot.flight.data(airspeed_pitot.flight.valid)),mean(vecnorm(Vg_NED.flight.data,2,2)) ,wind.norm ,...
                    kalman_res{iii}.error.valid_pitot.error_RMS, kalman_res{iii}.error.valid_pitot.error_mean, kalman_res{iii}.error.valid_pitot.error_max, kalman_res{iii}.error.valid_pitot.error_min, kalman_res{iii}.error.valid_pitot.std_dev,...
                    kalman_res{iii}.error.constant_wind.error_RMS, kalman_res{iii}.error.constant_wind.error_mean, kalman_res{iii}.error.constant_wind.error_max, kalman_res{iii}.error.constant_wind.error_min, kalman_res{iii}.error.constant_wind.std_dev,...
                    kalman_res{iii}.error.hover.error_RMS,kalman_res{iii}.error.transition.error_RMS,kalman_res{iii}.error.ff.error_RMS}];

analyzed_files{end+1} = file;
    end
end
warning ('on','all');
%% To excel file

writecell(variableData, excel_filename , 'Sheet', excel_sheet_results);

end
