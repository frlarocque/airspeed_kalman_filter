%% Load list of files
clear all
clc
close all

% Add all paths
log_path = '/home/frederic/Documents/thesis/tools/airspeed_estimation/mat_files/log/to_analyze/v3a';

addpath('/home/frederic/Documents/thesis/tools/airspeed_estimation/kalman_filter_based/');

file_list = dir(fullfile(log_path, '*.mat'));
%% Go through all logs in log path

results = cell(length(file_list),1);

for index=1:length(file_list)
    clear ac_data hover_prop_rpm
    
    fprintf("%d/%d\n",index,length(file_list));
    load(fullfile(file_list(index).folder,file_list(index).name));
    
    fprintf("FOLDER: %s\nNAME: %s\nLOGGED ON: %s\n",file_list(index).folder,file_list(index).name,file_list(index).date);
    
    results{index}.file_info = file_list(index); 
    %% Wind triangle setup
    % Setup Options
    graph = 0;
    beta_est = 1;
    alpha_est = 1;
    recalculate_variance = false;
    pitot_correction = 1.0;
    
    file = file_list(index).name;
    % Run setup
    wind_triangle_setup

    %% Fit Drag Model
    
    t = airspeed_pitot.flight.time;
    dt = mean(diff(t));
    filter_freq = 0.1; %[Hz]
    [b,a] = butter(2,2*filter_freq*dt,'low');

    %Only select data with valid pitot tube data
    cond = airspeed_pitot.flight.valid;
    
    a_x_filt = filter(b,a,IMU_accel.flight.data(:,1));
    a_y_filt = filter(b,a,IMU_accel.flight.data(:,2));
    pusher_prop_rpm_filt = filter(b,a,pusher_prop_rpm.flight.data);
    elevator_angle = filter(b,a,elevator_pprz2angle(control_surface_pprz.flight.data(:,4)));
    
    figure
    plot(t(cond),rad2deg(elevator_angle(cond)))

    %% Fit ax

    x = [Fx_pusher_2(pusher_prop_rpm_filt(cond),airspeed_pitot.flight.data(cond)) airspeed_pitot.flight.data(cond) elevator_angle(cond)];
    y = a_x_filt(cond);
    
    fit = @(k,x)  (k(1).*x(:,1)+k(2).*x(:,2)+k(3).*x(:,2).^2+ k(4).*x(:,3).*x(:,2).^2)./5.75;          % Function to fit
    fcn = @(k) sqrt(mean((fit(k,x) - y).^2));  % Least-Squares cost function
    [results{index}.s_x,results{index}.RMS_x] = fminsearchbnd(fcn,[1 -2E-1 -4E-2 0],[1 -8E-1 -8E-2 0],[1 0 0 0]); 

    results{index}.t = t(cond);
    results{index}.pusher_prop = pusher_prop_rpm_filt(cond);
    results{index}.airspeed = airspeed_pitot.flight.data(cond);

    results{index}.Fx_pusher = Fx_pusher(pusher_prop_rpm_filt(cond),airspeed_pitot.flight.data(cond));
    results{index}.a_x = a_x_filt(cond);
    
    figure
    plot(t(cond),y)
    hold on
    plot(t(cond),fit(results{index}.s_x,x))
    xlabel('Time [s]')
    ylabel('Acceleration in x axis [m/s^2]')
    title([results{index}.file_info.name])

    %% Fit a_y with beta
    x = [beta.flight.data(cond) airspeed_pitot.flight.data(cond)];
    y = a_y_filt(cond);
    
    fit = @(k,x)  k(1).*x(:,1).*x(:,2).^2/5.75;          % Function to fit
    fcn = @(k) sqrt(mean((fit(k,x) - y).^2));  % Least-Squares cost function
    [results{index}.s_beta,results{index}.RMS_beta] = fminsearchbnd(fcn,[-1.15],[10],[-10]); 
    
    results{index}.beta = beta.flight.data(cond);
    results{index}.a_y = a_y_filt(cond);

    figure
    plot(t(cond),y)
    hold on
    plot(t(cond),fit(results{index}.s_beta,x))
    xlabel('Time [s]')
    ylabel('Acceleration in y axis [m/s^2]')
    title([results{index}.file_info.name])
    
    x = [sin(beta.flight.data(cond)).*airspeed_pitot.flight.data(cond)];
    y = a_y_filt(cond);

    fit = @(k,x)  (k(1).*x(:,1).^2.*sign(x(:,1)))/5.75;          % Function to fit
    fcn = @(k) sqrt(mean((fit(k,x) - y).^2));  % Least-Squares cost function
    [results{index}.s_v,results{index}.RMS_v] = fminsearchbnd(fcn,[-1.15],[10],[-10]); 
    
    results{index}.v = sin(beta.flight.data(cond)).*airspeed_pitot.flight.data(cond);
    results{index}.a_y = a_y_filt(cond);

    figure
    plot(t(cond),y)
    hold on
    plot(t(cond),fit(results{index}.s_beta,x))
    xlabel('Time [s]')
    ylabel('Acceleration in y axis [m/s^2]')
    title([results{index}.file_info.name])
end

%% Print s_x
fprintf('Index  | Mean V | Std Dev V |   s(1)   |   s(2)   |   s(3)   \n')
sx_temp = [];
for i=1:length(results)
    fprintf('   %d      %2.2f      %2.2f     %2.2e   %2.2e   %2.2e\n',i,mean(results{i}.airspeed),std(results{i}.airspeed),results{i}.s_x(1),results{i}.s_x(2),results{i}.s_x(3))
    sx_temp(i,:) = results{i}.s_x;
    v_temp(i,:) = mean(results{i}.airspeed);
end
fprintf('--------------------------------------------------------------\n')
fprintf('   Mean      -         -     %2.2e   %2.2e   %2.2e\n',mean(sx_temp))

figure
subplot(3,1,1)
plot(v_temp,sx_temp(:,1),'*')
xlabel('Airspeed [m/s]')
ylabel('s(1)')
subplot(3,1,2)
plot(v_temp,sx_temp(:,2),'*')
xlabel('Airspeed [m/s]')
ylabel('s(2)')
subplot(3,1,3)
plot(v_temp,sx_temp(:,3),'*')
xlabel('Airspeed [m/s]')
ylabel('s(3)')

%% Print s_y
fprintf('Index  | Mean V | Std Dev V |   s_beta   |   s_v  \n')
sy_temp = [];
for i=1:length(results)
    fprintf('   %d      %2.2f      %2.2f     %2.2e   %2.2e  \n',i,mean(results{i}.airspeed),std(results{i}.airspeed),results{i}.s_beta,results{i}.s_v)
    sy_temp(i,:) = [results{i}.s_beta results{i}.s_v];
    v_temp(i,:) = mean(results{i}.airspeed);
end
fprintf('--------------------------------------------------------------\n')
fprintf('   Mean      -         -     %2.2e   %2.2e  \n',mean(sy_temp))

figure
subplot(2,1,1)
plot(v_temp,sy_temp(:,1),'*')
xlabel('Airspeed [m/s]')
ylabel('s(1)')
subplot(2,1,2)
plot(v_temp,sy_temp(:,2),'*')
xlabel('Airspeed [m/s]')
ylabel('s(2)')


%% 
select = 18;

figure
subplot(3,1,1)
plot(results{select}.t,results{select}.airspeed)
xlabel('Time [s]')
ylabel('Airspeed [m/s]')

subplot(3,1,2)
plot(results{select}.t,results{select}.a_x)
xlabel('Time [s]')
ylabel('Acceleration x axis [m/s^2]')

subplot(3,1,3)
plot(results{select}.t,results{select}.pusher_prop)
xlabel('Time [s]')
ylabel('Pusher Prop RPM')


%%
v_list = [0:0.2:15];

figure
plot(v_list,v_list.*-2.14E-01)
hold on
plot(v_list,v_list.^2.*-5.02E-02)
plot(v_list,v_list.*-2.14E-01+v_list.^2.*-5.02E-02)
legend('Linear','Quadratic','Sum')