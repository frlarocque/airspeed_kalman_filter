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

Q = diag([[1 1 1E1].*EKF_AW_Q_accel,[1 1 1].*EKF_AW_Q_gyro,[1 1 1E-2].*EKF_AW_Q_mu,[1 1 1].*EKF_AW_Q_offset]); %process noise
P_0 = diag([[1 1 1].*EKF_AW_P0_V_body [1 1 1].*EKF_AW_P0_mu [1 1 1].*EKF_AW_P0_offset]); %covariance
R = diag([[1 1 1].*EKF_AW_R_V_gnd EKF_AW_R_accel_filt_x EKF_AW_R_accel_filt_y EKF_AW_R_accel_filt_z EKF_AW_R_V_pitot]); %measurement noise

f_EKF = 25;
% Resample to different sample time
u_list_resampled = resample(u_list',t,f_EKF)';
z_list_resampled = resample(z_list',t,f_EKF)';
airspeed_pitot_resampled.flight.data = resample(airspeed_pitot.flight.data,t,f_EKF);
t = [t(1):1/f_EKF:t(end)]';
dt = 1/f_EKF;
airspeed_pitot_resampled.flight.time = t;
airspeed_pitot_resampled.flight.valid = logical(interp1(airspeed_pitot.flight.time,double(airspeed_pitot.flight.valid),t));
%% Run filter
kalman_res = {};
[EKF_res] = run_EKF(epsi,t,Q,R,P_0,x_0,u_list_resampled,z_list_resampled,f_fh,g_fh,true);

kalman_res{1} = EKF_res;

kalman_res{1}.error = error_quantification_full(kalman_res{1}.x(1,:)',airspeed_pitot_resampled.flight.data,airspeed_pitot_resampled.flight.valid,kalman_res{1}.u(12,:)');
kalman_res{1}.error.constant_wind = error_quantification(kalman_res{1}.x(1,:)',interp1(airspeed_estimation.time,airspeed_estimation.data,kalman_res{1}.t));

fprintf("FINISHED!\n \nWAKE UP!\n")

%% Run failure detection
filter_detection_freq = 0.1; %[Hz]
[b,a] = butter(2,2*filter_detection_freq*dt,'low');
low_pass_online = filter_discrete(b,a,0,0);

crit_low = 3;  %[m/s]
crit_high = 8; %[m/s] %could be variable with speed (as a percentage of current airspeed), with a floor For example: 

time_low = 2;    %sec
time_high = 0.2; %sec


count_low = 0;
count_high = 0;
flag_low_fault = false;
flag_high_fault = false;
log_var = [];
for i=1:length(kalman_res{1}.t)
    innov_pitot = kalman_res{1}.y(7,i);
    
    innov_pitot_filt = low_pass_online.update_filter_discrete(innov_pitot);

    if count_high>time_high./dt
        fprintf('High Pitot tube error at %2.2f\n',i*dt);
        %return
    end

    if innov_pitot>crit_high && ~flag_high_fault
        flag_high_fault = 1;
    elseif innov_pitot>crit_high && flag_high_fault
        count_high = count_high+1;
    else
        flag_high_fault = 0;
        count_high = 0;
    end

    if count_low>time_low./dt
        fprintf('Low Pitot tube error at %2.2f\n',i*dt);
        %return
    end

    if innov_pitot_filt>crit_low && ~flag_low_fault
        flag_low_fault = 1;
    elseif innov_pitot_filt>crit_low && flag_low_fault
        count_low = count_low+1;
    else
        flag_low_fault = 0;
        count_low = 0;
    end


    log_var(end+1,:) = [i innov_pitot innov_pitot_filt count_low count_high];
end

figure;
ax1 = subplot(2,1,1);
s1=plot(log_var(:,1)*dt,log_var(:,2));
hold on
s2=plot(log_var(:,1)*dt,log_var(:,3));
s3=yline(crit_low,'--','color',"#D95319");yline(-crit_low,'o--','color',"#D95319")
s4=yline(crit_high,'r--');yline(-crit_high,'r--')
legend([s1,s2,s3,s4],'Innov','LP','Low Criteria','High Criteria')
xlabel('Time [s]')
ylabel('Innovation [m/s]')

ax2 = subplot(2,1,2);
s1 = plot(log_var(:,1)*dt,log_var(:,4)*dt);
hold on
s2 = plot(log_var(:,1)*dt,log_var(:,5)*dt);
s3 = yline(time_low,'--','color',"#D95319");
s4 = yline(time_high,'r--');
legend([s1 s2 s3 s4],'Low Criteria','High Criteria','Limit Low','Limit High')
xlabel('Time [s]')
ylabel('Time for fault')
axis([-inf inf 0 1.1*time_low])

linkaxes([ax1 ax2],'x')

%% Plot
select = 1;
%plot_EKF_result(kalman_res{select},airspeed_pitot.flight,wind)
plot_EKF_result_full(kalman_res{select},airspeed_pitot.flight,beta.flight,alpha.flight,wind)
fprintf('Estimated wind (using Kalman Filter) is %0.2f m/s going %0.2f deg\n',mean(vecnorm(kalman_res{select}.x(4:6,:),2)),rad2deg(atan2(mean(kalman_res{select}.x(4,:)),mean(kalman_res{select}.x(5,:)))))
fprintf('Error RMS Overall %2.2f Hover %2.2f Transition %2.2f FF %2.2f\n',kalman_res{1}.error.valid_pitot.error_RMS,kalman_res{1}.error.hover.error_RMS,kalman_res{1}.error.transition.error_RMS,kalman_res{1}.error.ff.error_RMS)

if EKF_AW_PROPAGATE_OFFSET
figure;
subplot(3,1,1)
plot(kalman_res{1}.t,kalman_res{1}.x(7,:)')
xlabel('Time [s]')
ylabel('offset_x')
subplot(3,1,2)
plot(kalman_res{1}.t,kalman_res{1}.x(8,:)')
xlabel('Time [s]')
ylabel('offset_y')
subplot(3,1,3)
plot(kalman_res{1}.t,kalman_res{1}.x(9,:)')
xlabel('Time [s]')
ylabel('offset_z')
end

figure;

subplot(4,1,1)
plot(kalman_res{1}.t,kalman_res{1}.y(1:3,:)')
grid on
xlabel('Time [s]')
ylabel('Innovation')
title('V_{gnd}')
legend('N','E','D')

subplot(4,1,2)
plot(kalman_res{1}.t,kalman_res{1}.y(4:6,:)')
grid on
xlabel('Time [s]')
ylabel('Innovation')
title('Accel')
legend('x','y','z')

subplot(4,1,3)
plot(kalman_res{1}.t,kalman_res{1}.y(7,:)')
grid on
xlabel('Time [s]')
ylabel('Innovation')
title('Pitot')

subplot(4,1,4)
yyaxis left
plot(kalman_res{1}.t,rad2deg(kalman_res{1}.u(12,:)))
xlabel('Time [s]')
ylabel('Skew [deg]')

% Residuals histogram
residual_hist(kalman_res{1}.y',100)

%% Plot covariance

P_temp = zeros(length(kalman_res{select}.P),length(kalman_res{select}.P{1}));
for k=1:length(kalman_res{select}.P)
    P_temp(k,:) = kalman_res{select}.P{k}(sub2ind(size(kalman_res{select}.P{k}),1:size(kalman_res{select}.P{k},1),1:size(kalman_res{select}.P{k},2))); %get diagonal elements
    R_temp(k,:) = kalman_res{select}.R{k}(sub2ind(size(kalman_res{select}.R{k}),1:size(kalman_res{select}.R{k},1),1:size(kalman_res{select}.R{k},2))); %get diagonal elements
    Q_temp(k,:) = kalman_res{select}.Q{k}(sub2ind(size(kalman_res{select}.Q{k}),1:size(kalman_res{select}.Q{k},1),1:size(kalman_res{select}.Q{k},2))); %get diagonal elements
end

figure
ax1 = subplot(2,1,1);
semilogy(kalman_res{select}.t,R_temp(:,[4]))
ax2 = subplot(2,1,2);
semilogy(kalman_res{select}.t,Q_temp(:,[7:9]))

figure
ax1 = subplot(2,1,1);
semilogy(kalman_res{select}.t,P_temp(:,[1:2]))
xlabel('Time [s]')
ylabel('Covariance')
title('Body Velocity')
legend('u','v')
grid on

ax2 = subplot(2,1,2);
loglog(kalman_res{select}.t,P_temp(:,[4:5]))
xlabel('Time [s]')
ylabel('Covariance')
title('Wind Velocity')
legend('mu_x','mu_y')
grid on

linkaxes([ax1,ax2],'x')
sgtitle(sprintf('Wind Covariance %.1d | RMS error %.2f',cov_list(select),kalman_res{select}.error.error_RMS))

%% Plot gains
figure
n = size(kalman_res{select}.K{k},1);
m = size(kalman_res{select}.K{k},2);
dk = floor(length(kalman_res{select}.K)./100);
AX = [];
for k=1:dk:length(kalman_res{select}.K)
    for i=1:n
        for j=1:m
        AX (i,j) = subplot(n,m,j+(i-1)*m);
        semilogy(t(k),abs(kalman_res{select}.K{k}(i,j)),'*')
        hold on
        end
    end
end
for i=1:n
    linkaxes([AX(i,:)],'y')
end
