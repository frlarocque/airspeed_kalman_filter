%%
clear all
close all
clc

format long
% Add all paths
addpath('/home/frederic/Documents/thesis/tools/airspeed_estimation/functions/');
%% Load single filedata
clear ac_data
log_path = '/home/frederic/Documents/thesis/tools/airspeed_estimation/mat_files/log/wind_tunnel_prototype';
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

Q = diag([[1 1 1].*EKF_AW_Q_accel,[1 1 1].*EKF_AW_Q_gyro,[1 1 1E-12].*EKF_AW_Q_mu,[1 1 1].*EKF_AW_Q_offset]); %process noise
P_0 = diag([[1 1 1].*EKF_AW_P0_V_body [1 1 1].*EKF_AW_P0_mu [1 1 1].*EKF_AW_P0_offset]); %covariance
R = diag([[1 1 1E-3].*EKF_AW_R_V_gnd EKF_AW_R_accel_filt_x EKF_AW_R_accel_filt_y EKF_AW_R_accel_filt_z EKF_AW_R_V_pitot]); %measurement noise

f_EKF = 20;
% Resample to different sample time
u_list = resample(u_list',t,f_EKF)';
z_list = resample(z_list',t,f_EKF)';
t = [t(1):1/f_EKF:t(end)]';
dt = 1/f_EKF;
airspeed = resample(airspeed_pitot.flight.data,airspeed_pitot.flight.time,f_EKF);

%% Run filter
kalman_res = {};
[EKF_res] = run_EKF(epsi,t,Q,R,P_0,x_0,u_list,z_list,f_fh,g_fh,true);

kalman_res{1} = EKF_res;

kalman_res{1}.error = error_quantification(kalman_res{1}.x(1,logical(interp1(airspeed_pitot.flight.time,double(airspeed_pitot.flight.valid),t,'nearest')))',airspeed(logical(interp1(airspeed_pitot.flight.time,double(airspeed_pitot.flight.valid),t,'nearest'))));

fprintf("FINISHED!\n \nWAKE UP!\n")

%% Plot
select = 1;
%plot_EKF_result(kalman_res{select},airspeed_pitot.flight,wind)
plot_EKF_result_full(kalman_res{select},airspeed_pitot.flight,beta.flight,alpha.flight,wind)
fprintf('Estimated wind (using Kalman Filter) is %0.2f m/s going %0.2f deg\n',mean(vecnorm(kalman_res{select}.x(4:6,:),2)),rad2deg(atan2(mean(kalman_res{select}.x(4,:)),mean(kalman_res{select}.x(5,:)))))

if EKF_AW_PROPAGATE_OFFSET
figure;
ax1 = subplot(3,1,1);
plot(kalman_res{1}.t,kalman_res{1}.x(7,:)')
xlabel('Time [s]')
ylabel('offset_x')
ax2 = subplot(3,1,2);
plot(kalman_res{1}.t,kalman_res{1}.x(8,:)')
xlabel('Time [s]')
ylabel('offset_y')
ax3 = subplot(3,1,3);
plot(kalman_res{1}.t,kalman_res{1}.x(9,:)')
xlabel('Time [s]')
ylabel('offset_z')
linkaxes([ax1,ax2,ax3],'x')
end

figure;
filter_freq = 0.1; %[Hz]
[b,a] = butter(2,2*filter_freq*dt,'low');

ax1 = subplot(4,1,1);
plot(kalman_res{1}.t,filtfilt(b,a,kalman_res{1}.y(1:3,:)')')
grid on
xlabel('Time [s]')
ylabel('Innovation')
title('V_{gnd}')
legend('N','E','D')

ax2 = subplot(4,1,2);
plot(kalman_res{1}.t,filtfilt(b,a,kalman_res{1}.y(4:6,:)')')
grid on
xlabel('Time [s]')
ylabel('Innovation')
title('Accel')
legend('x','y','z')

ax3 = subplot(4,1,3);
plot(kalman_res{1}.t,filtfilt(b,a,kalman_res{1}.y(7,:)')')
grid on
xlabel('Time [s]')
ylabel('Innovation')
title('Pitot')

ax4 = subplot(4,1,4);
yyaxis left
plot(kalman_res{1}.t,rad2deg(kalman_res{1}.u(12,:)))
xlabel('Time [s]')
ylabel('Skew [deg]')
linkaxes([ax1,ax2,ax3,ax4],'x')

%% Plot covariance

P_temp = zeros(length(kalman_res{select}.P),length(kalman_res{select}.P{1}));
for k=1:length(kalman_res{select}.P)
    P_temp(k,:) = kalman_res{select}.P{k}(sub2ind(size(kalman_res{select}.P{k}),1:size(kalman_res{select}.P{k},1),1:size(kalman_res{select}.P{k},2))); %get diagonal elements
    R_temp(k,:) = kalman_res{select}.R{k}(sub2ind(size(kalman_res{select}.R{k}),1:size(kalman_res{select}.R{k},1),1:size(kalman_res{select}.R{k},2))); %get diagonal elements
    Q_temp(k,:) = kalman_res{select}.Q{k}(sub2ind(size(kalman_res{select}.Q{k}),1:size(kalman_res{select}.Q{k},1),1:size(kalman_res{select}.Q{k},2))); %get diagonal elements
end

figure
ax1 = subplot(2,1,1);
semilogy(kalman_res{select}.t,R_temp(:,[4:6]))
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
