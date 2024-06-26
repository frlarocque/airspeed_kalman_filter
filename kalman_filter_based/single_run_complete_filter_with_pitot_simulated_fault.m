%%
clear all
close all
clc

format long
% Add all paths
addpath('/home/frederic/Documents/thesis/tools/airspeed_estimation/functions/');
%% Load single filedata
clear ac_data
clear hover_prop_rpm
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

f_EKF = 25;
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

%% Fault injection
t_start = 100; %[s]
ramp = -2.5; %[m/s /s]

for i=1:length(z_list_resampled)
    if i*dt>t_start
        z_list_resampled(7,i) = z_list_resampled(7,find(t-t(1)>t_start,1,'first')) + ramp * (i*dt-t_start);
    end
end


%% Run filter
kalman_res = {};
[EKF_res] = run_EKF(epsi,t,Q,R,P_0,x_0,u_list_resampled,z_list_resampled,f_fh,g_fh,true);

kalman_res{1} = EKF_res;

kalman_res{1}.error = error_quantification_full(kalman_res{1}.x(1,:)',airspeed_pitot_resampled.flight.data,airspeed_pitot_resampled.flight.valid,kalman_res{1}.u(12,:)');
kalman_res{1}.error.constant_wind = error_quantification(kalman_res{1}.x(1,:)',interp1(airspeed_estimation.time,airspeed_estimation.data,kalman_res{1}.t));

fprintf("FINISHED!\n \nWAKE UP!\n")

%% Plot
select = 1;
%plot_EKF_result(kalman_res{select},airspeed_pitot.flight,wind)
plot_EKF_result_full(kalman_res{select},airspeed_pitot.flight,beta.flight,alpha.flight,wind,0.5)
fprintf('Estimated wind (using Kalman Filter) is %0.2f m/s going %0.2f deg\n',mean(vecnorm(kalman_res{select}.x(4:6,:),2)),rad2deg(atan2(mean(kalman_res{select}.x(4,:)),mean(kalman_res{select}.x(5,:)))))
fprintf('Error RMS Overall %2.2f Hover %2.2f Transition %2.2f FF %2.2f\n',kalman_res{1}.error.valid_pitot.error_RMS,kalman_res{1}.error.hover.error_RMS,kalman_res{1}.error.transition.error_RMS,kalman_res{1}.error.ff.error_RMS)

%%
figure;
ax1 = subplot(2,1,1);
plot(kalman_res{select}.t,vecnorm(kalman_res{select}.x([4:5],:)))
xlabel('Time [s]')
ylabel('Wind Magnitude')
ax2 = subplot(2,1,2);
plot(kalman_res{select}.t,rad2deg(atan2(kalman_res{select}.x(4,:),kalman_res{select}.x(5,:))))
xlabel('Time [s]')
ylabel('Wind Direction')
linkaxes([ax1,ax2],'x')

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
ax1=subplot(4,1,1);
plot(kalman_res{1}.t,kalman_res{1}.y(1:3,:)')
grid on
xlabel('Time [s]')
ylabel('Innovation')
title('V_{gnd}')
legend('N','E','D')

ax2=subplot(4,1,2);
plot(kalman_res{1}.t,kalman_res{1}.y(4:6,:)')
grid on
xlabel('Time [s]')
ylabel('Innovation')
title('Accel')
legend('x','y','z')

ax3=subplot(4,1,3);
plot(kalman_res{1}.t,kalman_res{1}.y(7,:)')
grid on
xlabel('Time [s]')
ylabel('Innovation')
title('Pitot')

ax4=subplot(4,1,4);
yyaxis left
plot(kalman_res{1}.t,rad2deg(kalman_res{1}.u(12,:)))
xlabel('Time [s]')
ylabel('Skew [deg]')
linkaxes([ax1,ax2,ax3,ax4],'x')

figure;
plot(kalman_res{1}.t,kalman_res{1}.innov_gates)
legend({'GPS_x','GPS_y','GPS_z','a_x','a_y','a_z','V_{pitot}'})

% Residuals histogram
residual_hist(kalman_res{1}.y',100,dt,1,2)

%% Plot covariance

P_temp = zeros(length(kalman_res{select}.P),length(kalman_res{select}.P{1}));
for k=1:length(kalman_res{select}.P)
    P_temp(k,:) = kalman_res{select}.P{k}(sub2ind(size(kalman_res{select}.P{k}),1:size(kalman_res{select}.P{k},1),1:size(kalman_res{select}.P{k},2))); %get diagonal elements
    R_temp(k,:) = kalman_res{select}.R{k}(sub2ind(size(kalman_res{select}.R{k}),1:size(kalman_res{select}.R{k},1),1:size(kalman_res{select}.R{k},2))); %get diagonal elements
    Q_temp(k,:) = kalman_res{select}.Q{k}(sub2ind(size(kalman_res{select}.Q{k}),1:size(kalman_res{select}.Q{k},1),1:size(kalman_res{select}.Q{k},2))); %get diagonal elements
end

figure
ax1 = subplot(2,1,1);
semilogy(kalman_res{select}.t,R_temp(:,4),'-')
hold on
semilogy(kalman_res{select}.t,R_temp(:,5),'--')
semilogy(kalman_res{select}.t,R_temp(:,6),':')
legend('a_x','a_y','a_z')
ax2 = subplot(2,1,2);
semilogy(kalman_res{select}.t,Q_temp(:,7),'-')
hold on;
semilogy(kalman_res{select}.t,Q_temp(:,8),'--')
semilogy(kalman_res{select}.t,Q_temp(:,9),':')
legend('mu_x','mu_y','mu_z')

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
