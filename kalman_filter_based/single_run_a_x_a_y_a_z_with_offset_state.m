%%
clear all
close all
clc

format long
% Add all paths
addpath('/home/frederic/Documents/thesis/tools/airspeed_estimation/functions/');

%% Load single filedata
[file,path] = uigetfile({'*.mat'},'Select a file');

load(fullfile(path,file))

%% Setup
% Setup Options
graph = 0;
beta_est = 1;
alpha_est = 1;
recalculate_variance = false;
pitot_correction = 1.0;

% Run setup
wind_triangle_setup

%% Kalman Filter Without Euler angle estimation
% 𝑢, 𝑣, 𝑤 = Velocity in the Body Frame
% 𝜇𝑥, 𝜇𝑦, 𝜇𝑧 = Wind Velocity in Earth Fixed Frame
% 𝑝, 𝑞, 𝑟 = Angular Rates Measured by Gyroscopes
% 𝑎x, 𝑎𝑦, 𝑎𝑧 = Accelerations Measured by Accelerometers
% 𝑉𝑥 , 𝑉𝑦, 𝑉𝑧 = Velocity in Earth Fixed Frame
% alpha, beta = angle of attack and sideslip angle

%x = [u v w mu_x mu_y mu_z k_x k_y k_z];
%u = [a_x a_y a_z p q r phi theta psi];
%z = [V_x V_y V_z beta];

%w_w = [0;0;0]; %noise angular rate
%w_a = [0;0;0]; %noise accelerations
%w_mu =[0;0;0]; %noise wind

epsi = 1E-2;

t = airspeed_pitot.flight.time;
dt = mean(diff(t));

f_fh = str2func('f_4');
g_fh = str2func('g_11_ident');

% Get filter accel agressively
filter_freq = 0.2; %[Hz]
[b,a] = butter(2,2*filter_freq*dt,'low');

a_x_filt = filter(b,a,IMU_accel.flight.data(:,1));
a_y_filt = filter(b,a,IMU_accel.flight.data(:,2));
a_z_filt = filter(b,a,IMU_accel.flight.data(:,3));
pusher_prop_rpm_filt = filter(b,a,pusher_prop_rpm.flight.data);
hover_prop_rpm_filt = filter(b,a,mean(hover_prop_rpm.flight.data,2));
skew_filt = filter(b,a,skew.flight.data);
elevator_pprz_filt = filter(b,a,control_surface_pprz.flight.data(:,4));

% Initial conditions
x_0 = [0 0 0   -3.9368 -4.345 0   -0.45 0 0]';

u_list = [IMU_accel.flight.data IMU_rate.flight.data IMU_angle.flight.data ...
            pusher_prop_rpm_filt hover_prop_rpm_filt skew_filt elevator_pprz_filt]';
z_list = [Vg_NED.flight.data a_x_filt a_y_filt a_z_filt]'; %measurement

% Filter Data coming in
filter_freq = 10.0; %[Hz]
[b,a] = butter(2,2*filter_freq*dt,'low');
u_list(1:6,:) = filter(b,a,u_list(1:6,:),[],2);
z_list(1:3,:) = filter(b,a,z_list(1:3,:),[],2);

cov_list = 7.8E-10; %7.8E-7;%1.43E-6;
kalman_res = {};

wind_var = [1 1 1E-1].*cov_list(1);
offset_var = [1 1 1].*1E-5;
Q = diag([IMU_accel.var,IMU_rate.var,wind_var,offset_var]); %process noise
P_0 = diag([1E-2 1E-2 1E-2 1E1.*wind_var offset_var]); %covariance
R = diag([Vg_NED.var 1E-1.*IMU_accel.var(1) IMU_accel.var(2) 1E1.*IMU_accel.var(3)]); %measurement noise

%%
EKF_res = {};

x_list = zeros(size(x_0,1),length(t));
y_list = zeros(size(z_list,1),length(z_list));

K = cell(1,length(t));
P = cell(1,length(t));
R_variable = cell(1,length(t));
Q_variable = cell(1,length(t));

for k=1:length(t)
    if k==1
        x = x_0;
        P_last = P_0;
    else
        x=x_list(:,k-1);
        P_last = P{k-1};
    end
        
    Q_variable{k} = Q;

    if t(k)-t(1)<10
        Q_variable{k}(7,7) = 1E1*Q(7,7);
        Q_variable{k}(8,8) = 1E1*Q(8,8);
        Q_variable{k}(9,9) = 1E1*Q(9,9);
    end

    u=u_list(:,k);
    z=z_list(:,k);
    
    R_variable{k} = R;

    %if abs(asin(x(2)./vecnorm(x(1:3))))> deg2rad(20)
    %    R_variable{k}(4,4) = 1E2.*R_variable{k}(4,4);
    %end

    F_val = F(f_fh,x,u,epsi);
    G_val = G(g_fh,x,u,epsi);
    L_val = L(f_fh,x,u,epsi);
    M_val = M(g_fh,x,u,epsi);

    % Prediction
    x_pred = x + dt*f_fh(x,u);      
    P_pred = F_val*P_last*F_val'+L_val*Q_variable{k}*L_val';

    % Update
    y_list(:,k) = z-g_fh(x,u);

    %Kalman gain
    K{k} = P_pred*G_val'*inv(G_val*P_pred*G_val'+M_val*R_variable{k}*M_val');

    x_list(:,k) = x_pred+K{k}*y_list(:,k);
    P{k} = (eye(length(x))-K{k}*G_val)*P_pred;

end

EKF_res.t = t;
EKF_res.x = x_list;
EKF_res.y = y_list;
EKF_res.u = u_list;
EKF_res.z = z_list;
EKF_res.P = P;
EKF_res.Q = Q_variable;
EKF_res.R = R_variable;
EKF_res.K = K;

kalman_res{1} = EKF_res;

kalman_res{1}.error = error_quantification(kalman_res{1}.x(1,airspeed_pitot.flight.valid)',airspeed_pitot.flight.data(airspeed_pitot.flight.valid));

%% Plot
select = 1;
%plot_EKF_result(kalman_res{select},airspeed_pitot.flight,wind)
plot_EKF_result_full(kalman_res{select},airspeed_pitot.flight,beta.flight,alpha.flight,wind)
fprintf('Estimated wind (using Kalman Filter) is %0.2f m/s going %0.2f deg\n',mean(vecnorm(kalman_res{select}.x(4:6,:),2)),rad2deg(atan2(mean(kalman_res{select}.x(4,:)),mean(kalman_res{select}.x(5,:)))))

%% Plot covariance

P_temp = zeros(length(kalman_res{select}.P),length(kalman_res{select}.P{1}));
for k=1:length(kalman_res{select}.P)
    P_temp(k,:) = kalman_res{select}.P{k}(sub2ind(size(kalman_res{select}.P{k}),1:size(kalman_res{select}.P{k},1),1:size(kalman_res{select}.P{k},2))); %get diagonal elements
end

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