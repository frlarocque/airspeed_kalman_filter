%%
clear all
close all
clc

% Add all paths
addpath('functions/');
%% Load data
clear all
close all
[file,path] = uigetfile({'*.mat'},'Select a file');

load(fullfile(path,file))

fprintf('OPENED: %s\n',file)

%% Assign
conditions = split(file,'_');
conditions = conditions{1};

if strcmp(conditions,'WINDTUNNEL')
    % Obtained directly
    IMU_accel.raw.data = [ac_data.EFF_FULL_INDI.body_accel_x,ac_data.EFF_FULL_INDI.body_accel_y,ac_data.EFF_FULL_INDI.body_accel_z];IMU_accel.raw.time =ac_data.EFF_FULL_INDI.timestamp;
    airspeed_pitot.raw.data = low_butter(ac_data.EFF_FULL_INDI.airspeed,0.4,1/mean(diff(ac_data.EFF_FULL_INDI.timestamp)),0,4);airspeed_pitot.raw.time = ac_data.EFF_FULL_INDI.timestamp;
    IMU_rate.raw.data = deg2rad([ac_data.EFF_FULL_INDI.angular_rate_p ac_data.EFF_FULL_INDI.angular_rate_q ac_data.EFF_FULL_INDI.angular_rate_r]);IMU_rate.raw.time = ac_data.EFF_FULL_INDI.timestamp;
    IMU_angle.raw.data = deg2rad([ac_data.EFF_FULL_INDI.phi_alt ac_data.EFF_FULL_INDI.theta_alt ac_data.EFF_FULL_INDI.psi_alt]);IMU_angle.raw.time = ac_data.EFF_FULL_INDI.timestamp;

    %Tunnel referential
    position_NED_tunnel.raw.data =[ac_data.EFF_FULL_INDI.position_N,ac_data.EFF_FULL_INDI.position_E,ac_data.EFF_FULL_INDI.position_D];position_NED_tunnel.raw.time = ac_data.EFF_FULL_INDI.timestamp;
    V_XYZ_tunnel.raw.data = [ac_data.EFF_FULL_INDI.speed_N ac_data.EFF_FULL_INDI.speed_E ac_data.EFF_FULL_INDI.speed_D]; V_XYZ_tunnel.raw.time = ac_data.EFF_FULL_INDI.timestamp;
    
    % As if were moving
    [airspeed_pitot.raw.data,airspeed_pitot.raw.time] = cut_resample(airspeed_pitot.raw.data,airspeed_pitot.raw.time,V_XYZ_tunnel.raw.time,[V_XYZ_tunnel.raw.time(1)+1,V_XYZ_tunnel.raw.time(end)-1]);
    [position_NED_tunnel.raw.data,position_NED_tunnel.raw.time] = cut_resample(position_NED_tunnel.raw.data,position_NED_tunnel.raw.time,V_XYZ_tunnel.raw.time,[V_XYZ_tunnel.raw.time(1)+1,V_XYZ_tunnel.raw.time(end)-1]);
    [V_XYZ_tunnel.raw.data,V_XYZ_tunnel.raw.time] = cut_resample(V_XYZ_tunnel.raw.data,V_XYZ_tunnel.raw.time,V_XYZ_tunnel.raw.time,[V_XYZ_tunnel.raw.time(1)+1,V_XYZ_tunnel.raw.time(end)-1]);
    
    % Model wind
    max_wind = 8; %[m/s]
    rate_wind = 0.1; %[m/s /s]
    wind.time = V_XYZ_tunnel.raw.time;
    wind.data = zeros(length(V_XYZ_tunnel.raw.data),1);
    for i=1:length(wind.data)
        if wind.time(i)>ac_data.motors_on(end-1)
            wind.data(i) = min((wind.time(i)-ac_data.motors_on(end-1))*rate_wind,max_wind);
        end
    end
    
    V_XYZ.raw.data = V_XYZ_tunnel.raw.data+[airspeed_pitot.raw.data zeros(length(airspeed_pitot.raw.data),2) ]+[wind.data zeros(length(wind.data),2) ];V_XYZ.raw.time = V_XYZ_tunnel.raw.time;
    position_NED.raw.data = position_NED_tunnel.raw.data+cumtrapz(V_XYZ.raw.time,V_XYZ.raw.data);position_NED.raw.time = position_NED_tunnel.raw.time;

    %%% GET DATA FROM WIND TUNNEL FOR OFFICIAL AIRSPEED if possible

elseif strcmp(conditions,'OUTDOOR')
    % Assign values
    position_NED.raw.data = [ac_data.ROTORCRAFT_FP.north_alt,ac_data.ROTORCRAFT_FP.east_alt,-ac_data.ROTORCRAFT_FP.up_alt]; position_NED.raw.time = ac_data.ROTORCRAFT_FP.timestamp;
    IMU_accel.raw.data = [ac_data.IMU_ACCEL_SCALED.ax_alt ac_data.IMU_ACCEL_SCALED.ay_alt ac_data.IMU_ACCEL_SCALED.az_alt]; IMU_accel.raw.time = ac_data.IMU_ACCEL_SCALED.timestamp;
    airspeed_pitot.raw.data = low_butter(ac_data.AIR_DATA.airspeed,0.4,1/mean(diff(ac_data.AIR_DATA.timestamp)),0,4); airspeed_pitot.raw.time=ac_data.AIR_DATA.timestamp;
    IMU_rate.raw.data = deg2rad([ac_data.IMU_GYRO_SCALED.gp_alt ac_data.IMU_GYRO_SCALED.gq_alt ac_data.IMU_GYRO_SCALED.gr_alt]); IMU_rate.raw.time = ac_data.IMU_GYRO_SCALED.timestamp;
    V_XYZ.raw.data = [ac_data.ROTORCRAFT_FP.vnorth_alt,ac_data.ROTORCRAFT_FP.veast_alt,-ac_data.ROTORCRAFT_FP.vup_alt];V_XYZ.raw.time=ac_data.ROTORCRAFT_FP.timestamp;
    IMU_angle.raw.data = deg2rad([ac_data.ROTORCRAFT_FP.phi_alt,ac_data.ROTORCRAFT_FP.theta_alt,ac_data.ROTORCRAFT_FP.psi_alt]);IMU_angle.raw.time=ac_data.ROTORCRAFT_FP.timestamp;
    
else
    fprintf('Wrong or No Condition')
end

%% Resample
% Resample Choice 
resample_time = airspeed_pitot.raw.time; %Airspeed has the lowest dt
cut_condition = [ac_data.motors_on(end-1),ac_data.motors_on(end)];

% Resampling
[IMU_accel.flight.data,IMU_accel.flight.time] = cut_resample(IMU_accel.raw.data,IMU_accel.raw.time,resample_time,cut_condition);
[airspeed_pitot.flight.data,airspeed_pitot.flight.time] = cut_resample(airspeed_pitot.raw.data,airspeed_pitot.raw.time,resample_time,cut_condition);
[IMU_rate.flight.data,IMU_rate.flight.time] = cut_resample(IMU_rate.raw.data,IMU_rate.raw.time,resample_time,cut_condition);
[V_XYZ.flight.data,V_XYZ.flight.time] = cut_resample(V_XYZ.raw.data,V_XYZ.raw.time,resample_time,cut_condition);
[IMU_angle.flight.data,IMU_angle.flight.time] = cut_resample(IMU_angle.raw.data,IMU_angle.raw.time,resample_time,cut_condition);
[position_NED.flight.data,position_NED.flight.time] = cut_resample(position_NED.raw.data,position_NED.raw.time,resample_time,cut_condition);

% Arbitrarly set so far
    % To add a square wave: +deg2rad(10*square(IMU_angle.flight.time./10))
alpha.flight.data = IMU_angle.flight.data(:,2);alpha.flight.time = IMU_angle.flight.time;
beta.flight.data = 0*ones(length(IMU_angle.flight.data),1);beta.flight.time = IMU_angle.flight.time;

%% Visualizing raw data
% Raw
plot_3_2(IMU_accel.raw,airspeed_pitot.raw,IMU_rate.raw,V_XYZ.raw,IMU_angle.raw,position_NED.raw,cut_condition)

%% Visualizing flight data
% Flight
plot_3_2(IMU_accel.flight,airspeed_pitot.flight,IMU_rate.flight,V_XYZ.flight,IMU_angle.flight,position_NED.flight)

%% Visualize Trajectory
trajectory(position_NED.flight,V_XYZ.flight,IMU_angle.flight,10)

%% Estimating wind
[wind,airspeed_estimation] = wind_estimation(V_XYZ.flight,IMU_angle.flight,airspeed_pitot.flight,alpha.flight,beta.flight);

fprintf('Estimated wind is %0.2f m/s going %0.2f deg\n',wind.norm,rad2deg(wind.direction))

figure
plot(wind.raw.time,wind.raw.data(:,[1 2]))
hold on
plot(wind.raw.time,ones(length(wind.raw.time),1)*wind.vect(:,[1:2]))
title('Wind Estimation')
xlabel('Time [s]')
ylabel('Wind Component [m/s]')
legend('N','E','Mean N','Mean E')
grid on

figure;
plot(airspeed_estimation.time,airspeed_estimation.data)
hold on
plot(airspeed_pitot.flight.time,airspeed_pitot.flight.data)
title('Airspeed Estimation Using Constant Wind')
xlabel('Time [s]')
ylabel('Airspeed [m/s]')
legend('Estimation','Measured')
grid on

error_airspeed = error_quantification(airspeed_estimation.data,airspeed_pitot.flight.data);

%2.2 m/s offset (20% offset) measured airspeed is 20% lower than estimated
%airspeed

%due to pitot not being inline with flow? (sideslip+angle of attack?)
%due to airspeed calculation error?
%due to 

%% Beta estimation using Wind
psi_gnd = atan2(V_XYZ.flight.data(:,2),V_XYZ.flight.data(:,1));
psi_a = atan2(V_XYZ.flight.data(:,2)-wind.vect(1),V_XYZ.flight.data(:,1)-wind.vect(2));
beta_est = psi_a-IMU_angle.flight.data(:,3);
beta_est = beta_est-ceil(beta_est/(2*pi)-0.5)*2*pi; %set on [-180,180] interval


beta.flight.data = beta_est;

figure
ax1 = subplot(2,1,1);
plot(V_XYZ.flight.time,rad2deg(IMU_angle.flight.data(:,3)))
hold on
plot(V_XYZ.flight.time,rad2deg(psi_gnd))
plot(V_XYZ.flight.time,rad2deg(psi_a))
legend('Psi','Psi Gnd','Psi Air')
xlabel('Time [s]')
ylabel('Angle [deg]')
grid on

ax2 = subplot(2,1,2);
plot(V_XYZ.flight.time,rad2deg(beta_est))
xlabel('Time [s]')
ylabel('Estimated Sideslip Angle [deg]')
grid on
linkaxes([ax1,ax2],'x')


%% Estimating Variance
recalculate_variance = false;
static_conditions = [100,130];%[50,60];

if recalculate_variance
    fprintf('Recalculated Variance\n')
    [IMU_accel.var,IMU_accel.static] = cut_variance(IMU_accel.raw.data,IMU_accel.raw.time,static_conditions);
    [IMU_rate.var,IMU_rate.static] = cut_variance(IMU_rate.raw.data,IMU_rate.raw.time,static_conditions);
    [V_XYZ.var,V_XYZ.static] = cut_variance(V_XYZ.raw.data,V_XYZ.raw.time,static_conditions);
    
    figure
    ax1 = subplot(3,1,1);
    plot(IMU_accel.static.time,IMU_accel.static.data)
    title('Accelerations')
    xlabel('Time [s]')
    ylabel('[m/s^2]')
    legend('a_x','a_y','a_z')
    grid on
    
    ax2 = subplot(3,1,2);
    plot(IMU_rate.static.time,IMU_rate.static.data)
    title('Angular Rates')
    xlabel('Time [s]')
    ylabel('[deg/s]')
    legend('p','q','r')
    grid on
    
    ax3 = subplot(3,1,3);
    plot(V_XYZ.static.time,V_XYZ.static.data)
    title('Ground Velocity')
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('V_N','V_E','V_D')
    grid on
    
    linkaxes([ax1,ax2,ax3],'x')
else
    fprintf('Used Existing Variance\n')
    IMU_accel.var =[0.0087 0.0011 0.0011]; %V3
    IMU_rate.var =1.0e-03 *[0.0664    0.1253    0.1044]; %V3
    V_XYZ.var =1.0e-04 * [0.2220    0.2786    0.1342]; %V3
end

%% Visualize data going in Kalman
plot_2_2(IMU_accel.flight,IMU_rate.flight,V_XYZ.flight,IMU_angle.flight)

%% Kalman Filter Without euler angle estimation
k_wind_var =  [7.8E-8, 1.43E-6]; %1.43E-6 %7.8E-8; logspace(-9,-3,20);
kalman_res = {};

for i=1:length(k_wind_var)

% 𝑢, 𝑣, 𝑤 = Velocity in the Body Frame
% 𝜇𝑥, 𝜇𝑦, 𝜇𝑧 = Wind Velocity in Earth Fixed Frame
% 𝑝, 𝑞, 𝑟 = Angular Rates Measured by Gyroscopes
% 𝑎x, 𝑎𝑦, 𝑎𝑧 = Accelerations Measured by Accelerometers
% 𝑉𝑥 , 𝑉𝑦, 𝑉𝑧 = Velocity in Earth Fixed Frame


%x = [u v w phi theta psi mu_x mu_y mu_z];
%u = [a_x a_y a_z p q r];
%y = [V_x V_y V_z alpha beta];

%w_w = [0;0;0]; %noise angular rate
%w_a = [0;0;0]; %noise accelerations
%w_mu =[0;0;0]; %noise wind

%x = [u v w mu_x mu_y mu_z];
%u = [a_x a_y a_z p q r phi theta psi];
%y = [V_x V_y V_z alpha beta];

clear x_list u_list y_list z_list P K

%x = [u    v  w  mu_x mu_y mu_z]
x_0 =  [0, 0, 0,  0,  0,   0]';

%u = [a_x a_y a_z     p  q r phi theta psi];
u_0 =  [0   0   -9.81   0  0 0 0 0 0]';

%y = [V_x V_y V_z alpha beta];
y_0 = [0,0,0,0,0]';

t = airspeed_pitot.flight.time;
dt = mean(t(2:end)-t(1:end-1));

wind_var = [1 1 1]*k_wind_var(i);
Q = diag([IMU_accel.var,wind_var]); %process noise
R = diag([V_XYZ.var 1E-6 1E-6]); %measurement noise
P = {diag([1 1 1 wind_var])}; %covariance
K = {};

x_list(:,1) = [0 0 0 0 0 0]';
u_list = [IMU_accel.flight.data IMU_rate.flight.data IMU_angle.flight.data]';
y_list = zeros(length(y_0),length(t));
z_list = [V_XYZ.flight.data alpha.flight.data beta.flight.data]'; %measurement

epsi = 1E-2;

for k=1:length(t)
    x=x_list(:,k);
    u=u_list(:,k);
    z=z_list(:,k);

    F_val = F_2(x,u,epsi);
    G_val = G_2(x,u,epsi);

    % Prediction
    x_pred = x + dt*f_2(x,u);   
    P_pred = F_val*P{k}*F_val'+Q;

    % Update
    y_list(:,k+1) = z-g_2(x,u);

    %Kalman gain
    K{k} = P_pred*G_val'*inv(G_val*P_pred*G_val'+R);

    x_list(:,k+1) = x_pred+K{k}*y_list(:,k+1);
    P{k+1} = (eye(length(x_0))-K{k}*G_val)*P_pred;

end
kalman_res{i}.error = error_quantification(x_list(1,1:end-1)',airspeed_pitot.flight.data);
kalman_res{i}.t = t;
kalman_res{i}.x = x_list;
kalman_res{i}.u = u_list;
kalman_res{i}.z = z_list;
kalman_res{i}.P = P;
kalman_res{i}.Q = Q;
kalman_res{i}.R = R;
kalman_res{i}.K = K;

end
%% Analysis of Results
figure
for i=1:length(kalman_res)
    semilogx(k_wind_var(i),kalman_res{i}.error.error_RMS,'*','MarkerSize',10)
    hold on
    grid on
end
xlabel('Wind Covariance')
ylabel('Airspeed RMS Error [m/s]')
grid on

%% Plot
select = 1; %5 7 17

figure
ax1=subplot(3,1,1);
plot(airspeed_estimation.time,airspeed_estimation.data)
hold on
plot(kalman_res{select}.t,kalman_res{select}.x(1,1:end-1));
plot(airspeed_pitot.flight.time,airspeed_pitot.flight.data)
title('Airspeed Estimation Kalman')
xlabel('Time [s]')
ylabel('Speed [m/s]')
legend('Estimation Knowing wind','Kalman Estimation','Measured Airspeed')
grid on

ax2 = subplot(3,1,2);
plot(kalman_res{select}.t,kalman_res{select}.x([4:6],1:end-1));
hold on
plot(wind.raw.time,ones(length(wind.raw.time),1)*wind.vect(:,[1:2]),'--')
%plot(wind.raw.time,wind.raw.data(:,[1:2]),'--')
title('Wind mu')
xlabel('Time [s]')
ylabel('Speed [m/s]')
legend('Kalman N','Kalman E','Kalman D','Mean Real N','Mean Real E')
%legend('Kalman N','Kalman E','Kalman D','Mean Real N','Mean Real E','Real N','Real E')
grid on

ax3 = subplot(3,1,3);
plot(kalman_res{select}.t,kalman_res{select}.x([1:3],1:end-1));
title('Velocity Body Axis')
xlabel('Time [s]')
ylabel('Speed [m/s]')
legend('u_b','v_b','w_b')
grid on

linkaxes([ax1,ax2,ax3],'x')

sgtitle(sprintf('Wind Covariance %.1d | RMS error %.2f',k_wind_var(select),kalman_res{select}.error.error_RMS))

%% Plot covariance
select = 1;

P_temp = zeros(length(kalman_res{select}.P),length(kalman_res{select}.P{1}));
for k=1:length(kalman_res{select}.P)
    P_temp(k,:) = kalman_res{select}.P{k}(sub2ind(size(kalman_res{select}.P{k}),1:size(kalman_res{select}.P{k},1),1:size(kalman_res{select}.P{k},2))); %get diagonal elements
end

figure
ax1 = subplot(2,1,1);
semilogy(kalman_res{select}.t,P_temp([1:end-1],[1:3]))
xlabel('Time [s]')
ylabel('Covariance')
title('Body Velocity')
legend('u','v','w')
grid on

ax2 = subplot(2,1,2);
semilogy(kalman_res{select}.t,P_temp([1:end-1],[4:6]))
xlabel('Time [s]')
ylabel('Covariance')
title('Wind Velocity')
legend('mu_x','mu_y','mu_z')
grid on

linkaxes([ax1,ax2],'x')
sgtitle(sprintf('Wind Covariance %.1d | RMS error %.2f',k_wind_var(select),kalman_res{select}.error.error_RMS))
