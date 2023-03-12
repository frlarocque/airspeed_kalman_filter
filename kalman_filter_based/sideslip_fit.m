
%% Get beta and a_y
beta_est = beta_estimation_wind(Vg_NED.flight,IMU_angle.flight,wind,0);

% Filter Data coming in
filter_freq = 0.5; %[Hz]
[b,a] = butter(2,2*filter_freq*dt,'low');
a_y_filt = filtfilt(b,a,IMU_accel.flight.data(:,2));
pitot_filt_no_delay = filtfilt(b,a,airspeed_pitot.flight.data);
pitot_filt_with_delay = filter(b,a,airspeed_pitot.flight.data);

figure
ax1 = subplot(3,1,1);
plot(IMU_accel.flight.time,rad2deg(beta_est))
xlabel('Time [s]')
ylabel('\beta [deg]')
title('Sideslip Estimation using constant wind')
grid on

ax2 = subplot(3,1,2);
plot(IMU_accel.flight.time,a_y_filt)
xlabel('Time [s]')
ylabel('Filtered a_y [m/s^2]')
title('Side Acceleration')
grid on

ax3 = subplot(3,1,3);
plot(airspeed_pitot.flight.time,airspeed_pitot.flight.data)
xlabel('Time [s]')
ylabel('Airspeed u_b [m/s]')
title('Airspeed')
grid on

linkaxes([ax1,ax2,ax3],'x')

%% Fit

% beta = K.*(a_y/V^2)
% beta = K.*(a_y/V)
% beta = K.*(a_y)

% y  =  b * x
% ay =  K *

y = [a_y_filt];

X2  = [airspeed_pitot.flight.data.^2.*beta_est];
b2 = (X2'*X2)\X2'*y

X1  = [airspeed_pitot.flight.data.*beta_est];
b1 = (X1'*X1)\X1'*y

X0  = [beta_est];
b0 = (X0'*X0)\X0'*y

%%
col=linspecer(4);
figure
plot(IMU_accel.flight.time,rad2deg(beta_est),'color',col(1,:))
hold on
plot(IMU_accel.flight.time,rad2deg((a_y_filt./airspeed_pitot.flight.data.^2)./-2E-1),'--','color',col(2,:))
%plot(IMU_accel.flight.time,rad2deg((a_y_filt./airspeed_pitot.flight.data)./-2E0),'--','color',col(3,:))
%plot(IMU_accel.flight.time,rad2deg(a_y_filt./-2E1),'--','color',col(4,:))
%legend('Estimation','Quadratic','Linear','Constant')
xlabel('Time [s]')
ylabel('\beta estimation [deg]')
grid on
axis([-inf inf -180 180])
yline(-25)
yline(25)

%% Results
% Filter freq = 0.1 Hz
% b2 = -2E-1
% b1 = -1E0
% b0 = -2E0


