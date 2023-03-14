function x_axis_related_plot(IMU_accel,airspeed_pitot,pusher_prop_RPM,IMU_angle,low_pass_value,cut_condition)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Remove NAN
IMU_accel.data(isnan(IMU_accel.data))=0;
airspeed_pitot.data(isnan(airspeed_pitot.data))=0;
pusher_prop_RPM.data(isnan(pusher_prop_RPM.data))=0;
IMU_angle.data(isnan(IMU_angle.data))=0;

% Filter
dt = mean(diff(airspeed_pitot.time));
[b,a] = butter(2,2*low_pass_value*dt,'low');
IMU_accel.data = filtfilt(b,a,IMU_accel.data);
airspeed_pitot.data = filtfilt(b,a,airspeed_pitot.data);
pusher_prop_RPM.data = filtfilt(b,a,pusher_prop_RPM.data);
IMU_angle.data = filtfilt(b,a,IMU_angle.data);

if nargin==5
    cut_condition = [0 0];
end
figure

% Acceleration
ax1 = subplot(2,2,1);
plot(IMU_accel.time,IMU_accel.data(:,1));
% Plot Motor ON
if cut_condition(1)>IMU_accel.time(1) && cut_condition(2)<IMU_accel.time(end)
    hold on
    plot([cut_condition; cut_condition], repmat(ylim',1,length(cut_condition)), '--r');
end
title('Acceleration')
xlabel('Time [s]')
ylabel('[m/s^2]')
legend('a_x')
grid on

% Airspeed
ax2 = subplot(2,2,2);
plot(airspeed_pitot.time,airspeed_pitot.data);
% Plot Motor ON
if cut_condition(1)>airspeed_pitot.time(1) && cut_condition(2)<airspeed_pitot.time(end)
    hold on
    plot([cut_condition; cut_condition], repmat(ylim',1,length(cut_condition)), '--r');
end
title('Airspeed Pitot')
xlabel('Time [s]')
ylabel('[m/s]')
grid on

% Pusher command
ax3 = subplot(2,2,3);
plot(pusher_prop_RPM.time,pusher_prop_RPM.data);
% Plot Motor ON
if cut_condition(1)>pusher_prop_RPM.time(1) && cut_condition(2)<pusher_prop_RPM.time(end)
    hold on
    plot([cut_condition; cut_condition], repmat(ylim',1,length(cut_condition)), '--r');
end
title('Pusher Prop')
xlabel('Time [s]')
ylabel('RPM (revolution per minute)')
grid on

% Angle
ax4 = subplot(2,2,4);
plot(IMU_angle.time,rad2deg(IMU_angle.data(:,2)));
% Plot Motor ON
if cut_condition(1)>IMU_angle.time(1) && cut_condition(2)<IMU_angle.time(end)
    hold on
    plot([cut_condition; cut_condition], repmat(ylim',1,length(cut_condition)), '--r');
end
title('Euler Angles')
xlabel('Time [s]')
ylabel('[deg]')
legend('\theta')
grid on

linkaxes([ax1,ax2,ax3,ax4],'x')
end