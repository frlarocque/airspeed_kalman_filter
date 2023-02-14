function plot_3_2(IMU_accel,airspeed_pitot,IMU_rate,Vg_NED,IMU_angle,position_NED,cut_condition)
%PLOT_3_2 Plots main states of the vehicle in a 3x2 configuration
%
% Inputs:
%           -IMU_accel: struct with accelerations in body frame [m/s^2]
%           -airspeed_pitot: struct with pitot airspeed (u_b) [m/s]
%           -IMU_rate: struct with angular rates in body frame (p,q,r) [rad/s]
%           -Vg_NED: struct with ground velocity in NED frame [m/s]
%           -IMU_angle: struct with euleur angles [rad]
%           -position_NED: struct with position in NED frame [m]
%           -cut_condition: matrix with engine stop and start [s]

% Support case where no cut_condition is provided
if nargin==6
    cut_condition = [0 0];
end
figure

% Acceleration
ax1 = subplot(3,2,1);
plot(IMU_accel.time,IMU_accel.data);
% Plot Motor ON
if cut_condition(1)>IMU_accel.time(1) && cut_condition(2)<IMU_accel.time(end)
    hold on
    plot([cut_condition; cut_condition], repmat(ylim',1,length(cut_condition)), '--r');
end
title('Accelerations')
xlabel('Time [s]')
ylabel('[m/s^2]')
legend('a_x','a_y','a_z')
grid on

% Airspeed
ax2 = subplot(3,2,2);
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

% Angular Rate
ax3 = subplot(3,2,3);
plot(IMU_rate.time,rad2deg(IMU_rate.data));
% Plot Motor ON
if cut_condition(1)>IMU_rate.time(1) && cut_condition(2)<IMU_rate.time(end)
    hold on
    plot([cut_condition; cut_condition], repmat(ylim',1,length(cut_condition)), '--r');
end
title('Angular Rates')
xlabel('Time [s]')
ylabel('[deg/s]')
legend('p','q','r')
grid on

% Velocity
ax4 = subplot(3,2,4);
plot(Vg_NED.time,Vg_NED.data);
% Plot Motor ON
if cut_condition(1)>Vg_NED.time(1) && cut_condition(2)<Vg_NED.time(end)
    hold on
    plot([cut_condition; cut_condition], repmat(ylim',1,length(cut_condition)), '--r');
end
title('Ground Velocity')
xlabel('Time [s]')
ylabel('[m/s]')
legend('V_N','V_E','V_D')
grid on

% Angle
ax5 = subplot(3,2,5);
%yyaxis left
%plot(IMU_angle.time,rad2deg(IMU_angle.data(:,[1:2])));
%ylabel('[deg]')
%yyaxis right
%plot(IMU_angle.time,rad2deg(IMU_angle.data(:,3)));
plot(IMU_angle.time,rad2deg(IMU_angle.data));
% Plot Motor ON
if cut_condition(1)>IMU_angle.time(1) && cut_condition(2)<IMU_angle.time(end)
    hold on
    plot([cut_condition; cut_condition], repmat(ylim',1,length(cut_condition)), '--r');
end
title('Euler Angles')
xlabel('Time [s]')
ylabel('[deg]')
legend('\phi','\theta','\psi')
grid on

%Position
ax6 = subplot(3,2,6);
plot(position_NED.time,position_NED.data)
% Plot Motor ON
if cut_condition(1)>position_NED.time(1) && cut_condition(2)<position_NED.time(end)
    hold on
    plot([cut_condition; cut_condition], repmat(ylim',1,length(cut_condition)), '--r');
end
title('Position')
xlabel('Time [s]')
ylabel('[m]')
legend('N','E','D')
grid on

linkaxes([ax1,ax2,ax3,ax4,ax5,ax6],'x')
end