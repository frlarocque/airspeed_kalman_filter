function plot_2_2(IMU_accel,IMU_rate,Vg_NED,IMU_angle)
%PLOT_2_2 Plots main states of the vehicle in a 2x2 configuration
%
% Inputs:
%           -IMU_accel: struct with accelerations in body frame [m/s^2]
%           -IMU_rate: struct with angular rates in body frame (p,q,r) [rad/s]
%           -Vg_NED: struct with ground velocity in NED frame [m/s]
%           -IMU_angle: struct with euleur angles [rad]

% Acceleration
figure
ax1 = subplot(2,2,1);
plot(IMU_accel.time,IMU_accel.data);
title('Accelerations')
xlabel('Time [s]')
ylabel('[m/s^2]')
legend('a_x','a_y','a_z')
grid on

% Angular Rate
ax2 = subplot(2,2,2);
plot(IMU_rate.time,rad2deg(IMU_rate.data));
title('Angular Rates')
xlabel('Time [s]')
ylabel('[deg/s]')
legend('p','q','r')
grid on

% Velocity in NED Frame
ax3 = subplot(2,2,3);
plot(Vg_NED.time,Vg_NED.data);
title('Ground Velocity')
xlabel('Time [s]')
ylabel('[m/s]')
legend('V_N','V_E','V_D')
grid on

% IMU Angle
ax4 = subplot(2,2,4);
plot(IMU_angle.time,rad2deg(IMU_angle.data));
title('Euler Angles')
xlabel('Time [s]')
ylabel('[deg]')
legend('\phi','\theta','\psi')
grid on

linkaxes([ax1,ax2,ax3,ax4],'x')
end