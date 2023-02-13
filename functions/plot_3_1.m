function plot_3_2(IMU_accel,IMU_rate,V_XYZ,IMU_angle)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
figure
ax1 = subplot(3,2,1);
plot(IMU_accel.time,IMU_accel.data);
title('Accelerations')
xlabel('Time [s]')
ylabel('[m/s^2]')
legend('a_x','a_y','a_z')
grid on

ax2 = subplot(3,2,2);
plot(airspeed_pitot.time,airspeed_pitot.data);
title('Airspeed Pitot')
xlabel('Time [s]')
ylabel('[m/s]')
grid on

ax3 = subplot(3,2,3);
plot(IMU_rate.time,rad2deg(IMU_rate.data));
title('Angular Rates')
xlabel('Time [s]')
ylabel('[deg/s]')
legend('p','q','r')
grid on

ax4 = subplot(3,2,4);
plot(V_XYZ.time,V_XYZ.data);
title('Ground Velocity')
xlabel('Time [s]')
ylabel('[m/s]')
legend('V_N','V_E','V_D')
grid on

ax5 = subplot(3,2,5);
plot(IMU_angle.time,rad2deg(IMU_angle.data));
title('Euler Angles')
xlabel('Time [s]')
ylabel('[deg]')
legend('\phi','\theta','\psi')
grid on

ax6 = subplot(3,2,6);
plot(position_NED.time,position_NED.data)
title('Position')
xlabel('Time [s]')
ylabel('[m]')
legend('N','E','D')
grid on

linkaxes([ax1,ax2,ax3,ax4,ax5,ax6],'x')
end