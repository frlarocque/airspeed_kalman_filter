function [real_skew] = measSkew2Real(meas_skew)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% In rad

meas2real = [1.139065574081584   0.049376822710408];

real_skew = polyval(meas2real,meas_skew);

real_skew = max([real_skew zeros(size(real_skew))]')';
real_skew = min([real_skew ones(size(real_skew)).*deg2rad(90)]')';


% sys = tf([1],[1.75 1]);
% t_temp = linspace(temp_ac_data.EFF_FULL_INDI.timestamp(1),temp_ac_data.EFF_FULL_INDI.timestamp(end),length(temp_ac_data.EFF_FULL_INDI.timestamp));
% y = lsim(sys,temp_ac_data.EFF_FULL_INDI.wing_angle_deg_sp,t_temp);
% 
% figure;
% plot(temp_ac_data.EFF_FULL_INDI.wing_angle_deg)
% hold on
% plot(temp_ac_data.EFF_FULL_INDI.wing_angle_deg_sp)
% plot(y)
% plot(rad2deg(measSkew2Real(deg2rad(temp_ac_data.EFF_FULL_INDI.wing_angle_deg))))
% plot(polyval(meas2real,temp_ac_data.EFF_FULL_INDI.wing_angle_deg))
% 
% real = deg2rad([90 0 30 60 70 70]);
% meas = deg2rad([76 -5 23 53 66 65]);
% meas2real = polyfit(meas,real,1);
% meas2real = polyfit(temp_ac_data.EFF_FULL_INDI.wing_angle_deg,y,1)


end