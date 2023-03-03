function [alpha_est] = alpha_estimation_wind(V_pitot,Vg_NED,IMU_angle,wind,graph)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
sat = deg2rad(60);

gamma = min(max(real(asin((-Vg_NED.data(:,3)-wind.vect(3))./(vecnorm([Vg_NED.data+wind.vect]')'))),-sat),sat);
gamma(V_pitot.data<5) = 0;
alpha_est = IMU_angle.data(:,2)-gamma;

if graph
    figure
    plot(IMU_angle.time,rad2deg(IMU_angle.data(:,2)))
    hold on
    plot(IMU_angle.time,rad2deg(gamma))
    plot(IMU_angle.time,rad2deg(alpha_est))

    legend('\theta','\gamma',['\alpha_{est}'])
    xlabel('Time [s]')
    ylabel('Angle [deg]')
    grid on

end