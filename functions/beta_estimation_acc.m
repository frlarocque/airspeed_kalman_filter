function [beta] = beta_estimation_acc(m,accel_y,V_pitot,degree,graph)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
%
% Fy = (beta [rad] *-2.194341022544540*1E-1)* V [m/s]^2
% RMS = 1.2

% Fy = (beta [rad] *-2.552098952269741*1E0)* V [m/s]
% RMS = 4.8

% Fy = (beta [rad] *-2.226114526713827*1E1)
% RMS = 10.7
t=[];
if isstruct(accel_y) && isstruct(V_pitot)
    t = accel_y.time;
    accel_y = accel_y.data;
    V_pitot = V_pitot.data;
end

k_beta = [-2.226114526713827*1E1   -2.552098952269741*1E0  -2.194341022544540*1E-1];

beta = ((accel_y.*m)./(V_pitot.^degree))./k_beta(degree+1);

% Beta set on [-pi,pi] interval
beta = beta-ceil(beta/(2*pi)-0.5)*2*pi; 

if graph && ~isempty(t)
    figure
    plot(t,rad2deg(beta))

    xlabel('Time [s]')
    ylabel('Angle [deg]')
    grid on
end

end