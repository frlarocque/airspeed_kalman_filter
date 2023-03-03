function [beta_est] = beta_estimation_wind(Vg_NED,IMU_angle,wind,graph)
%BETA_ESTIMATION Using ground speed, heading and wind, estimate sideslip
%angle
%
% Inputs: 
%           -Vg_NED: struct with ground velocity in NED frame [m/s]
%           -IMU_angle: struct with euleur angles [rad]
%           -wind: struct with wind information [m/s, rad]
%
% Output:
%           -beta: struct of sideslip angle [rad]

% Calculate ground track, air track
psi_gnd = atan2(Vg_NED.data(:,2),Vg_NED.data(:,1));
psi_a = atan2(Vg_NED.data(:,2)-wind.vect(2),Vg_NED.data(:,1)-wind.vect(1));
beta_est = psi_a-IMU_angle.data(:,3);

% Beta set on [-pi,pi] interval
beta_est = beta_est-ceil(beta_est/(2*pi)-0.5)*2*pi; 

if graph
    figure
    ax1 = subplot(2,1,1);
    plot(Vg_NED.time,rad2deg(IMU_angle.data(:,3)))
    hold on
    plot(Vg_NED.time,rad2deg(psi_gnd))
    plot(Vg_NED.time,rad2deg(psi_a))
    legend('Psi','Psi Gnd','Psi Air')
    xlabel('Time [s]')
    ylabel('Angle [deg]')
    grid on
    
    ax2 = subplot(2,1,2);
    plot(Vg_NED.time,rad2deg(beta_est))
    xlabel('Time [s]')
    ylabel('Estimated Sideslip Angle [deg]')
    grid on
    linkaxes([ax1,ax2],'x')
end

end