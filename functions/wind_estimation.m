function [wind,airspeed_estimation] = wind_estimation(Vg_NED,IMU_angle,airspeed_pitot,alpha,beta)
%WIND_ESTIMATION Estimates wind using pitot tube airspeed and GPS airspeed
%
% Inputs:
%           -Vg_NED: struct of GPS NED ground velocity [m/s]
%           -IMU_angle: struct of Euler Angles [rad]
%           -airspeed_pitot: struct of pitot airspeed [m/s]
%           -alpha: struct of angle of attack [rad]
%           -beta: struct of sideslip angle [rad]
%
% Outputs:
%           -wind: struct with wind information [m/s, rad]
%           -airspeed_estimation: struct with airspeed (not pitot tube 
%               airspeed) estimated using
%           constant wind assumption [m/s]

% Decomposing airspeed into body components

V_t = airspeed_pitot.data./(cos(alpha.data).*cos(beta.data));
v_b = sin(beta.data).*V_t;
w_b = V_t.*sin(alpha.data).*cos(beta.data);
u_b = airspeed_pitot.data;

% Changing reference frame from body to NED
Va_NED = [];
for i=1:length(airspeed_pitot.data)
    Va_NED(i,:) = [DCM(IMU_angle.data(i,1),IMU_angle.data(i,2),IMU_angle.data(i,3))*[u_b(i) v_b(i) w_b(i)]']';
end

% Calculate wind using wind triangle relationship
wind.raw.data = Vg_NED.data-Va_NED; wind.raw.time=airspeed_pitot.time;
wind.vect = mean(wind.raw.data);
wind.norm = norm(wind.vect);
wind.direction = atan2(wind.vect(2),wind.vect(1));

% Estimate airspeed using constant wind assumption
v_g = sqrt(Vg_NED.data(:,1).^2+Vg_NED.data(:,2).^2);
airspeed_estimation.data = sqrt(wind.norm.^2+v_g.^2-2.*v_g.*wind.norm.*cos(wind.direction-IMU_angle.data(:,3)));
airspeed_estimation.time = Vg_NED.time;
end