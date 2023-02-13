function [wind,airspeed_estimation] = wind_estimation(V_XYZ,IMU_angle,airspeed_pitot,alpha,beta)

V_t = airspeed_pitot.data./(cos(alpha.data).*cos(beta.data));
v = sin(beta.data).*V_t;
w = V_t.*sin(alpha.data).*cos(beta.data);
u = airspeed_pitot.data;

V_airspeed_NED = [];
for i=1:length(airspeed_pitot.data)
    V_airspeed_NED(i,:) = [DCM(IMU_angle.data(i,1),IMU_angle.data(i,2),IMU_angle.data(i,3))*[u(i) v(i) w(i)]']';
end
%V_temp_airspeed = [airspeed_pitot.*cos(deg2rad(IMU_angle(:,3))), airspeed_pitot.*sin(deg2rad(IMU_angle(:,3)))];

%V_airspeed_XYZ_filt = low_butter(V_XYZ.data,0.2,1/0.02,0,4);
% [theta_rad,theta_t]=offset_sinus_addition(airspeed_pitot.time,V_XYZ.data(:,1),V_airspeed_NED(:,1),0,1)
%

wind.raw.data = V_XYZ.data-V_airspeed_NED; wind.raw.time=airspeed_pitot.time;
wind.vect = mean(wind.raw.data);
wind.norm = norm(wind.vect);
wind.direction = atan2(wind.vect(2),wind.vect(1));

v_g = sqrt(V_XYZ.data(:,1).^2+V_XYZ.data(:,2).^2);
airspeed_estimation.data = sqrt(wind.norm.^2+v_g.^2-2.*v_g.*wind.norm.*cos(wind.direction-IMU_angle.data(:,3)));
airspeed_estimation.time = V_XYZ.time;

end