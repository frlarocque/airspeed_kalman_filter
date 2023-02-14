function [wind] = circle_wind_estimation(Vg_NED,cut_condition)
%CIRCLE_WIND_ESTIMATION Estimates wind using ground velocity when traveling
%at constant TAS
% 
% Based on : A  no-flow-sensor  Wind Estimation Algorithm for Unmanned Aerial Systems
% Assumes constant TAS to work
%
% Inputs:
%           -Vg_NED: struct with ground velocity in NED [m/s]
%           -cut_condition: cut condition

if nargin==6
    cut_condition = [0 0];
end

cut_condition = [400 500];
V_XYZ_cut.data = Vg_NED.data(Vg_NED.time<cut_condition(2)&Vg_NED.time>cut_condition(1),:);

x = V_XYZ_cut.data(:,1);
y = V_XYZ_cut.data(:,2);
plot(x,y)
 
x=x(:); y=y(:);
a=[x y ones(size(x))]\[-(x.^2+y.^2)];
xc = -.5*a(1);
yc = -.5*a(2);
R  =  sqrt((a(1)^2+a(2)^2)/4-a(3));

wind.vect = [xc,yc,0];
wind.norm = norm(wind.vect);
wind.direction = atan2(wind.vect(2),wind.vect(1));

end