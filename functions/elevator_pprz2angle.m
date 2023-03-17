function angle = elevator_pprz2angle(pprz)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
if pprz<0
    angle = -7.28E-5.*pprz+-2.54E-2; %[rad]
elseif pprz>0
    angle = -1.49E-5.*pprz+-3.57E-2; %[rad]
else
    angle = 0;
end

% Saturate at -10 to 37 deg of deflection
angle = min(max(angle,deg2rad(-10)),deg2rad(37));

end