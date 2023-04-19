function F_z = Fz_hover_prop(hover_prop_PWM)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% From RCTB data + test fligth in hover

if size(hover_prop_PWM,2)>1
    error("Too many PWM");
else
    % Supposes same constant for all motors
    %Fz = K1*RPM^2
    F_z = 1.6.*-5.69889e-05.*(hover_prop_PWM-1000).^2;
end

end