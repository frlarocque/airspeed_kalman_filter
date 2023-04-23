function F_z = Fz_hover_prop(hover_prop_PWM,V)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% From RCTB data + test fligth in hover

if size(hover_prop_PWM,2)>1
    error("Too many PWM");
else
    % Supposes same constant for all motors
    %Fz = K1*RPM^2
    F_z = 1.2.*-5.69889e-05.*(hover_prop_PWM-1000).^2;

    F_z = F_z + -2.5.*1E-3.*V.*(hover_prop_PWM-1000);
end

end