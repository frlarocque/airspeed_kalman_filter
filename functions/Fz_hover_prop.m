function F_z = Fz_hover_prop(hover_prop_RPM)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

if size(hover_prop_RPM,2)>=4
    % Different motor constant for all motors
    %Fz = K1*RPM_1^2+K2*RPM_2^2+K3*RPM_3^2+K4*RPM_4^2
    Fz_hover_prop_coeff = 1E-6.*[-0.873870581108021 ...
                                 -0.951740938617989 ...
                                 -0.894621788336263 ...
                                 -0.852055641614473];
    F_z = sum(hover_prop_RPM.^2.*Fz_hover_prop_coeff);
else
    % Supposes same constant for all motors
    %Fz = K1*RPM^2
    Fz_hover_prop_coeff = 1.1.*[-3.650781249999991e-06];
    F_z = Fz_hover_prop_coeff(1).*hover_prop_RPM.^2;

end

end