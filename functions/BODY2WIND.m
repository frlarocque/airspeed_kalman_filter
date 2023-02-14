function BODY2WIND = BODY2WWIND(alpha,beta)
%B2WIND Calculate rotation matrix to transfer from body to wind
%reference frame
%
% Convert from body frame to wind frame:
%   [Airspeed ;0;0] = B2W(alpha,beta)*[u_b;v_b;w_b]
%  
% Convert from wind frame to body frame:
%   [u_b;v_b;w_b] = inv(B2W(alpha,beta))*[Airspeed;0;0]
%
% Angles in rad


% Pitch through an angle of -alpha about the Yb axis
M_alpha_m = [cos(-alpha)  0 -sin(-alpha);
             0            1            0;
             sin(-alpha)  0  cos(-alpha)];

% Yaw through an angle of beta about the Zb axis
M_beta = [cos(beta)  sin(beta) 0;
          -sin(beta) cos(beta) 0;
          0          0         1];

BODY2WIND = M_beta*M_alpha_m;

end