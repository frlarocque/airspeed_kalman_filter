function B2W = body_to_wind(alpha,beta)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

% Pitch through an angle of -alpha about the Yb axis
M_alpha_m = [cos(-alpha)  0 -sin(-alpha);
             0            1            0;
             sin(-alpha)  0  cos(-alpha)];

% Yaw through an angle of beta about the Zb axis
M_beta = [cos(beta)  sin(beta) 0;
          -sin(beta) cos(beta) 0;
          0          0         1];

B2W = M_beta*M_alpha_m;

end