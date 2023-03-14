function DCM = DCM(phi,theta,psi)
%DCM Calculates direction cosine matrix (conversion between body and NED earth
% fixed axes)
%
% Convert from body to NED earth:
%       [x_NED;y_NED;z_NED] = DCM(phi,theta,psi)*[x_b;y_b;z_b]
%
% Convert from NED earth to body:
%       [x_b;y_b;z_b] = inv(DCM(phi,theta,psi))*[x_NED;y_NED;z_NED]
%
% Angles in RAD
%
% [c(psi)*c(theta)  -s(psi)*c(phi)+c(psi)*s(theta)*s(phi) s(psi)*s(phi)+c(psi)*s(theta)*c(phi  ;
%  s(psi)*c(theta)  c(psi)c(phi)+s(psi)*s(theta)*s(phi)   -c(psi)*s(phi)+s(psi)*s(theta)*c(phi);
%  s(theta)         c(theta)*s(phi)                       c(theta)*c(phi)                      ];
%

DCM = ...
[cos(psi)     -sin(psi) 0;
 sin(psi)    cos(psi)   0;
 0           0          1]*...
[cos(theta)  0          sin(theta)
 0           1          0;
 -sin(theta) 0          cos(theta)]*...
[1           0          0;
0            cos(phi)   -sin(phi)
0            sin(phi)   cos(phi)];

end

