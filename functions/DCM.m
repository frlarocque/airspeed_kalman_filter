function DCM = DCM(phi,theta,psi)
% Calculates direction cosine matrix (conversion between body and earth
% fixed axes)
% Angles in RAD
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

