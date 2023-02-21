function g = g_4(state,input,meas_noise)
% Output function for Kalman Filter without Euler Angle Approximation
% witout beta nor alpha
%
% x = [u v w mu_x mu_y mu_z];
% u = [a_x a_y a_z p q r phi theta psi];
% z = [V_x V_y V_z];

u=state(1);v=state(2);w=state(3);mu_x=state(4);mu_y=state(5);mu_z=state(6);
a_x=input(1);a_y=input(2);a_z=input(3);p=input(4);q=input(5);r=input(6);phi=input(7);theta=input(8);psi=input(9);

if nargin==3
    v_noise = meas_noise; %[Vx Vy Vz alpha beta]
else
    v_noise = zeros(3,1);
end

% V_x V_y V_z
speed = DCM(phi,theta,psi)*[u;v;w]+[mu_x;mu_y;mu_z];

g = [speed]+v_noise;

end