function g = g_5(state,input,meas_noise)
% Output function for Kalman Filter without Euler Angle Approximation
% without alpha
%
% x = [u v w mu_x mu_y mu_z];
% u = [a_x a_y a_z p q r phi theta psi];
% z = [V_x V_y V_z beta];

u=state(1);v=state(2);w=state(3);mu_x=state(4);mu_y=state(5);mu_z=state(6);
a_x=input(1);a_y=input(2);a_z=input(3);p=input(4);q=input(5);r=input(6);phi=input(7);theta=input(8);psi=input(9);

if nargin==3
    v_noise = meas_noise; %[Vx Vy Vz alpha beta]
else
    v_noise = zeros(4,1);
end

% V_x V_y V_z
speed = DCM(phi,theta,psi)*[u;v;w]+[mu_x;mu_y;mu_z];

if v==0
    beta=0;
else
    beta = asin(v/sqrt(u.^2+v.^2+w.^2));
end

g = [speed;beta]+v_noise;

end