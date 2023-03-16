function g = g_9(state,input,meas_noise)
% Output function for Kalman Filter without Euler Angle Approximation
% without alpha
% with a_y
%
% x = [u v w mu_x mu_y mu_z];
% u = [a_x a_y a_z p q r phi theta psi RPM_pusher];
% z = [V_x V_y V_z a_y a_x];

u=state(1);v=state(2);w=state(3);mu_x=state(4);mu_y=state(5);mu_z=state(6);
a_x=input(1);a_y=input(2);a_z=input(3);p=input(4);q=input(5);r=input(6);phi=input(7);theta=input(8);psi=input(9);RPM_pusher = input(10);

if nargin==3
    v_noise = meas_noise; %[Vx Vy Vz a_y a_x]
else
    v_noise = zeros(5,1);
end

% V_x V_y V_z
speed = DCM(phi,theta,psi)*[u;v;w]+[mu_x;mu_y;mu_z];

if vecnorm([u,v,w])==0
    beta=0;
else
    beta = asin(v/vecnorm([u,v,w]));
end

k_beta = -2E-1;
a_y = beta.*k_beta.*(vecnorm([u,v,w]).^2);


% F_pusher = k1*RPM^2+k2*RPM*V+k3*V
% F_drag = -k4*V-k5*V^2
k(1) = 3.680278471152158e-07; %pusher_RPM2Fx_coeff(1);
k(2)= -4.391797235636297e-05; %pusher_RPM2Fx_coeff(2);
k(3) = -0.086517700632897; %pusher_RPM2Fx_coeff(3);
k(4) = 0;
k(5) = -0.083946669313987; %b2;
m = 5;

if RPM_pusher<500
    F_pusher = 0;
else
    F_pusher = k(1).*RPM_pusher.^2+k(2).*RPM_pusher.*u+k(3).*u;
end
F_drag = k(4).*u+k(5).*u.^2;
a_x = (F_pusher+F_drag)./m;

g = [speed;a_y;a_x]+v_noise;

end