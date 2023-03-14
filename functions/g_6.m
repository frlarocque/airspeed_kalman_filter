function g = g_6(state,input,meas_noise)
% Output function for Kalman Filter without Euler Angle Approximation
% without alpha
%
% x = [u v w mu_x mu_y mu_z];
% u = [a_x a_y a_z p q r phi theta psi];
% z = [V_x V_y V_z beta pusher_rpm];

u=state(1);v=state(2);w=state(3);mu_x=state(4);mu_y=state(5);mu_z=state(6);
a_x=input(1);a_y=input(2);a_z=input(3);p=input(4);q=input(5);r=input(6);phi=input(7);theta=input(8);psi=input(9);

if nargin==3
    v_noise = meas_noise; %[Vx Vy Vz beta pusher_rpm]
else
    v_noise = zeros(5,1);
end

% V_x V_y V_z
speed = DCM(phi,theta,psi)*[u;v;w]+[mu_x;mu_y;mu_z];

if v==0
    beta=0;
else
    beta = asin(v/sqrt(u.^2+v.^2+w.^2));
end

%F_pusher(RPM,V) = k4*V^2
% k1*RPM^2+k2*RPM*V+k3*V-k4*V^2 = 0

k(1) = 3.680278471152158e-07; %pusher_RPM2Fx_coeff(1);
k(2)= -4.391797235636297e-05; %pusher_RPM2Fx_coeff(2);
k(3) = -0.086517700632897; %pusher_RPM2Fx_coeff(3);
k(4) = 0;
k(5) = 0.083946669313987; %b2;
y = u; 
grav = 9.81;
m = 5.5;
z = 0; % accel_x
x = 0;%asin(-w./vecnorm([u v w])); %gamma

if y>0
    pusher_rpm = max(real((sqrt(4.*k(1).*(grav.*m.*sin(x) + k(5).*y.^2 - k(3).*y + k(4).*y + m.*z) + k(2).^2.*y.^2) - k(2).*y)./(2.*k(1))),0); %only taking real part and saturating at 0
else
    pusher_rpm = 0;
end

g = [speed;beta;pusher_rpm]+v_noise;

end