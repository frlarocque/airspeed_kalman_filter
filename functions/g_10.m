function g = g_10(state,input,meas_noise)
% Output function for Kalman Filter without Euler Angle Approximation
% without alpha
% with a_y
%
% x = [u v w mu_x mu_y mu_z];
% u = [a_x a_y a_z p q r phi theta psi RPM_pusher RPM_hover skew elevator_pprz];
% z = [V_x V_y V_z a_x a_y a_z];

u=state(1);v=state(2);w=state(3);mu_x=state(4);mu_y=state(5);mu_z=state(6);
a_x=input(1);a_y=input(2);a_z=input(3);
p=input(4);q=input(5);r=input(6);
phi=input(7);theta=input(8);psi=input(9);
RPM_pusher = input(10); RPM_hover = input(11);
skew = input(12); elevator_pprz = input(13);

if nargin==3
    v_noise = meas_noise; %[Vx Vy Vz a_x a_y a_z]
else
    v_noise = zeros(6,1);
end

% V_x V_y V_z
speed = DCM(phi,theta,psi)*[u;v;w]+[mu_x;mu_y;mu_z];

% Calculate alpha and saturate it
alpha = atan2(w,u);
alpha = max(min(alpha,deg2rad(15)),deg2rad(-15));

% V_a
V_a = vecnorm([u,v,w]);

% A_x
m = 5;

if RPM_pusher<500
    Fx_push = 0;
else
    Fx_push = Fx_pusher(RPM_pusher,u);
end

Fx_fus = Fx_fuselage(skew,alpha,V_a);
Fx_hprop = Fx_hover_prop(RPM_hover,u);
Fx_w = 0 ; %wing is not installed! Fx_w = Fx_wing(skew,alpha,V_a);
Fx_elev = Fx_elevator(elevator_pprz,V_a);
a_x = (Fx_push+Fx_fus+Fx_hprop+Fx_elev+Fx_w)./m;

%k(4) = 0;
%k(5) = -0.083946669313987*0.8; %b2;
%F_drag = k(4).*u+k(5).*u.^2;
%a_x = (Fx_push+F_drag)./m;

% A_y
if vecnorm([u,v,w])==0
    beta=0;
else
    beta = asin(v/V_a);
end

k_beta = -2E-1;
a_y = beta.*k_beta.*(V_a.^2);

% A_z
Fz_hprop = Fz_hover_prop(RPM_hover);
Fz_fus = Fz_fuselage(skew,alpha,V_a);
Fz_w = 0; % no wing installed! Fz_w = Fz_wing(skew,alpha,V_a);
Fz_elev = Fz_elevator(elevator_pprz,V_a);

a_z = (Fz_fus+Fz_w+Fz_elev+Fz_hprop)./m;

g = [speed;a_x;a_y;a_z]+v_noise;

end