function g = g_14_cst(state,input,meas_noise)
% Output function for Kalman Filter without Euler Angle Approximation
% without alpha
% with a_y
%
% x = [u v w mu_x mu_y mu_z k_x k_y k_z];
% u = [a_x a_y a_z p q r phi theta psi RPM_pusher RPM_hover skew elevator_pprz];
% z = [V_x V_y V_z a_x a_y a_z pitot];

global EKF_AW_USE_MODEL_BASED EKF_AW_USE_BETA EKF_AW_WING_INSTALLED EKF_AW_PROPAGATE_OFFSET EKF_AW_VEHICLE_MASS

u=state(1);v=state(2);w=state(3);
mu_x=state(4);mu_y=state(5);mu_z=state(6);
k_x = state(7);k_y = state(8);k_z = state(9);

sign_u = sign(u);sign_v = sign(v);sign_w = sign(w);

a_x=input(1);a_y=input(2);a_z=input(3);
p=input(4);q=input(5);r=input(6);
phi=input(7);theta=input(8);psi=input(9);
PWM_pusher = input(10); PWM_hover = input(11);
skew = input(12); elevator_PWM = input(13);

if nargin==3
    v_noise = meas_noise; %[Vx Vy Vz a_x a_y a_z pitot]
else
    v_noise = zeros(7,1);
end

% V_x V_y V_z
speed = DCM(phi,theta,psi)*[u;v;w]+[mu_x;mu_y;mu_z];

% Calculate alpha and saturate it
alpha = atan2(w,u);
alpha = max([min([alpha,deg2rad(15)]),deg2rad(-15)]);

% V_a
V_a = vecnorm([u,v,w]);

% A_x
pwm2RPM =1.0e+04 .*[0.001721000000000  -1.675885714285714];
RPM = polyval(pwm2RPM,PWM_pusher);

if PWM_pusher<1000
    Fx_push = 0;
else
    Fx_push = Fx_pusher(RPM,u);
end

if EKF_AW_WING_INSTALLED
    Fx_w = Fx_wing(skew,alpha,u);
else
    Fx_w = 0; 
end

bias_x = 0.50;
Fx_fus = -0.03.*u.*u.*sign(u);%-0.083668;
Fx_hover = -0.5.*u; %-0.2

a_x = (Fx_push + Fx_fus + Fx_hover + Fx_w)./EKF_AW_VEHICLE_MASS+bias_x;
%a_x = a_x + u.*u.*k_x;
%a_x = a_x + u.*u.*(k_x+k_y*sin(skew).^2+k_z.*alpha.*sin(skew).^2)./EKF_AW_VEHICLE_MASS;

% A_y
bias_y = -0.15.*cos(skew);
k_v = 2.*-3.2E-1;
Fy_wing = -0.022.*u.^2.*cos(skew).*sin(skew).^2;


a_y = (k_v.*v.^2.*sign_v + Fy_wing)./EKF_AW_VEHICLE_MASS + bias_y;

% A_z
Fz_hprop = Fz_hover_prop(PWM_hover,u);
Fz_w = Fz_wing(skew,alpha,u);

a_z = (Fz_w+Fz_hprop)./EKF_AW_VEHICLE_MASS;

g = [speed;a_x;a_y;a_z;u]+v_noise;

end