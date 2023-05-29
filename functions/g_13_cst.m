function g = g_13_cst(state,input,meas_noise)
% Output function for Kalman Filter without Euler Angle Approximation
% without alpha
% with a_y
%
% x = [u v w mu_x mu_y mu_z k_x k_y k_z];
% u = [a_x a_y a_z p q r phi theta psi RPM_pusher RPM_hover skew elevator_pprz];
% z = [V_x V_y V_z a_x a_y a_z pitot];

global EKF_AW_USE_MODEL_BASED EKF_AW_USE_BETA EKF_AW_WING_INSTALLED EKF_AW_PROPAGATE_OFFSET EKF_AW_VEHICLE_MASS
global EKF_AW_FORCES_FUSELAGE EKW_AW_FORCES_HOVER EKF_AW_FORCES_PUSHER EKF_AW_FORCES_WING EKF_AW_FORCES_ELEVATOR

u=state(1);v=state(2);w=state(3);
mu_x=state(4);mu_y=state(5);mu_z=state(6);
k_x = state(7);k_y = state(8);k_z = state(9);

sign_u = sign(u);sign_v = sign(v);sign_w = sign(w);

a_x=input(1);a_y=input(2);a_z=input(3);
p=input(4);q=input(5);r=input(6);
phi=input(7);theta=input(8);psi=input(9);
RPM_pusher = input(10); RPM_hover = input(11);
skew = input(12); elevator_pprz = input(13);

if nargin==3
    v_noise = meas_noise; %[Vx Vy Vz a_x a_y a_z pitot]
else
    v_noise = zeros(7,1);
end

% Saturate Skew
skew = min(max(skew,0),deg2rad(90));

% V_x V_y V_z
speed = DCM(phi,theta,psi)*[u;v;w]+[mu_x;mu_y;mu_z];

% Calculate alpha and saturate it
alpha = atan2(w,u);
alpha = max(min(alpha,deg2rad(15)),deg2rad(-15));

% V_a
V_a = vecnorm([u,v,w]);

% A_x
if RPM_pusher<500
    Fx_push = 0;
else
    Fx_push = Fx_pusher(RPM_pusher,u);
end

Fx_fus = -0.048.*u.*u.*sign(u); %Fx_fuselage(skew,alpha,u);

if RPM_hover<1000
    Fx_hprop = 0;
else
    Fx_hprop = -0.75.*u;
end

if EKF_AW_WING_INSTALLED
    Fx_w = Fx_wing_3(skew,alpha,u);
else
    Fx_w = 0; 
end

a_x = (Fx_fus + Fx_hprop + Fx_w + Fx_push + u.^2*sign_u*k_x)./EKF_AW_VEHICLE_MASS;

% A_y
if RPM_hover<1000
    Fy_hprop = 0;
else
    Fy_hprop = 1.0.*-0.75.*v;
end
if EKF_AW_USE_BETA
    if vecnorm([u,v,w])==0
        beta=0;
    else
        beta = asin(v/V_a);
    end
    
    k_beta = -0.219;
    a_y = ((beta.*k_beta.*(V_a.^2)+Fy_hprop)./EKF_AW_VEHICLE_MASS) + k_y;
else
    k_v = 1.*-3.2E-1;
    a_y = (k_v.*v.^2.*sign_v+Fy_hprop)./EKF_AW_VEHICLE_MASS + k_y.*u.^2.*sign_u;
end

if EKF_AW_WING_INSTALLED
    Fy_w = Fy_wing(skew,alpha,u);
    a_y = a_y + Fy_w./EKF_AW_VEHICLE_MASS;
end

% A_z
Fz_hprop = Fz_hover_prop(RPM_hover);
Fz_fus = 0;%Fz_fuselage(skew,alpha,V_a);
if EKF_AW_WING_INSTALLED
    Fz_w = Fz_wing(skew,alpha,V_a);
else
    Fz_w = 0; 
end
Fz_elev = 0;%Fz_elevator_2(elevator_pprz,V_a);

a_z = (Fz_fus+Fz_w+Fz_elev+Fz_hprop+k_z*u.^2.*sign(u))./EKF_AW_VEHICLE_MASS;

g = [speed;a_x;a_y;a_z;u]+v_noise;

EKF_AW_FORCES_FUSELAGE = [Fx_fus;0;0];
EKW_AW_FORCES_HOVER = [Fx_hprop;Fy_hprop;Fz_hprop];
EKF_AW_FORCES_PUSHER = [Fx_push;0;0];
EKF_AW_FORCES_WING = [Fx_w;Fy_w;Fz_w];
EKF_AW_FORCES_ELEVATOR = [0;0;Fz_elev];

end