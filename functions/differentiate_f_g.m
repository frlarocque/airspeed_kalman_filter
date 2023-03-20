%% Define syms

syms u v w mu_x mu_y mu_z k_x k_y k_z a_x a_y a_z p q r phi theta psi RPM_pusher RPM_hover skew elevator_angle
syms g
syms w_a_x w_a_y w_a_z w_w_x w_w_y w_w_z w_mu_x w_mu_y w_mu_z w_k_x w_k_y w_k_z

assume([u v w mu_x mu_y mu_z k_x k_y k_z a_x a_y a_z p q r phi theta psi RPM_pusher RPM_hover skew elevator_pprz g],'real')
assume([w_a_x w_a_y w_a_z w_w_x w_w_y w_w_z w_mu_x w_mu_y w_mu_z w_k_x w_k_y w_k_z],'real')
%% Define f (f_4.m)
%u=state(1);v=state(2);w=state(3);mu_x=state(4);mu_y=state(5);mu_z=state(6);k_x = state(7);k_y = state(8);k_z = state(9);
%a_x=input(1);a_y=input(2);a_z=input(3);
%p=input(4);q=input(5);r=input(6);
%phi=input(7);theta=input(8);psi=input(9);
%RPM_pusher = input(10); RPM_hover = input(11);
%skew = input(12); elevator_angle = input(13);


w_a  = [w_a_x w_a_y w_a_z]';
w_w  = [w_w_x w_w_y w_w_z]';
w_mu = [w_mu_x w_mu_y w_mu_z]';
w_k  = [w_k_x w_k_y w_k_z]';

%g= 9.81;

% [u_dot v_dot w_dot]'
velocity_body = [0 -w  v;
                 w  0 -u;
                 -v u  0;]*([p;q;r]+w_w)+ ...
                 DCM(phi,theta,psi)'*[0;0;g]+...
                 [a_x;a_y;a_z]+w_a;

%[mu_x_dot mu_y_dot mu_z_dot]'
wind = w_mu;

%[k_x_dot k_y_dot k_z_dot]'
k = w_k;

%f = x_dot = [u_dot v_dot w_dot mu_x_dot mu_y_dot mu_z_dot k_x_dot k_y_dot k_z_dot];
f_out = [velocity_body;wind;k];

%% Get F (d x/dx)
% F = [] nxn where n=number of states, n=number of states
%
% F = [ dx_1/dx_1 dx_1/dx_2 ... dx_1/dx_n ;
%       dx_2/dx_1 dx_2/dx_2 ....dx_2/dx_n ;
%       ...                               ;
%       dx_n/dx_1 dx_n/dx_2 ....dx_n/dx_n ];

x = [u v w mu_x mu_y mu_z k_x k_y k_z];

F = sym('F',length(x));

for i=1:length(x)
    F(:,i) = diff(f_out,x(i));
end

F

% F =
%  
% [          0,   r + w_w_z, - q - w_w_y, 0, 0, 0, 0, 0, 0]
% [- r - w_w_z,           0,   p + w_w_x, 0, 0, 0, 0, 0, 0]
% [  q + w_w_y, - p - w_w_x,           0, 0, 0, 0, 0, 0, 0]
% [          0,           0,           0, 0, 0, 0, 0, 0, 0]
% [          0,           0,           0, 0, 0, 0, 0, 0, 0]
% [          0,           0,           0, 0, 0, 0, 0, 0, 0]
% [          0,           0,           0, 0, 0, 0, 0, 0, 0]
% [          0,           0,           0, 0, 0, 0, 0, 0, 0]
% [          0,           0,           0, 0, 0, 0, 0, 0, 0]

%% Get L (d x/dw)
% L = [] nxm where n=number of states, m=number of noise
%
% L = [ dx_1/dw_1 dx_1/dw_2 ... dx_1/dw_m ;
%       dx_2/dw_1 dx_2/dw_2 ....dx_2/dw_m ;
%       ...                               ;
%       dx_n/dw_1 dx_n/dw_2 ....dx_n/dw_m ];

w_noise = [w_a' w_w' w_mu' w_k'];

L = sym('L',[length(x),length(w_noise)]);

for i=1:length(w_noise)
    L(:,i) = diff(f_out,w_noise(i));
end

L

% L =
%  
% [1, 0, 0,  0, -w,  v, 0, 0, 0, 0, 0, 0]
% [0, 1, 0,  w,  0, -u, 0, 0, 0, 0, 0, 0]
% [0, 0, 1, -v,  u,  0, 0, 0, 0, 0, 0, 0]
% [0, 0, 0,  0,  0,  0, 1, 0, 0, 0, 0, 0]
% [0, 0, 0,  0,  0,  0, 0, 1, 0, 0, 0, 0]
% [0, 0, 0,  0,  0,  0, 0, 0, 1, 0, 0, 0]
% [0, 0, 0,  0,  0,  0, 0, 0, 0, 1, 0, 0]
% [0, 0, 0,  0,  0,  0, 0, 0, 0, 0, 1, 0]
% [0, 0, 0,  0,  0,  0, 0, 0, 0, 0, 0, 1]

%% Define var for g
syms V_x V_y V_z a_x_filt a_y_filt a_z_filt
syms w_V_x w_V_y w_V_z w_a_x_filt w_a_y_filt w_a_z_filt
syms m 
syms alpha beta
assume([alpha beta],'real')

syms k1_Fx_push k2_Fx_push k3_Fx_push
syms k1_Fx_fus k2_Fx_fus k3_Fx_fus k4_Fx_fus
syms k1_Fx_hprop k2_Fx_hprop
syms k1_Fx_w k2_Fx_w k3_Fx_w k4_Fx_w k5_Fx_w
syms k1_Fx_elev k2_Fx_elev k3_Fx_elev

syms k_beta

syms k1_Fz_hprop
syms k1_Fz_fus k2_Fz_fus k3_Fz_fus k4_Fz_fus
syms k1_Fz_w k2_Fz_w k3_Fz_w k4_Fz_w
syms k1_Fz_elev k2_Fz_elev k3_Fz_elev

assume([V_x V_y V_z a_x_filt a_y_filt a_z_filt  w_V_x w_V_y w_V_z w_a_x_filt w_a_y_filt w_a_z_filt  m],'real')
assume([k1_Fx_push k2_Fx_push k3_Fx_push k1_Fx_fus k2_Fx_fus k3_Fx_fus k4_Fx_fus k1_Fx_hprop k2_Fx_hprop k1_Fx_w k2_Fx_w k3_Fx_w k4_Fx_w k5_Fx_w k1_Fx_elev k2_Fx_elev k3_Fx_elev],'real')
assume([k_beta],'real')
assume([k1_Fz_hprop k1_Fz_fus k2_Fz_fus k3_Fz_fus k4_Fz_fus k1_Fz_w k2_Fz_w k3_Fz_w k4_Fz_w k1_Fz_elev k2_Fz_elev k3_Fz_elev],'real')

%% Define g (g_11.m)

% x = [u v w mu_x mu_y mu_z k_x k_y k_z];
% u = [a_x a_y a_z p q r phi theta psi RPM_pusher RPM_hover skew elevator_angle];
% z = [V_x V_y V_z a_x a_y a_z];

%u=state(1);v=state(2);w=state(3);
%mu_x=state(4);mu_y=state(5);mu_z=state(6);
%k_x = state(7);k_y = state(8);k_z = state(9);

%a_x=input(1);a_y=input(2);a_z=input(3);
%p=input(4);q=input(5);r=input(6);
%phi=input(7);theta=input(8);psi=input(9);
%RPM_pusher = input(10); RPM_hover = input(11);
%skew = input(12); elevator_pprz = input(13);


v_noise = [w_V_x w_V_y w_V_z w_a_x_filt w_a_y_filt w_a_z_filt]';


% V_x V_y V_z
speed = DCM(phi,theta,psi)*[u;v;w]+[mu_x;mu_y;mu_z];

% Calculate alpha and saturate it
alpha = atan(w./u);
%alpha = max(min(alpha,deg2rad(15)),deg2rad(-15));

% V_a
%V_a = u;
V_a = sqrt(u.^2+v.^2+w.^2);

% A_x
%m = 5.75;

Fx_push = k1_Fx_push.*RPM_pusher.^2+k2_Fx_push.*RPM_pusher.*u+k3_Fx_push.*u;
Fx_fus = (k1_Fx_fus.*cos(skew)+k2_Fx_fus+k3_Fx_fus.*alpha+k4_Fx_fus.*alpha.^2).*V_a.^2;
Fx_hprop = k1_Fx_hprop.*u.^2+k2_Fx_hprop.*RPM_hover.^2.*sqrt(u);
Fx_w = (k1_Fx_w.*(1+k5_Fx_w.*skew)+(k2_Fx_w.*alpha+k3_Fx_w.*alpha.^2)).*(sin(skew).^2+k4_Fx_w).*V_a.^2;
Fx_elev = (k1_Fx_elev+k2_Fx_elev.*elevator_angle+k3_Fx_elev.*elevator_angle.^2).*V_a.^2;
a_x = (Fx_push+Fx_fus+Fx_hprop+Fx_elev+Fx_w+k_x.*u.^2)./m;

% A_y
beta = asin(v/V_a);

%k_beta = -2E-1;
a_y = beta.*k_beta.*(V_a.^2) + k_y;

% A_z
Fz_hprop = k1_Fz_hprop.*RPM_hover.^2;
Fz_fus = (k1_Fz_fus.*cos(skew)+k2_Fz_fus+k3_Fz_fus.*alpha+k4_Fz_fus.*alpha.^2).*V_a.^2;
Fz_w = ((k1_Fz_w+k2_Fz_w.*alpha+k3_Fz_w.*alpha.^2).*(sin(skew).^2+k4_Fz_w)).*V_a.^2;
Fz_elev = (k1_Fz_elev+k2_Fz_elev.*elevator_angle+k3_Fz_elev.*elevator_angle.^2).*V_a.^2;

a_z = (Fz_fus+Fz_w+Fz_elev+Fz_hprop)./m + k_z;

g_out = [speed;a_x;a_y;a_z]+v_noise;

%% Get G (d g/dx)
% G = [] nxm where n=number of outputs, m=number of states
%
% G = [ dg_1/dx_1 dg_1/dx_2 ... dg_1/dx_m ;
%       dg_2/dx_1 dg_2/dx_2 ....dg_2/dx_m ;
%       ...                               ;
%       dg_n/dx_1 dg_n/dx_2 ....dg_n/dx_m ];

x = [u v w mu_x mu_y mu_z k_x k_y k_z];

G = sym('G',[length(g_out), length(x)]);

for i=1:length(x)
    G(:,i) = diff(g_out,x(i));
end

G

% G =
%  
% [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       cos(psi)*cos(theta),                                                                                                                                                                                                                          cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi),                                                                                                                                                                                                                                                                                                                                                                                                                                          sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta), 1, 0, 0,     0, 0, 0]
% [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       cos(theta)*sin(psi),                                                                                                                                                                                                                          cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta),                                                                                                                                                                                                                                                                                                                                                                                                                                          cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi), 0, 1, 0,     0, 0, 0]
% [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               -sin(theta),                                                                                                                                                                                                                                                       cos(theta)*sin(phi),                                                                                                                                                                                                                                                                                                                                                                                                                                                                       cos(phi)*cos(theta), 0, 0, 1,     0, 0, 0]
% [(k3_Fx_push + RPM_pusher*k2_Fx_push + 2*k1_Fx_hprop*u + 2*k_x*u + 2*u*(k2_Fx_fus + k4_Fx_fus*atan(w/u)^2 + k1_Fx_fus*cos(skew) + k3_Fx_fus*atan(w/u)) + 2*u*(k3_Fx_elev*elevator_angle^2 + k2_Fx_elev*elevator_angle + k1_Fx_elev) - ((k3_Fx_fus*w)/(u^2*(w^2/u^2 + 1)) + (2*k4_Fx_fus*w*atan(w/u))/(u^2*(w^2/u^2 + 1)))*(u^2 + v^2 + w^2) - (sin(skew)^2 + k4_Fx_w)*((k2_Fx_w*w)/(u^2*(w^2/u^2 + 1)) + (2*k3_Fx_w*w*atan(w/u))/(u^2*(w^2/u^2 + 1)))*(u^2 + v^2 + w^2) + (0.5000*RPM_hover^2*k2_Fx_hprop)/u^0.5000 + 2*u*(sin(skew)^2 + k4_Fx_w)*(k3_Fx_w*atan(w/u)^2 + k1_Fx_w*(k5_Fx_w*skew + 1) + k2_Fx_w*atan(w/u)))/m, (2*v*(k2_Fx_fus + k4_Fx_fus*atan(w/u)^2 + k1_Fx_fus*cos(skew) + k3_Fx_fus*atan(w/u)) + 2*v*(k3_Fx_elev*elevator_angle^2 + k2_Fx_elev*elevator_angle + k1_Fx_elev) + 2*v*(sin(skew)^2 + k4_Fx_w)*(k3_Fx_w*atan(w/u)^2 + k1_Fx_w*(k5_Fx_w*skew + 1) + k2_Fx_w*atan(w/u)))/m, (2*w*(k2_Fx_fus + k4_Fx_fus*atan(w/u)^2 + k1_Fx_fus*cos(skew) + k3_Fx_fus*atan(w/u)) + 2*w*(k3_Fx_elev*elevator_angle^2 + k2_Fx_elev*elevator_angle + k1_Fx_elev) + (k3_Fx_fus/(u*(w^2/u^2 + 1)) + (2*k4_Fx_fus*atan(w/u))/(u*(w^2/u^2 + 1)))*(u^2 + v^2 + w^2) + (sin(skew)^2 + k4_Fx_w)*(k2_Fx_w/(u*(w^2/u^2 + 1)) + (2*k3_Fx_w*atan(w/u))/(u*(w^2/u^2 + 1)))*(u^2 + v^2 + w^2) + 2*w*(sin(skew)^2 + k4_Fx_w)*(k3_Fx_w*atan(w/u)^2 + k1_Fx_w*(k5_Fx_w*skew + 1) + k2_Fx_w*atan(w/u)))/m, 0, 0, 0, u^2/m, 0, 0]
% [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  2*k_beta*u*asin(v/(u^2 + v^2 + w^2)^0.5000) - (k_beta*u*v)/((1 - v^2/(u^2 + v^2 + w^2))^0.5000*(u^2 + v^2 + w^2)^0.5000),                                                                                                   2*k_beta*v*asin(v/(u^2 + v^2 + w^2)^0.5000) + (k_beta*(1/(u^2 + v^2 + w^2)^0.5000 - v^2/(u^2 + v^2 + w^2)^1.5000)*(u^2 + v^2 + w^2))/(1 - v^2/(u^2 + v^2 + w^2))^0.5000,                                                                                                                                                                                                                                                                                                                                                                  2*k_beta*w*asin(v/(u^2 + v^2 + w^2)^0.5000) - (k_beta*v*w)/((1 - v^2/(u^2 + v^2 + w^2))^0.5000*(u^2 + v^2 + w^2)^0.5000), 0, 0, 0,     0, 1, 0]
% [                                                                                                                                (2*u*(k2_Fz_fus + k4_Fz_fus*atan(w/u)^2 + k1_Fz_fus*cos(skew) + k3_Fz_fus*atan(w/u)) + 2*u*(k3_Fz_elev*elevator_angle^2 + k2_Fz_elev*elevator_angle + k1_Fz_elev) - ((k3_Fz_fus*w)/(u^2*(w^2/u^2 + 1)) + (2*k4_Fz_fus*w*atan(w/u))/(u^2*(w^2/u^2 + 1)))*(u^2 + v^2 + w^2) - (sin(skew)^2 + k4_Fz_w)*((k2_Fz_w*w)/(u^2*(w^2/u^2 + 1)) + (2*k3_Fz_w*w*atan(w/u))/(u^2*(w^2/u^2 + 1)))*(u^2 + v^2 + w^2) + 2*u*(sin(skew)^2 + k4_Fz_w)*(k1_Fz_w + k3_Fz_w*atan(w/u)^2 + k2_Fz_w*atan(w/u)))/m,                    (2*v*(k2_Fz_fus + k4_Fz_fus*atan(w/u)^2 + k1_Fz_fus*cos(skew) + k3_Fz_fus*atan(w/u)) + 2*v*(k3_Fz_elev*elevator_angle^2 + k2_Fz_elev*elevator_angle + k1_Fz_elev) + 2*v*(sin(skew)^2 + k4_Fz_w)*(k1_Fz_w + k3_Fz_w*atan(w/u)^2 + k2_Fz_w*atan(w/u)))/m,                    (2*w*(k2_Fz_fus + k4_Fz_fus*atan(w/u)^2 + k1_Fz_fus*cos(skew) + k3_Fz_fus*atan(w/u)) + 2*w*(k3_Fz_elev*elevator_angle^2 + k2_Fz_elev*elevator_angle + k1_Fz_elev) + (k3_Fz_fus/(u*(w^2/u^2 + 1)) + (2*k4_Fz_fus*atan(w/u))/(u*(w^2/u^2 + 1)))*(u^2 + v^2 + w^2) + (sin(skew)^2 + k4_Fz_w)*(k2_Fz_w/(u*(w^2/u^2 + 1)) + (2*k3_Fz_w*atan(w/u))/(u*(w^2/u^2 + 1)))*(u^2 + v^2 + w^2) + 2*w*(sin(skew)^2 + k4_Fz_w)*(k1_Fz_w + k3_Fz_w*atan(w/u)^2 + k2_Fz_w*atan(w/u)))/m, 0, 0, 0,     0, 0, 1]
%  

%% Get M (d g/dw)
% M = [] nxm where n=number of outputs, m=number of noise
%
% M = [ dg_1/dw_1 dg_1/dw_2 ... dg_1/dw_m ;
%       dg_2/dw_1 dg_2/dw_2 ....dg_2/dw_m ;
%       ...                               ;
%       dg_n/dw_1 dg_n/dw_2 ....dg_n/dw_m ];

v_noise

M = sym('M',[length(g_out),length(v_noise)]);

for i=1:length(v_noise)
    M(:,i) = diff(g_out,v_noise(i));
end

M

% M =
%  
% [1, 0, 0, 0, 0, 0]
% [0, 1, 0, 0, 0, 0]
% [0, 0, 1, 0, 0, 0]
% [0, 0, 0, 1, 0, 0]
% [0, 0, 0, 0, 1, 0]
% [0, 0, 0, 0, 0, 1]
