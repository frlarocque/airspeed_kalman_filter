%% Init
clear all
close all
clc

%% Define syms for f

syms u v w mu_x mu_y mu_z k_x k_y k_z a_x a_y a_z p q r phi theta psi RPM_pusher RPM_hover_1 RPM_hover_2 RPM_hover_3 RPM_hover_4 skew elevator_angle
syms sign_u sign_v
syms g
syms w_accel_x w_accel_y w_accel_z w_gyro_x w_gyro_y w_gyro_z w_mu_x w_mu_y w_mu_z w_k_x w_k_y w_k_z

assume([u v w mu_x mu_y mu_z k_x k_y k_z a_x a_y a_z p q r phi theta psi RPM_pusher RPM_hover_1 RPM_hover_2 RPM_hover_3 RPM_hover_4 skew elevator_angle g sign_u sign_v],'real')
assume([w_accel_x w_accel_y w_accel_z w_gyro_x w_gyro_y w_gyro_z w_mu_x w_mu_y w_mu_z w_k_x w_k_y w_k_z],'real')

%% Define var for g
syms V_gnd_x V_gnd_y V_gnd_z a_x_filt a_y_filt a_z_filt V_pitot
syms w_V_gnd_x w_V_gnd_y w_V_gnd_z w_a_x_filt w_a_y_filt w_a_z_filt w_V_pitot
syms m 
syms alpha beta
assume([alpha beta],'real')

syms k1_Fx_push k2_Fx_push k3_Fx_push
syms k1_Fx_fus k2_Fx_fus k3_Fx_fus k4_Fx_fus
syms k1_Fx_hprop k2_Fx_hprop
syms k1_Fx_w k2_Fx_w k3_Fx_w k4_Fx_w k5_Fx_w
syms k1_Fx_elev k2_Fx_elev k3_Fx_elev


syms k_beta k_v

syms k1_Fz_hprop
syms k1_Fz_fus k2_Fz_fus k3_Fz_fus k4_Fz_fus
syms k1_Fz_w k2_Fz_w k3_Fz_w k4_Fz_w
syms k1_Fz_elev k2_Fz_elev k3_Fz_elev

syms k1_drag k2_drag Fz_hprop k_Fz_fus k_Fz_w k_Fz_elev

assume([V_gnd_x V_gnd_y V_gnd_z a_x_filt a_y_filt a_z_filt V_pitot],'real')
assume([w_V_gnd_x w_V_gnd_y w_V_gnd_z w_a_x_filt w_a_y_filt w_a_z_filt w_V_pitot m],'real')
assume([k1_Fx_push k2_Fx_push k3_Fx_push k1_Fx_fus k2_Fx_fus k3_Fx_fus k4_Fx_fus k1_Fx_hprop k2_Fx_hprop k1_Fx_w k2_Fx_w k3_Fx_w k4_Fx_w k5_Fx_w k1_Fx_elev k2_Fx_elev k3_Fx_elev],'real')
assume([k_beta,k_v],'real')
assume([k1_Fz_hprop k1_Fz_fus k2_Fz_fus k3_Fz_fus k4_Fz_fus k1_Fz_w k2_Fz_w k3_Fz_w k4_Fz_w k1_Fz_elev k2_Fz_elev k3_Fz_elev],'real')
assume([k1_drag k2_drag Fz_hprop k_Fz_fus k_Fz_w k_Fz_elev],'real')

%% Define dimensions
x = [u v w mu_x mu_y mu_z k_x k_y k_z];
state_dim = length(x);

w_a  = [w_accel_x w_accel_y w_accel_z]';
w_gyro  = [w_gyro_x w_gyro_y w_gyro_z]';
w_mu = [w_mu_x w_mu_y w_mu_z]';
w_k  = [w_k_x w_k_y w_k_z]';
state_noise = [w_a;w_gyro;w_mu;w_k];
state_noise_dim = length(state_noise);

input = [a_x a_y a_z p q r phi theta psi RPM_pusher RPM_hover_1 RPM_hover_2 RPM_hover_3 RPM_hover_4 skew elevator_angle];
input_dim = length(input);

z = [V_gnd_x V_gnd_y V_gnd_z a_x_filt a_y_filt a_z_filt];
z_dim = length(z);

z_noise = [w_V_gnd_x w_V_gnd_y w_V_gnd_z w_a_x_filt w_a_y_filt w_a_z_filt w_V_pitot]';
z_noise_dim = length(z_noise);

%% Produce list of states, noise and

% States
fprintf('Producing list of states:\n')
for i=1:length(x)
    if i==1
        fprintf('EKF_AW_%s',x(i))
    else
        fprintf(', EKF_AW_%s',x(i))
    end
end
fprintf(', EKF_AW_COV_SIZE\n')

% Process Noise
fprintf('Producing list of process noise:\n')
for i=1:length(state_noise)
    if i==1
        fprintf('EKF_AW_%s',state_noise(i))
    else
        fprintf(', EKF_AW_%s',state_noise(i))
    end
end
fprintf(', EKF_AW_R_SIZE\n')

% Measurement noise
fprintf('Producing list of measurement noise:\n')
for i=1:length(z_noise)
    if i==1
        fprintf('EKF_AW_%s',z_noise(i))
    else
        fprintf(', EKF_AW_%s',z_noise(i))
    end
end
fprintf(', EKF_AW_Q_SIZE\n')

%% Define f (f_4.m)
% x = [u v w mu_x mu_y mu_z k_x k_y k_z];
% u = [a_x a_y a_z p q r phi theta psi RPM_pusher RPM_hover skew elevator_angle];
% z = [V_x V_y V_z a_x a_y a_z];

% [u_dot v_dot w_dot]'
velocity_body = [0 -w  v;
                 w  0 -u;
                 -v u  0;]*([p;q;r]+w_gyro)+ ...
                 DCM(phi,theta,psi)'*[0;0;g]+...
                 [a_x;a_y;a_z]+w_a;

%[mu_x_dot mu_y_dot mu_z_dot]'
wind = w_mu;

%[k_x_dot k_y_dot k_z_dot]'
k = w_k;

%f = x_dot = [u_dot v_dot w_dot mu_x_dot mu_y_dot mu_z_dot k_x_dot k_y_dot k_z_dot];
f_out = [velocity_body;wind;k];

fprintf('Printing State Derivative\n')
for i=1:length(f_out)
    temp_cell = children(subs(f_out(i),[w_a; w_gyro; w_mu; w_k],zeros(12,1)));
    fprintf('state_dev(%d,%d) =',i-1,0)
    for j=1:length(temp_cell)
        if j==1
            fprintf(' %s',string(temp_cell(j)))
        else
            fprintf(' + %s',string(temp_cell(j)))
        end
    end
    fprintf('\n')
end

%% Get F (d x/dx)
% F = [] nxn where n=number of states, n=number of states
%
% F = [ dx_1/dx_1 dx_1/dx_2 ... dx_1/dx_n ;
%       dx_2/dx_1 dx_2/dx_2 ....dx_2/dx_n ;
%       ...                               ;
%       dx_n/dx_1 dx_n/dx_2 ....dx_n/dx_n ];

F = sym('F',state_dim);

for i=1:state_dim
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

for i=1:size(F,1)
    for j=1:size(F,2)
        if ~isAlways(F(i,j)==0,Unknown="false")
            fprintf('F(%d,%d) = %s;\n',i-1,j-1,subs(F(i,j),state_noise,zeros(12,1)));
        end
    end
end

%% Get L (d x/dw)
% L = [] nxm where n=number of states, m=number of noise
%
% L = [ dx_1/dw_1 dx_1/dw_2 ... dx_1/dw_m ;
%       dx_2/dw_1 dx_2/dw_2 ....dx_2/dw_m ;
%       ...                               ;
%       dx_n/dw_1 dx_n/dw_2 ....dx_n/dw_m ];

L = sym('L',[state_dim,state_noise_dim]);

for i=1:state_noise_dim
    L(:,i) = diff(f_out,state_noise(i));
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

for i=1:size(L,1)
    for j=1:size(L,2)
        if ~isAlways(L(i,j)==0,Unknown="false")
            fprintf('L(%d,%d) =%s;\n',i-1,j-1,L(i,j));
        end
    end
end

%% Look at: P_pred = F_val*P_last*F_val'+L_val*Q_variable{k}*L_val'

P_last = sym('P',state_dim);
for i=1:size(P_last,1)
    for j=1:size(P_last,2)
    P_last(i,j) = str2sym(sprintf('P(%d,%d)',i-1,j-1));
    end
end

Q = diag(state_noise);
F_simplified = subs(F,state_noise,zeros(state_noise_dim,1));

P_pred = F_simplified*P_last*F_simplified'+L*Q*L';


for i=1:size(P_pred,1)
    for j=1:size(P_pred,2)
        if ~isAlways(P_pred(i,j)==0,Unknown="false")
            fprintf('P_pred(%d,%d) = %s;\n',i-1,j-1,P_pred(i,j));
        end
    end
end

%% Define g

% x = [u v w mu_x mu_y mu_z k_x k_y k_z];
% u = [a_x a_y a_z p q r phi theta psi RPM_pusher RPM_hover skew elevator_angle];
% z = [V_x V_y V_z a_x a_y a_z];

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
Fx_hprop = k1_Fx_hprop.*u.^2.*sign_u+k2_Fx_hprop.*((RPM_hover_1+RPM_hover_2+RPM_hover_3+RPM_hover_4)./4).^2.*sqrt(u).*sign_u;

Fx_w = ((k1_Fx_w+k5_Fx_w.*skew)+(k2_Fx_w.*alpha+k3_Fx_w.*alpha.^2).*(sin(skew).^2+k4_Fx_w)).*V_a.^2;
Fx_elev = (k1_Fx_elev+k2_Fx_elev.*elevator_angle+k3_Fx_elev.*elevator_angle.^2).*V_a.^2;
a_x_filt = (Fx_push+Fx_fus+Fx_hprop+Fx_elev+Fx_w+k_x.*u.^2)./m;

% A_y
beta = asin(v/V_a);

%k_beta = -2E-1;
a_y_filt = beta.*k_beta.*(V_a.^2) + k_y;

% A_z
Fz_hprop = k1_Fz_hprop.*((RPM_hover_1+RPM_hover_2+RPM_hover_3+RPM_hover_4)./4).^2;
Fz_fus = (k1_Fz_fus.*cos(skew)+k2_Fz_fus+k3_Fz_fus.*alpha+k4_Fz_fus.*alpha.^2).*V_a.^2;
Fz_w = ((k1_Fz_w+k2_Fz_w.*alpha+k3_Fz_w.*alpha.^2).*(sin(skew).^2+k4_Fz_w)).*V_a.^2;
Fz_elev = (k1_Fz_elev+k2_Fz_elev.*elevator_angle).*V_a.^2;

a_z_filt = (Fz_fus+Fz_w+Fz_elev+Fz_hprop)./m + k_z;

g_out = [speed;a_x_filt;a_y_filt;a_z_filt;u]+z_noise;

%% Define g (simplified version)

% x = [u v w mu_x mu_y mu_z k_x k_y k_z];
% u = [a_x a_y a_z p q r phi theta psi RPM_pusher RPM_hover skew elevator_angle];
% z = [V_x V_y V_z a_x a_y a_z];

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
drag = k1_drag.*u + k2_drag.*u.^2.*sign_u;
a_x_filt = (Fx_push+drag+k_x.*u.^2.*sign_u)./m;

% A_y
beta = asin(v/V_a);

a_y_filt = beta.*k_beta.*(u.^2) + k_y;

% A_z
Fz_hprop = k1_Fz_hprop.*((RPM_hover_1+RPM_hover_2+RPM_hover_3+RPM_hover_4)./4).^2;
Fz_fus = k_Fz_fus.*u.^2;
Fz_w = k_Fz_w.*u.^2;
Fz_elev = k_Fz_elev.*u.^2;

a_z_filt = (Fz_fus+Fz_w+Fz_elev+Fz_hprop)./m + k_z;

g_out = [speed;a_x_filt;a_y_filt;a_z_filt;u]+z_noise;

%% Define g (simplified version, with k_v)

% x = [u v w mu_x mu_y mu_z k_x k_y k_z];
% u = [a_x a_y a_z p q r phi theta psi RPM_pusher RPM_hover skew elevator_angle];
% z = [V_x V_y V_z a_x a_y a_z];

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
drag = k1_drag.*u + k2_drag.*u.^2.*sign_u;
a_x_filt = (Fx_push+drag+k_x.*u.^2.*sign_u)./m;

% A_y
beta = asin(v/V_a);

a_y_filt = sign_v*v.^2.*k_v + k_y;

% A_z
Fz_hprop = k1_Fz_hprop.*((RPM_hover_1+RPM_hover_2+RPM_hover_3+RPM_hover_4)./4).^2;
Fz_fus = k_Fz_fus.*u.^2;
Fz_w = k_Fz_w.*u.^2;
Fz_elev = k_Fz_elev.*u.^2;

a_z_filt = (Fz_fus+Fz_w+Fz_elev+Fz_hprop)./m + k_z;

g_out = [speed;a_x_filt;a_y_filt;a_z_filt;u]+z_noise;

%% Get G (d g/dx)
% G = [] nxm where n=number of outputs, m=number of states
%
% G = [ dg_1/dx_1 dg_1/dx_2 ... dg_1/dx_m ;
%       dg_2/dx_1 dg_2/dx_2 ....dg_2/dx_m ;
%       ...                               ;
%       dg_n/dx_1 dg_n/dx_2 ....dg_n/dx_m ];

G = sym('G',[length(g_out), length(x)]);

for i=1:length(x)
    G(:,i) = diff(g_out,x(i));
end

G

% G =
%  
% [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 cos(psi)*cos(theta),                                                                                                                                                                                                                          cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi),                                                                                                                                                                                                                                                                                                                                                                                                                                          sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta), 1, 0, 0,     0, 0, 0]
% [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 cos(theta)*sin(psi),                                                                                                                                                                                                                          cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta),                                                                                                                                                                                                                                                                                                                                                                                                                                          cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi), 0, 1, 0,     0, 0, 0]
% [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         -sin(theta),                                                                                                                                                                                                                                                       cos(theta)*sin(phi),                                                                                                                                                                                                                                                                                                                                                                                                                                                                       cos(phi)*cos(theta), 0, 0, 1,     0, 0, 0]
% [(k3_Fx_push + RPM_pusher*k2_Fx_push + 2*k1_Fx_hprop*u + 2*k_x*u + 2*u*(k2_Fx_fus + k4_Fx_fus*atan(w/u)^2 + k1_Fx_fus*cos(skew) + k3_Fx_fus*atan(w/u)) + 2*u*(k3_Fx_elev*elevator_angle^2 + k2_Fx_elev*elevator_angle + k1_Fx_elev) - ((k3_Fx_fus*w)/(u^2*(w^2/u^2 + 1)) + (2*k4_Fx_fus*w*atan(w/u))/(u^2*(w^2/u^2 + 1)))*(u^2 + v^2 + w^2) + (0.5000*k2_Fx_hprop*(0.2500*RPM_hover_1 + 0.2500*RPM_hover_2 + 0.2500*RPM_hover_3 + 0.2500*RPM_hover_4)^2)/u^0.5000 - (sin(skew)^2 + k4_Fx_w)*((k2_Fx_w*w)/(u^2*(w^2/u^2 + 1)) + (2*k3_Fx_w*w*atan(w/u))/(u^2*(w^2/u^2 + 1)))*(u^2 + v^2 + w^2) + 2*u*(sin(skew)^2 + k4_Fx_w)*(k3_Fx_w*atan(w/u)^2 + k1_Fx_w*(k5_Fx_w*skew + 1) + k2_Fx_w*atan(w/u)))/m, (2*v*(k2_Fx_fus + k4_Fx_fus*atan(w/u)^2 + k1_Fx_fus*cos(skew) + k3_Fx_fus*atan(w/u)) + 2*v*(k3_Fx_elev*elevator_angle^2 + k2_Fx_elev*elevator_angle + k1_Fx_elev) + 2*v*(sin(skew)^2 + k4_Fx_w)*(k3_Fx_w*atan(w/u)^2 + k1_Fx_w*(k5_Fx_w*skew + 1) + k2_Fx_w*atan(w/u)))/m, (2*w*(k2_Fx_fus + k4_Fx_fus*atan(w/u)^2 + k1_Fx_fus*cos(skew) + k3_Fx_fus*atan(w/u)) + 2*w*(k3_Fx_elev*elevator_angle^2 + k2_Fx_elev*elevator_angle + k1_Fx_elev) + (k3_Fx_fus/(u*(w^2/u^2 + 1)) + (2*k4_Fx_fus*atan(w/u))/(u*(w^2/u^2 + 1)))*(u^2 + v^2 + w^2) + (sin(skew)^2 + k4_Fx_w)*(k2_Fx_w/(u*(w^2/u^2 + 1)) + (2*k3_Fx_w*atan(w/u))/(u*(w^2/u^2 + 1)))*(u^2 + v^2 + w^2) + 2*w*(sin(skew)^2 + k4_Fx_w)*(k3_Fx_w*atan(w/u)^2 + k1_Fx_w*(k5_Fx_w*skew + 1) + k2_Fx_w*atan(w/u)))/m, 0, 0, 0, u^2/m, 0, 0]
% [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            2*k_beta*u*asin(v/(u^2 + v^2 + w^2)^0.5000) - (k_beta*u*v)/((1 - v^2/(u^2 + v^2 + w^2))^0.5000*(u^2 + v^2 + w^2)^0.5000),                                                                                                   2*k_beta*v*asin(v/(u^2 + v^2 + w^2)^0.5000) + (k_beta*(1/(u^2 + v^2 + w^2)^0.5000 - v^2/(u^2 + v^2 + w^2)^1.5000)*(u^2 + v^2 + w^2))/(1 - v^2/(u^2 + v^2 + w^2))^0.5000,                                                                                                                                                                                                                                                                                                                                                                  2*k_beta*w*asin(v/(u^2 + v^2 + w^2)^0.5000) - (k_beta*v*w)/((1 - v^2/(u^2 + v^2 + w^2))^0.5000*(u^2 + v^2 + w^2)^0.5000), 0, 0, 0,     0, 1, 0]
% [                                                                                                                                                                                                          (2*u*(k2_Fz_fus + k4_Fz_fus*atan(w/u)^2 + k1_Fz_fus*cos(skew) + k3_Fz_fus*atan(w/u)) + 2*u*(k3_Fz_elev*elevator_angle^2 + k2_Fz_elev*elevator_angle + k1_Fz_elev) - ((k3_Fz_fus*w)/(u^2*(w^2/u^2 + 1)) + (2*k4_Fz_fus*w*atan(w/u))/(u^2*(w^2/u^2 + 1)))*(u^2 + v^2 + w^2) - (sin(skew)^2 + k4_Fz_w)*((k2_Fz_w*w)/(u^2*(w^2/u^2 + 1)) + (2*k3_Fz_w*w*atan(w/u))/(u^2*(w^2/u^2 + 1)))*(u^2 + v^2 + w^2) + 2*u*(sin(skew)^2 + k4_Fz_w)*(k1_Fz_w + k3_Fz_w*atan(w/u)^2 + k2_Fz_w*atan(w/u)))/m,                    (2*v*(k2_Fz_fus + k4_Fz_fus*atan(w/u)^2 + k1_Fz_fus*cos(skew) + k3_Fz_fus*atan(w/u)) + 2*v*(k3_Fz_elev*elevator_angle^2 + k2_Fz_elev*elevator_angle + k1_Fz_elev) + 2*v*(sin(skew)^2 + k4_Fz_w)*(k1_Fz_w + k3_Fz_w*atan(w/u)^2 + k2_Fz_w*atan(w/u)))/m,                    (2*w*(k2_Fz_fus + k4_Fz_fus*atan(w/u)^2 + k1_Fz_fus*cos(skew) + k3_Fz_fus*atan(w/u)) + 2*w*(k3_Fz_elev*elevator_angle^2 + k2_Fz_elev*elevator_angle + k1_Fz_elev) + (k3_Fz_fus/(u*(w^2/u^2 + 1)) + (2*k4_Fz_fus*atan(w/u))/(u*(w^2/u^2 + 1)))*(u^2 + v^2 + w^2) + (sin(skew)^2 + k4_Fz_w)*(k2_Fz_w/(u*(w^2/u^2 + 1)) + (2*k3_Fz_w*atan(w/u))/(u*(w^2/u^2 + 1)))*(u^2 + v^2 + w^2) + 2*w*(sin(skew)^2 + k4_Fz_w)*(k1_Fz_w + k3_Fz_w*atan(w/u)^2 + k2_Fz_w*atan(w/u)))/m, 0, 0, 0,     0, 0, 1]
% [    

for i=1:size(G,1)
    for j=1:size(G,2)
        if ~isAlways(G(i,j)==0,Unknown="false")
            fprintf('G(%d,%d) = %s;\n',i-1,j-1,G(i,j));
        end
        
    end
end

string = 'u^2+v^2';
index = strfind(string,'^2');
new_string = string;
for i=1:length(index)
    new_string(index(i):index(i)+1) = ['*' new_string(index(i)-1)];
end

%% Get M (d g/dw)
% M = [] nxm where n=number of outputs, m=number of noise
%
% M = [ dg_1/dw_1 dg_1/dw_2 ... dg_1/dw_m ;
%       dg_2/dw_1 dg_2/dw_2 ....dg_2/dw_m ;
%       ...                               ;
%       dg_n/dw_1 dg_n/dw_2 ....dg_n/dw_m ];


M = sym('M',[length(g_out),length(z_noise)]);

for i=1:length(z_noise)
    M(:,i) = diff(g_out,z_noise(i));
end

M

% M =
%  
% [1, 0, 0, 0, 0, 0, 0]
% [0, 1, 0, 0, 0, 0, 0]
% [0, 0, 1, 0, 0, 0, 0]
% [0, 0, 0, 1, 0, 0, 0]
% [0, 0, 0, 0, 1, 0, 0]
% [0, 0, 0, 0, 0, 1, 0]
% [0, 0, 0, 0, 0, 0, 1]

%% Look at: S = G_val*P_pred*G_val'+M_val*R_variable{k}*M_val'

G_simplified = subs(G,[z_noise],[zeros(z_noise_dim,1)]);
%G_simplified = subs(G,[z_noise;phi;theta],[zeros(z_noise_dim,1);0;0]);

P_pred = sym('P',state_dim);
for i=1:size(P_pred,1)
    for j=1:size(P_pred,2)
    P_pred(i,j) = str2sym(sprintf('P_pred(%d,%d)',i-1,j-1));
    end
end

R = diag(z_noise);

S = G_simplified*P_pred*G_simplified'+M*R*M';

for i=1:size(S,1)
    for j=1:size(S,2)
            fprintf('S(%d,%d) = %s;\n',i-1,j-1,S(i,j));
    end
end

