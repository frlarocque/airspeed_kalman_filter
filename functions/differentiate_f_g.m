%% Init
clear all
close all
clc

%% Define syms for f

syms u v w mu_x mu_y mu_z k_x k_y k_z a_x a_y a_z p q r phi theta psi RPM_pusher hover_RPM_mean skew elevator_angle
syms sign_u sign_v
syms g
syms RPM_hover_1 RPM_hover_2 RPM_hover_3 RPM_hover_4
syms w_accel_x w_accel_y w_accel_z w_gyro_x w_gyro_y w_gyro_z w_mu_x w_mu_y w_mu_z w_k_x w_k_y w_k_z

assume([u v w mu_x mu_y mu_z k_x k_y k_z a_x a_y a_z p q r phi theta psi RPM_pusher hover_RPM_mean skew elevator_angle g sign_u sign_v],'real')
assume([w_accel_x w_accel_y w_accel_z w_gyro_x w_gyro_y w_gyro_z w_mu_x w_mu_y w_mu_z w_k_x w_k_y w_k_z],'real')
assume([RPM_hover_1 RPM_hover_2 RPM_hover_3 RPM_hover_4],'real')
%% Define var for g
syms V_gnd_x V_gnd_y V_gnd_z a_x_filt a_y_filt a_z_filt V_pitot
syms w_V_gnd_x w_V_gnd_y w_V_gnd_z w_a_x_filt w_a_y_filt w_a_z_filt w_V_pitot
syms m 
syms alpha beta
assume([alpha beta],'real')

syms k1_fx_push k2_fx_push k3_fx_push
syms k1_fx_fuselage k2_fx_fuselage k3_fx_fuselage k4_fx_fuselage
syms k1_fx_hover k2_fx_hover k3_fx_hover
syms k1_fx_wing k2_fx_wing k3_fx_wing k4_fx_wing k5_fx_wing
syms k1_fx_elev k2_fx_elev k3_fx_elev


syms k_fy_beta k_fy_v
syms k1_fy_wing

syms k1_fz_hover k5_fz_hover
syms k1_fz_fuselage k2_fz_fuselage k3_fz_fuselage k4_fz_fuselage
syms k1_fz_wing k2_fz_wing k3_fz_wing k4_fz_wing
syms k1_fz_elev k2_fz_elev k3_fz_elev

syms k1_drag k2_drag Fz_hprop k_fz_fuselage k_fz_wing k_fz_elev

assume([V_gnd_x V_gnd_y V_gnd_z a_x_filt a_y_filt a_z_filt V_pitot],'real')
assume([w_V_gnd_x w_V_gnd_y w_V_gnd_z w_a_x_filt w_a_y_filt w_a_z_filt w_V_pitot m],'real')
assume([k1_fx_push k2_fx_push k3_fx_push k1_fx_fuselage k2_fx_fuselage k3_fx_fuselage k4_fx_fuselage k1_fx_hover k2_fx_hover k3_fx_hover k1_fx_wing k2_fx_wing k3_fx_wing k4_fx_wing k5_fx_wing k1_fx_elev k2_fx_elev k3_fx_elev],'real')
assume([k_fy_beta,k_fy_v],'real')
assume([k1_fz_hover k5_fz_hover k1_fz_fuselage k2_fz_fuselage k3_fz_fuselage k4_fz_fuselage k1_fz_wing k2_fz_wing k3_fz_wing k4_fz_wing k1_fy_wing k1_fz_elev k2_fz_elev k3_fz_elev],'real')
assume([k1_drag k2_drag Fz_hprop k_fz_fuselage k_fz_wing k_fz_elev],'real')

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

Q = sym('Q',state_noise_dim);
for i=1:size(Q,1)
    for j=1:size(Q,2)
        if i~=j
            Q(i,j) = 0;
        else
            Q(i,j) = str2sym(sprintf('Q(%d,%d)',i-1,j-1));
        end
    end
end

F_simplified = subs(F,state_noise,zeros(state_noise_dim,1));

P_pred = F_simplified*P_last*F_simplified'+L*Q*L';

for i=1:size(P_pred,1)
    for j=1:size(P_pred,2)
        if ~isAlways(P_pred(i,j)==0,Unknown="false")
            fprintf('eawp.P(%d,%d) = %s;\n',i-1,j-1,simplifyForCode(P_pred(i,j)));
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

Fx_push = k1_fx_push.*RPM_pusher.^2+k2_fx_push.*RPM_pusher.*u+k3_fx_push.*u;
Fx_fus = (k1_fx_fuselage.*cos(skew)+k2_fx_fuselage+k3_fx_fuselage.*alpha+k4_fx_fuselage.*alpha.^2).*V_a.^2;
Fx_hprop = k1_fx_hover.*u.^2.*sign_u+k2_fx_hover.*hover_RPM_mean.^2.*sqrt(u).*sign_u + k3_fx_hover.*u.*sign_u;

Fx_w = ((k1_fx_wing+k5_fx_wing.*skew)+(k2_fx_wing.*alpha+k3_fx_wing.*alpha.^2).*(sin(skew).^2+k4_fx_wing)).*V_a.^2;
Fx_elev = (k1_fx_elev+k2_fx_elev.*elevator_angle+k3_fx_elev.*elevator_angle.^2).*V_a.^2;
a_x_filt = (Fx_push+Fx_fus+Fx_hprop+Fx_elev+Fx_w+k_x.*u.^2)./m;

% A_y
Fy_w = k1_fy_wing.*cos(skew).*sin(skew).*u.^2;
a_y_filt = (sign_v*v.^2.*k_fy_v + Fy_w)./m +k_y;

% A_z
Fz_hprop = k1_fz_hover.*hover_RPM_mean.^2 + k5_fz_hover.*u.*sign_u;
Fz_fus = (k1_fz_fuselage.*cos(skew)+k2_fz_fuselage+k3_fz_fuselage.*alpha+k4_fz_fuselage.*alpha.^2).*V_a.^2;
Fz_w = ((k1_fz_wing+k2_fz_wing.*alpha+k3_fz_wing.*alpha.^2).*(sin(skew).^2+k4_fz_wing)).*V_a.^2;
Fz_elev = (k1_fz_elev+k2_fz_elev.*elevator_angle).*V_a.^2;

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

Fx_push = k1_fx_push.*RPM_pusher.^2+k2_fx_push.*RPM_pusher.*u+k3_fx_push.*u;
drag = k1_drag.*u + k2_drag.*u.^2.*sign_u;
a_x_filt = (Fx_push+drag+k_x.*u.^2.*sign_u)./m;

% A_y
beta = asin(v/V_a);

a_y_filt = beta.*k_fy_beta.*(u.^2) + k_y;

% A_z
Fz_hprop = k1_fz_hover.*hover_RPM_mean.^2;
Fz_fus = k_fz_fuselage.*u.^2;
Fz_w = k_fz_wing.*u.^2;
Fz_elev = k_fz_elev.*u.^2;

a_z_filt = (Fz_fus+Fz_w+Fz_elev+Fz_hprop)./m + k_z;

g_out = [speed;a_x_filt;a_y_filt;a_z_filt;u]+z_noise;

%% Define g (simplified version, with k_fy_v)

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

Fx_push = k1_fx_push.*RPM_pusher.^2+k2_fx_push.*RPM_pusher.*u+k3_fx_push.*u;
drag = k1_drag.*u + k2_drag.*u.^2.*sign_u;
a_x_filt = (Fx_push+drag+k_x.*u.^2.*sign_u)./m;

% A_y
beta = asin(v/V_a);

a_y_filt = sign_v*v.^2.*k_fy_v + k_y;

% A_z
Fz_hprop = k1_fz_hover.*hover_RPM_mean.^2;
Fz_fus = k_fz_fuselage.*u.^2;
Fz_w = k_fz_wing.*u.^2;
Fz_elev = k_fz_elev.*u.^2;

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
G_fus = sym('G',[length(g_out), length(x)]);
G_wing = sym('G',[length(g_out), length(x)]);
G_elevator = sym('G',[length(g_out), length(x)]);
G_pusher = sym('G',[length(g_out), length(x)]);
G_hover = sym('G',[length(g_out), length(x)]);

for i=1:length(x)
    G(:,i) = diff(g_out,x(i));
    G_fus(:,i) = diff([0;0;0;Fx_fus./m;0;Fz_fus./m;0],x(i));
    G_w(:,i) = diff([0;0;0;Fx_w./m;Fy_w./m;Fz_w./m;0],x(i));
    G_elev(:,i) = diff([0;0;0;Fx_elev./m;0;Fz_elev./m;0],x(i));
    G_push(:,i) = diff([0;0;0;Fx_push./m;0;0;0],x(i));
    G_hprop(:,i) = diff([0;0;0;Fx_hprop./m;0;Fz_hprop./m;0],x(i));
end

clear alpha; syms alpha; assume(alpha,'real')
clear beta; syms beta; assume(beta,'real')
clear V_a; syms V_a; assume(V_a,'real')
syms diff_alpha_u; assume(diff_alpha_u,'real')
syms diff_alpha_w; assume(diff_alpha_w,'real')

fprintf('-----ALL TOGETHER-----\n')
for i=1:size(G,1)
    for j=1:size(G,2)
        if ~isAlways(G(i,j)==0,Unknown="false")
            fprintf('G(%d,%d) = %s;\n',i-1,j-1,simplifyForCode(G(i,j),u,v,w,alpha,V_a,beta,diff_alpha_u,diff_alpha_w));
        end
    end
end

fprintf("-----SEPARATE-----\n")
fprintf("\nFUSELAGE CONTRIBUTION\n")
for i=1:size(G,1)
    for j=1:size(G,2)
        if ~isAlways(G_fus(i,j)==0,Unknown="false")
            fprintf('G(%d,%d) += %s;\n',i-1,j-1,simplifyForCode(G_fus(i,j),u,v,w,alpha,V_a,beta,diff_alpha_u,diff_alpha_w));
        end
    end
end

fprintf("\nWING CONTRIBUTION\n")
for i=1:size(G,1)
    for j=1:size(G,2)
        if ~isAlways(G_w(i,j)==0,Unknown="false")
            fprintf('G(%d,%d) += %s;\n',i-1,j-1,simplifyForCode(G_w(i,j),u,v,w,alpha,V_a,beta,diff_alpha_u,diff_alpha_w));
        end
    end
end

fprintf("\nELEVATOR CONTRIBUTION\n")
for i=1:size(G,1)
    for j=1:size(G,2)
        if ~isAlways(G_elev(i,j)==0,Unknown="false")
            fprintf('G(%d,%d) += %s;\n',i-1,j-1,simplifyForCode(G_elev(i,j),u,v,w,alpha,V_a,beta,diff_alpha_u,diff_alpha_w));
        end
    end
end

fprintf("\nPUSHER CONTRIBUTION\n")
for i=1:size(G,1)
    for j=1:size(G,2)
        if ~isAlways(G_push(i,j)==0,Unknown="false")
            fprintf('G(%d,%d) += %s;\n',i-1,j-1,simplifyForCode(G_push(i,j),u,v,w,alpha,V_a,beta,diff_alpha_u,diff_alpha_w));
        end
    end
end

fprintf("\nHOVER PROP CONTRIBUTION\n")
for i=1:size(G,1)
    for j=1:size(G,2)
        if ~isAlways(G_hprop(i,j)==0,Unknown="false")
            fprintf('G(%d,%d) += %s;\n',i-1,j-1,simplifyForCode(G_hprop(i,j),u,v,w,alpha,V_a,beta,diff_alpha_u,diff_alpha_w));
        end
    end
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
G_simplified = sym('G',[length(g_out), length(x)],'real');
for i=1:size(G_temp,1)
    for j=1:size(G_temp,2)
        if ~isAlways(G_temp(i,j)==0,Unknown="false")
        G_simplified(i,j) = str2sym(sprintf('GG(%d,%d)',i-1,j-1));
        else
        G_simplified(i,j) = 0;
        end
    end
end

P_last = sym('P',state_dim);
for i=1:size(P_last,1)
    for j=1:size(P_last,2)
    P_last(i,j) = str2sym(sprintf('P(%d,%d)',i-1,j-1));
    end
end

R = sym('R',z_noise_dim);
for i=1:size(R,1)
    for j=1:size(R,2)
        if i~=j
            R(i,j) = 0;
        else
            R(i,j) = str2sym(sprintf('R(%d,%d)',i-1,j-1));
        end
    end
end

S = G_simplified*P_last*G_simplified'+M*R*M';

for i=1:size(S,1)
    for j=1:size(S,2)
        if ~isAlways(S(i,j)==0,Unknown="false")
            fprintf('S(%d,%d) = %s;\n',i-1,j-1,simplifyForCode(S(i,j)));
        end
    end
end
%%
function str = simplifyForCode(expr,u,v,w,alpha,V_a,beta,diff_alpha_u,diff_alpha_w)
    
    if nargin~=1
        expr = subs(expr,atan(w/u),alpha);
        expr = subs(expr,sqrt(u.^2+v.^2+w.^2),V_a);
        expr = subs(expr,(u.^2+v.^2+w.^2),V_a^2);
        expr = subs(expr,asin(v/sqrt(u.^2+v.^2+w.^2)),beta);
        expr = subs(expr,asin(v/V_a),beta);
        expr = subs(expr,v/V_a,sin(beta));
        expr = subs(expr,1-sin(beta)^2,cos(beta)^2);
        expr = subs(expr,(u^2*(w^2/u^2 + 1)),-w/diff_alpha_u);
        expr = subs(expr,1/(u*(w^2/u^2 + 1)),diff_alpha_w);
        expr = simplify(expr);
    end
    str = sprintf("%s",expr);
    % Replace cos() by cos_ and sin() by sin_
    str = regexprep(str, "(?<!a)cos\((\w+)\)", "cos_$1");
    str = regexprep(str, "(?<!a)sin\((\w+)\)", "sin_$1");
    
    % Pad multiplication operators with whitespaces
    str = regexprep(str, '(\S)\*(\S)', '$1 * $2');
    
    % Replace any ^2 by a square
    str = regexprep(str, '(\w+)\^2', '$1*$1');
    
    % Adapt coefficients
    str = regexprep(str, "k(\d+)_(\w+)_(\w+)", "ekf_aw_params.k_$2_$3[$1-1]");

    % Adapt m
    str = regexprep(str, '(?<!\w)m(?!\w)', 'ekf_aw_params.vehicle_mass');

    % Adapt P
    str = regexprep(str, '(?<!\w)P(?!\w)', 'eawp.P');

    % Adapt Q
    str = regexprep(str, '(?<!\w)Q(?!\w)', 'eawp.Q');

    % Adapt R
    str = regexprep(str, '(?<!\w)R(?!\w)', 'eawp.R');

    % Adapt GG
    str = regexprep(str, '(?<!\w)GG(?!\w)', 'G');

    % Adapt Skew
    str = regexprep(str, '(?<!\w)skew(?!\w)', 'eawp.inputs.skew');

    % Adapt RPM_pusher
    str = regexprep(str, '(?<!\w)RPM_pusher(?!\w)', 'eawp.inputs.RPM_pusher');

    % Adapt elevator_angle
    str = regexprep(str, '(?<!\w)elevator_angle(?!\w)', 'eawp.inputs.elevator_angle');
    
    % Use regular expressions to extract the numbers in the format "[1-2]"
    matches = regexp(str, '\[(\d+)-(\d+)\]', 'tokens');
    
    % Loop through all the matches and replace the matched substring with the result
    for i = 1:length(matches)
        num1 = str2double(matches{i}{1});
        num2 = str2double(matches{i}{2});
        result = num1 - num2;
        str = regexprep(str, ['\[',matches{i}{1},'-',matches{i}{2},'\]'], ['[',num2str(result),']']);
    end

end