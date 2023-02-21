function [EKF_res] = run_EKF(epsi,t,Q,R,P_0,x_0,u_list,z_list,f_fh,g_fh)
%RUN_EKF Run an EKF with existing data
%   Detailed explanation goes here

dt = mean(t(2:end)-t(1:end-1));

EKF_res = {};

x_list = zeros(size(x_0,1),length(t));
y_list = zeros(size(z_list,1),length(z_list));

K = cell(1,length(t));
P = cell(1,length(t));

for k=1:length(t)
    if k==1
        x = x_0;
        P_last = P_0;
    else
        x=x_list(:,k-1);
        P_last = P{k-1};
    end
        
    u=u_list(:,k);
    z=z_list(:,k);

    F_val = F(f_fh,x,u,epsi);
    G_val = G(g_fh,x,u,epsi);
    L_val = L(f_fh,x,u,epsi);
    M_val = M(g_fh,x,u,epsi);

    % Prediction
    x_pred = x + dt*f_fh(x,u);      
    P_pred = F_val*P_last*F_val'+L_val*Q*L_val';

    % Update
    y_list(:,k) = z-g_fh(x,u);

    %Kalman gain
    K{k} = P_pred*G_val'*inv(G_val*P_pred*G_val'+M_val*R*M_val');

    x_list(:,k) = x_pred+K{k}*y_list(:,k);
    P{k} = (eye(length(x))-K{k}*G_val)*P_pred;

end

EKF_res.t = t;
EKF_res.x = x_list;
EKF_res.y = y_list;
EKF_res.u = u_list;
EKF_res.z = z_list;
EKF_res.P = P;
EKF_res.Q = Q;
EKF_res.R = R;
EKF_res.K = K;

end