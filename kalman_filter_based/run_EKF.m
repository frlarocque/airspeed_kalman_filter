function [EKF_res] = run_EKF(epsi,t,Q,R,P_0,x_0,u_list,z_list,f_fh,g_fh,show_waitbar)
%RUN_EKF Run an EKF with existing data
%   Detailed explanation goes here

if nargin<11
    show_waitbar = false;
end

global EKF_AW_AZ_QUICK_CONV_ACCEL_GAIN EKF_AW_AZ_QUICK_CONV_MU_GAIN EFK_AW_QUICK_CONVERGENCE EKF_AW_QUICK_CONVERGENCE_TIME
global EKF_AW_AZ_SCHED_GAIN EKF_AW_AZ_SCHED_START_DEG EKF_AW_AZ_SCHED_END_DEG
global EKF_AW_AX_SCHED_GAIN EKF_AW_AX_SCHED_START_DEG EKF_AW_AX_SCHED_END_DEG
global EKF_AW_USE_MODEL_BASED EKF_AW_USE_BETA EKF_AW_WING_INSTALLED EKF_AW_PROPAGATE_OFFSET EKF_AW_VEHICLE_MASS EKF_AW_USE_PITOT 
global EKF_AW_AX_INNOV_GATE EKF_AW_AY_INNOV_GATE EKF_AW_AZ_INNOV_GATE EKF_AW_V_GPS_INNOV_GATE

dt = mean(t(2:end)-t(1:end-1));

fprintf('Duration %2.2f s\n',t(end)-t(1))
EKF_res = {};

x_list = zeros(size(x_0,1),length(t));
y_list = zeros(size(z_list,1),length(z_list));

K = cell(1,length(t));
P = cell(1,length(t));
S = cell(1,length(t));
R_variable = cell(1,length(t));
Q_variable = cell(1,length(t));
innov_gate_state = zeros(size(z_list));

if show_waitbar
h = waitbar(0,'Please wait...');
end

flag_quick_convergence = false;
for k=1:length(t)
    if show_waitbar
    waitbar(k / length(t))
    end

    u=u_list(:,k);
    z=z_list(:,k);

    if k==1
        x = x_0;
        P_last = P_0;
    else
        x=x_list(:,k-1);
        P_last = P{k-1};
    end

    Q_variable{k} = Q.*dt;
    R_variable{k} = R;

    if t(k)-t(1)>50
    fprintf('')
    end

    %     // A_z covariance Scheduling
    %   /*
    %               Cov
    %               ▲
    %       10^Gain │ ────────
    %               │         \
    %               │          \
    %               │           \
    %               │            \
    %           1   │             ────────
    %               │
    %               └─────────┬───┬──────► Skew Angle
    %                         │   │
    %                      Start  End  
    %   
    %   */
    exp_az = 0;
    if (u(12)<deg2rad(EKF_AW_AZ_SCHED_START_DEG))
        exp_az = EKF_AW_AZ_SCHED_GAIN;
    elseif (u(12)>deg2rad(EKF_AW_AZ_SCHED_END_DEG))
    exp_az = 0;
    else
    exp_az = EKF_AW_AZ_SCHED_GAIN+(u(12)-deg2rad(EKF_AW_AZ_SCHED_START_DEG))*((EKF_AW_AZ_SCHED_GAIN)/(deg2rad(EKF_AW_AZ_SCHED_START_DEG-EKF_AW_AZ_SCHED_END_DEG)));
    end
    exp_az = max(min(exp_az,EKF_AW_AZ_SCHED_GAIN),0);
    R_variable{k}(6,6) = 10.^exp_az*R_variable{k}(6,6);
    
    %   // A_x covariance Scheduling
    %   /*
    %               Cov
    %               ▲
    %       10^Gain │              /────────
    %               │             /
    %               │            /
    %               │           / 
    %               │          /  
    %           1   │ ────────/   
    %               │
    %               └─────────┬───┬──────► Skew Angle
    %                         │   │
    %                      Start  End  
    %   
    %   */
      exp_ax = 0;
      if (u(12)<deg2rad(EKF_AW_AX_SCHED_START_DEG))
        exp_ax = 0;
      elseif (u(12)>deg2rad(EKF_AW_AX_SCHED_END_DEG))
        exp_ax = EKF_AW_AX_SCHED_GAIN;
      else
        exp_ax = (u(12)-deg2rad(EKF_AW_AX_SCHED_START_DEG))*((EKF_AW_AX_SCHED_GAIN)/(deg2rad(EKF_AW_AX_SCHED_END_DEG-EKF_AW_AX_SCHED_START_DEG)));
      end
      exp_ax = max(min(exp_ax,EKF_AW_AX_SCHED_GAIN),0);
      R_variable{k}(4,4) = 10.^exp_ax*R_variable{k}(4,4);

    if t(k)-t(1)<EKF_AW_QUICK_CONVERGENCE_TIME && EFK_AW_QUICK_CONVERGENCE
        flag_quick_convergence = true;
        Q_variable{k}(7,7) = 10.^EKF_AW_AZ_QUICK_CONV_MU_GAIN*Q_variable{k}(7,7); %increase wind covariance --> it can change faster
        Q_variable{k}(8,8) = 10.^EKF_AW_AZ_QUICK_CONV_MU_GAIN*Q_variable{k}(8,8);
        Q_variable{k}(9,9) = 10.^EKF_AW_AZ_QUICK_CONV_MU_GAIN*Q_variable{k}(9,9);

        R_variable{k}(4,4) = 10.^EKF_AW_AZ_QUICK_CONV_ACCEL_GAIN*R_variable{k}(4,4); %decrease a_x cov --> more weight put on it
        R_variable{k}(5,5) = 10.^EKF_AW_AZ_QUICK_CONV_ACCEL_GAIN*R_variable{k}(5,5); %decrease a_x cov --> more weight put on it
        R_variable{k}(6,6) = 10.^EKF_AW_AZ_QUICK_CONV_ACCEL_GAIN*R_variable{k}(6,6); %decrease a_x cov --> more weight put on it
    else
        flag_quick_convergence = false;
    end

    F_val = F(f_fh,x,u,epsi);
    G_val = G(g_fh,x,u,epsi);
    L_val = L(f_fh,x,u,epsi);
    M_val = M(g_fh,x,u,epsi);

    % Prediction
    x_pred = x + dt*f_fh(x,u);
    P_pred = F_val*P_last*F_val'+L_val*Q_variable{k}*L_val';
    
    % Innovation
    y_list(:,k) = z-g_fh(x,u);

    % Check innnovation
    innov_gate_state(1:end-1,k) = abs(y_list(1:end-1,k)) > [EKF_AW_V_GPS_INNOV_GATE.*ones(3,1);EKF_AW_AX_INNOV_GATE;EKF_AW_AY_INNOV_GATE;EKF_AW_AZ_INNOV_GATE] & ~flag_quick_convergence.*ones(6,1);
    
    % Calculate S
    S{k} = G_val*P_pred*G_val'+M_val*R_variable{k}*M_val';

    % Kalman gain
    K{k} = P_pred*G_val'*inv(S{k});

    % State update
    x_list(:,k) = x_pred;

    % V_gnd contribution
    if ~any(innov_gate_state(1:3,k))
        x_list(1:3,k) = x_list(1:3,k)+ K{k}(1:3,1:3)*y_list(1:3,k); % Update V_body
        x_list(4:6,k) = x_list(4:6,k)+ K{k}(4:6,1:3)*y_list(1:3,k); % Update mu
        if EKF_AW_PROPAGATE_OFFSET
            x_list(7:9,k) = x_list(7:9,k)+ K{k}(7:9,1:3)*y_list(1:3,k); % Update offset
        end
    end

    % A_filt contribution
    % Ax contribution
    if ~any(innov_gate_state(4,k))
        x_list(1:3,k) = x_list(1:3,k)+ K{k}(1:3,4)*y_list(4,k); % Update V_body
        x_list(4:6,k) = x_list(4:6,k)+ K{k}(4:6,4)*y_list(4,k); % Update mu
        if EKF_AW_PROPAGATE_OFFSET
            x_list(7:9,k) = x_list(7:9,k)+ K{k}(7:9,4)*y_list(4,k); % Update offset
        end
    end
    % Ay contribution
    if ~any(innov_gate_state(5,k))
        x_list(1:3,k) = x_list(1:3,k)+ K{k}(1:3,5)*y_list(5,k); % Update V_body
        x_list(4:6,k) = x_list(4:6,k)+ K{k}(4:6,5)*y_list(5,k); % Update mu
        if EKF_AW_PROPAGATE_OFFSET
            x_list(7:9,k) = x_list(7:9,k)+ K{k}(7:9,5)*y_list(5,k); % Update offset
        end
    end
    % Az contribution
    if ~any(innov_gate_state(6,k))
        x_list(1:3,k) = x_list(1:3,k)+ K{k}(1:3,6)*y_list(6,k); % Update V_body
        x_list(4:6,k) = x_list(4:6,k)+ K{k}(4:6,6)*y_list(6,k); % Update mu
        if EKF_AW_PROPAGATE_OFFSET
            x_list(7:9,k) = x_list(7:9,k)+ K{k}(7:9,6)*y_list(6,k); % Update offset
        end
    end

    % V_pitot contribution
    if EKF_AW_USE_PITOT
        x_list(1:3,k) = x_list(1:3,k)+ K{k}(1:3,7)*y_list(7,k); % Update V_body
        x_list(4:6,k) = x_list(4:6,k)+ K{k}(4:6,7)*y_list(7,k); % Update mu
        if EKF_AW_PROPAGATE_OFFSET
            x_list(7:9,k) = x_list(7:9,k)+ K{k}(7:9,7)*y_list(7,k); % Update offset
        end
    end

    P{k} = (eye(length(x))-K{k}*G_val)*P_pred;

end

EKF_res.t = t;
EKF_res.x = x_list;
EKF_res.y = y_list;
EKF_res.u = u_list;
EKF_res.z = z_list;
EKF_res.P = P;
EKF_res.Q = Q_variable;
EKF_res.R = R_variable;
EKF_res.K = K;
EKF_res.S = S;
EKF_res.innov_gates = innov_gate_state;

if show_waitbar
close(h);
end

end