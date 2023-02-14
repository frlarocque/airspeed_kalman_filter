classdef EKF < handle

    properties
        epsi
        dt
        Q
        R
        P
        K
        x_0
        P_0
        x_list
        u_list
        y_list
        z_list
        f_fh
        g_fh
    end

    methods
        function obj = EKF(epsi,dt,Q,R,P_0,x_0,f_fh,g_fh)
            obj.epsi = epsi;
            obj.dt = dt;

            obj.Q = Q;
            obj.R = R;
            obj.P_0 = P_0;
            obj.P = {};
            obj.K = {};

            obj.f_fh = f_fh;
            obj.g_fh = g_fh;

            obj.x_0 = x_0;
            obj.x_list = [] ;
            obj.y_list = [];
        end

        function update_EKF(obj,u,z)
            if isempty(obj.x_list)
                x = obj.x_0;
                k=1;
                P_last = obj.P_0;
            else
                x=obj.x_list(:,end);
                k = size(obj.x_list,2)+1;
                P_last = obj.P{k-1};
            end
            
            obj.u_list(:,k) = u;
            obj.z_list(:,k) = z;
        
            F_val = state_jacobian(obj,x,u);
            G_val = output_jacobian(obj,x,u);
        
            % Prediction
            x_pred = x + obj.dt*obj.f_fh(x,u);   
            P_pred = F_val*P_last*F_val'+obj.Q;
        
            % Update
            obj.y_list(:,k) = z-obj.g_fh(x,u);
        
            %Kalman gain
            obj.K{k} = P_pred*G_val'*inv(G_val*P_pred*G_val'+obj.R);
        
            obj.x_list(:,k) = x_pred+obj.K{k}*obj.y_list(:,k);
            obj.P{k} = (eye(length(x))-obj.K{k}*G_val)*P_pred;
        end

        function F = state_jacobian(obj,x,u)
            F = zeros(length(x),length(x));
            for i=1:length(x)
                
               x_p = x;x_p(i)=x_p(i)+obj.epsi;
               x_m = x;x_m(i)=x_m(i)-obj.epsi;
            
               df = (obj.f_fh(x_p,u)-obj.f_fh(x_m,u))/(2*obj.epsi);
                
               F(:,i) = df;
            end
        end

        function G = output_jacobian(obj,x,u)
            y = obj.g_fh(x,u);
            G = zeros(length(y),length(x));
            for i=1:length(x)
                
               x_p = x;x_p(i)=x_p(i)+obj.epsi;
               x_m = x;x_m(i)=x_m(i)-obj.epsi;
            
               dg = (obj.g_fh(x_p,u)-obj.g_fh(x_m,u))./(2*obj.epsi);
                
               G(:,i) = dg;
            end
        end

    end



end


%% To run EKF object oriented (really slow)
% epsi = 1E-2;
% t = airspeed_pitot.flight.time;
% dt = mean(t(2:end)-t(1:end-1));
% R = diag([Vg_NED.var IMU_angle.var(2) 1E-6]); %measurement noise
% x_0 =  [0, 0, 0,  0,  0,   0]';
% f_fh = str2func('f_2'); %with beta and alpha, without euler angle estimation
% g_fh = str2func('g_2'); %with beta and alpha, without euler angle estimation
% u_list = [IMU_accel.flight.data IMU_rate.flight.data IMU_angle.flight.data]';
% z_list = [Vg_NED.flight.data alpha.flight.data beta.flight.data]'; %measurement
% 
% cov_list = 1E-8;%logspace(-9,-5,15);
% kalman_res = cell(size(cov_list));
% %kalman_res = repmat({struct([])}, size(cov_list,1), size(cov_list,2));
% 
% for i=1:length(cov_list)
%     clear filter
%     wind_var = [1 1 1]*cov_list(i);
%     Q = diag([IMU_accel.var,wind_var]); %process noise
%     P_0 = diag([1 1 1 wind_var]); %covariance
%     
%     filter = EKF(epsi,dt,Q,R,P_0,x_0,f_fh,g_fh);
% 
%     for k=1:length(t)
%     update_EKF(filter,u_list(:,k),z_list(:,k));
%     end
%     
%     for fn = fieldnames(filter)'
%         kalman_res{i}.(erase(fn{1},"_list")) = filter.(fn{1});
%     end
% end