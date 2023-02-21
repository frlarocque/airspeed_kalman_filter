classdef airspeed_estimator_force < handle

    properties
        density = 1.225;
        m = 6.5; % [m]
        kf_hover_pitch = [0.0280  -29.2535]; %[N/pwm] Based on RCTest Bench T-motor MN3515 400kV/Zubax 6S/APC 16x10'
        kf_hover_roll = [0.0452  -52.8497];  %[N/pwm] Based on RCTest Bench T-motor MN3520 400kv - Tarot 17x5CF - 6S - Zubax 6S
        kf_pusher = [0.0452  -52.8497];      %[N/pwm] Based on RCTest Bench T-motor MN3520 400kv - Tarot 17x5CF - 6S - Zubax 6S
        
        kf_motors = []; %[front right back left push];

        CL_body = [-0.6580 26.4910 0.0138]; % [1/rad] [CL_0 CL_alpha S_body] 
              
    end
    

    methods
        % Initialisation Method
        function obj = airspeed_estimator_force()
            obj.kf_motors = [obj.kf_hover_pitch;obj.kf_hover_roll;obj.kf_hover_pitch;obj.kf_hover_roll;obj.kf_pusher];
        end

        %Estimate Airspeed using z axis
        function V = update_filter_z(obj,actuators,a_z)
            % Inputs: accelerations, angle of attack, actuator commands
            Fz_prop_per_motor = obj.pprz2pwm(actuators).*obj.kf_motors([1:4],1)+obj.kf_motors([1:4],2);

            Fz_prop = sum(Fz_prop_per_motor);
            
            Fz_body_no_V = 0.5*obj.density*obj.CL_body(3)*(obj.CL_body(1)+obj.CL_body(2)*AoA); % 1/2 rho S V^2 (CL_0+CL_alpha*alpha)

            V = sqrt((obj.m*a_z-Fz_prop)./Fz_body_no_V);

        end

        function pwm = pprz2pwm(pprz)
            pwm = ((pprz-1000).*0.1391)+1000;
        end

    end



end

