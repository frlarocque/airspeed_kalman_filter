classdef residual_fault_detector < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        res = 0; %[m/s]
        res_filt = 0; %[m/s]
        res_diff = 0; %[m/s]
        
        crit_low  = 3; %[m/s]
        crit_high = 8; %[m/s]
        crit_diff = 10; %[m/s /s]

        time_low  = 2; %[s]
        time_high = 0.2; %[s]
        time_diff = 0.1; %[s]

        count_low = 0;
        count_high = 0;
        count_diff = 0;

        flag_low_fault  = 0;
        flag_high_fault = 0;
        flag_diff_fault = 0;

        filter_freq = 0.1; %[Hz]

        dt = 0.05; %[s]

        low_pass_filter;

    end

    methods
        function obj = residual_fault_detector(crit_low,crit_high,crit_diff,time_low,time_high,time_diff,filter_freq,dt)
            obj.res = 0;
            obj.res_filt = 0;
            obj.res_diff = 0;
            
            obj.crit_low  = crit_low; %[m/s]
            obj.crit_high = crit_high; %[m/s]
            obj.crit_diff = crit_diff; %[m/s /s]
    
            obj.time_low  = time_low; %[s]
            obj.time_high = time_high; %[s]
            obj.time_diff = time_diff; %[s]
    
            obj.count_low  = 0;
            obj.count_high = 0;
            obj.count_diff = 0;
    
            obj.flag_low_fault  = 0;
            obj.flag_high_fault = 0;
            obj.flag_diff_fault = 0;
    
            obj.filter_freq = filter_freq; %[Hz]

            obj.dt = dt;

            [b,a] = butter(2,2*obj.filter_freq*dt,'low');
            obj.low_pass_filter = filter_discrete(b,a,0,0);
        end

        function update_innov(obj,res)
            last_res_filt = obj.res_filt;
            obj.res = res;
            obj.res_filt = obj.low_pass_filter.update_filter_discrete(res);
            obj.res_diff = (obj.res_filt-last_res_filt)./obj.dt;
        end

        function check_thresholds(obj,quick_convergence)

            % High Threshold
            if abs(obj.res_filt)>obj.crit_high && ~obj.flag_high_fault
                obj.count_high = obj.count_high+1;
            elseif abs(obj.res_filt)>obj.crit_high && obj.flag_high_fault
                obj.count_high = obj.count_high;
            else
                obj.count_high = 0;
            end
            if obj.count_high>=obj.time_high/obj.dt
                obj.flag_high_fault = 1;
            else
                obj.flag_high_fault = 0;
            end
            
            % Low Threshold
            if ~quick_convergence
                if abs(obj.res_filt)>obj.crit_low && ~obj.flag_low_fault
                    obj.count_low = obj.count_low+1;
                elseif abs(obj.res_filt)>obj.crit_low && obj.flag_low_fault
                    obj.count_low = obj.count_low;
                else
                    obj.count_low = 0;
                end
                if obj.count_low>=obj.time_low/obj.dt
                    obj.flag_low_fault = 1;
                else
                    obj.flag_low_fault = 0;
                end
            end
            
            % Diff Threshold
            if abs(obj.res_diff)>obj.crit_diff && ~obj.flag_diff_fault
                obj.count_diff = obj.count_diff+1;
            elseif abs(obj.res_diff)>obj.crit_diff && obj.flag_diff_fault
                obj.count_diff = obj.count_diff;
            else
                obj.count_diff = 0;
            end
            if obj.count_diff>=obj.time_diff/obj.dt
                obj.flag_diff_fault = 1;
            else
                obj.flag_diff_fault = 0;
            end

        end
    end
end