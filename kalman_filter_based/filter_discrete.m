classdef filter_discrete < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        b
        a
        u_m
        y_m
    end

    methods
        function obj = filter_discrete(b,a,y_0,u_0)
            
            obj.b = b./a(1);
            obj.a = a./a(1);

            obj.u_m = u_0.*ones(1,length(obj.b)-1);
            obj.y_m = y_0.*ones(1,length(obj.b)-1);
        end

        function y = update_filter_discrete(obj,u)
            y = (sum(obj.b.*[u obj.u_m])+-sum(obj.a(2:end).*[obj.y_m]))./obj.a(1);

            obj.u_m = [u obj.u_m(1:end-1)];
            obj.y_m = [y obj.y_m(1:end-1)];
        end
    end
end