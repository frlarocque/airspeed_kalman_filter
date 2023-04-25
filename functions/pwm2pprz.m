function [y] = pwm2pprz(x,pwm,pprz)
%PWM2PPRZ Summary of this function goes here
%   Detailed explanation goes here
y = (pprz(2)-pprz(1))/(pwm(2)-pwm(1))*(x-pwm(1))+pprz(1);
end

