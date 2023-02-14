function [theta_rad,theta_t] = offset_sinus_addition(t,sig_a,sig_b,addition,graph)
%OFFSET_SINUS_ADDITION Calculates the offset between two sinus function
%when adding or substracting them
%
% Based on the following equation:
%   a*sin(w*x)+b*sin(w*x+theta) = c sin(w*x+phi)
%   where:
%           c = (a^2+b^2+2*a*b*cos(theta))^(1/2)
%           tan(phi) = (b*sin(theta))/(a+b*cos(theta) 
%
% Inputs:
%           -t: time [s]
%           -sig_a: signal a
%           -sig_b: signal b
%           -addition: 1 if addition, 0 if substraction
%           -graph: 1 to show graph, 0 to hide figure
%
% Outputs:
%           -theta_rad: offset [rad]
%           -theta_t: offset [t]

% Amplitude of initial signals
[a,~,~,~] = fit_sinus(t,sig_a,graph);
[b,~,~,~] = fit_sinus(t,sig_b,graph);

% Handle the case of substractions
if addition
    b=b;
    sig_apb = sig_a+sig_b;
else
    b=-b;
    sig_apb = sig_a-sig_b;
end

% Amplitude signal
sig_apb_filt = movmean(sig_apb,floor(1/mean(diff(t))));
[c,period,~,~] = fit_sinus(t,sig_apb_filt,graph);

% Calculate offset between two initial sinus
theta_rad = acos((c.^2-a.^2-b.^2)./(2*a*b)); %[rad]
theta_t = (theta_rad/(2*pi))*period;

end