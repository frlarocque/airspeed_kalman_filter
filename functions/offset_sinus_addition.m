function [theta_rad,theta_t] = offset_sinus_addition(t,sig_a,sig_b,addition,graph)
%UNTITLED2 Summary of this function goes here
%   a*sin(w*x)+b*sin(w*x+theta) = c sin(w*x+phi)
%   where:
%           c = (a^2+b^2+2*a*b*cos(theta))^(1/2)
%           tan(phi) = (b*sin(theta))/(a+b*cos(theta) 

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

    function A = find_amplitude(sig)
        % Find amplitude
        pks_p = findpeaks(sig);
        pks_m = findpeaks(-sig);
        
        pks = [pks_p, pks_m];
        
        if ~isempty(pks)
            A = mean(pks);
        else
            error('No peaks found')
        end

    end


%min_length = min([length(pks_p),length(pks_m)]);
%pks_p = pks_p(1:min_length);
%locs_p = locs_p(1:min_length);
%pks_m = pks_m(1:min_length);
%locs_m = locs_m(1:min_length);




end