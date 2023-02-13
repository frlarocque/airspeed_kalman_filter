function [theta] = offset_sinus_addition(sig_a,sig_b)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Amplitude of initial signals
a = find_amplitude(sig_a-mean(sig_a));
b = find_amplitude(sig_b-mean(sig_b));

% Add signal
sig_apb = sig_a+sig_b;

% Calculate offset
offset = mean(sig_apb);

% Find zero signal
zeroed_sig_apb = sig_apb-offset;

c = find_amplitude(zeroed_sig_apb);

theta = acos((c.^2-a.^2-b.^2)./(2*a*b));

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