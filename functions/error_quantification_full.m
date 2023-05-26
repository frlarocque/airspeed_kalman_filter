function [error] = error_quantification_full(airspeed_estimation,airspeed_pitot,valid_pitot,skew)
%ERROR_QUANTIFICATION Calculates different types of error between two
%signals
%
% Calculates relative error, absolute error, RMS error

%% For all values
error.all = error_quantification(airspeed_estimation,airspeed_pitot);

%% For valid pitot tube values
error.valid_pitot = error_quantification(airspeed_estimation(valid_pitot),airspeed_pitot(valid_pitot));

%% For different flight conditions
[bool_hover,bool_transition,bool_ff] = identify_hover_transition_ff(skew);    
error.hover = error_quantification(airspeed_estimation(bool_hover),airspeed_pitot(bool_hover));
error.transition = error_quantification(airspeed_estimation(bool_transition),airspeed_pitot(bool_transition));
error.ff = error_quantification(airspeed_estimation(bool_ff),airspeed_pitot(bool_ff));

end