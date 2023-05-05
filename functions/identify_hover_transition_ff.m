function [bool_hover,bool_transition,bool_ff] = identify_hover_transition_ff(skew)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
skew_hover_min = deg2rad(0);
skew_hover_max = deg2rad(45);
skew_transition_min = skew_hover_max;
skew_transition_max = deg2rad(80);
skew_ff_min = skew_transition_max;
skew_ff_max = deg2rad(90);

skew = max(deg2rad(0), min(skew, deg2rad(90)));

bool_hover = skew>=skew_hover_min & skew<skew_hover_max;
bool_transition = skew>=skew_transition_min & skew<skew_transition_max;
bool_ff = skew>=skew_ff_min & skew<=skew_ff_max;

end