function [beta] = beta_estimation_acc(Fy,V,skew,degree)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
%
% Fy = (beta [rad] *2.194341022544540*1E-1+skew [rad]*7.781617995352236*1E-4 )* V [m/s]^2
% RMS = 1.2

% Fy = (beta [rad] *2.552098952269741*1E0+skew [rad]*8.086387413837056*1E-2 )* V [m/s]
% RMS = 4.8

% Fy = (beta [rad] *2.226114526713827*1E1+ skew [rad]*5.461659551681629*1E-2)
% RMS = 10.7

k_skew = [5.461659551681629*1E-2  8.086387413837056*1E-2 7.781617995352236*1E-4];
k_beta = [2.226114526713827*1E1 2.552098952269741*1E0  2.194341022544540*1E-1];

beta = (Fy./(V.^degree)-skew*k_skew(degree+1))./k_beta(degree+1);

% Saturate
beta = min(beta,pi/2);
beta = max(beta,-pi/2);

end