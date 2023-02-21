function [beta] = beta_estimation_acc(Fy,V,skew,degree)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
%
% Fy = (beta [rad] *1.3939*1E-1+skew [rad]*2.2888*1E-3 )* V [m/s]^2
% or
% Fy = (beta [rad] *1.5898*1E0+skew [rad]*7.8329*1E-2 )* V [m/s]
% or
% Fy = (beta [rad] *1.35926*1E1+ skew [rad]*1.4531*1E0)

k_skew = [1.4531*1E0  7.8329*1E-2 2.2888*1E-3];
k_beta = [1.35926*1E1 1.5898*1E0  1.3939*1E-1];

beta = (Fy./(V.^degree)-skew*k_skew(degree+1))./k_beta(degree+1);

% Saturate
beta = min(beta,pi/2);
beta = max(beta,-pi/2);

end