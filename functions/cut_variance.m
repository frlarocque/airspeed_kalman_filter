function [var_data,static_data] = cut_variance(data,initial_time,conditions)
%VARIANCE_ESTIMATOR Summary of this function goes here
%   Detailed explanation goes here
cut = initial_time<conditions(2)&initial_time>conditions(1);
static_data.time = initial_time(cut);
static_data.data = data(cut,:);

var_data = var(static_data.data);
end

