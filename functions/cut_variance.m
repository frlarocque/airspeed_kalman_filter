function [var_data,static_data] = cut_variance(data,initial_time,conditions)
%CUT_VARIANCE Cuts a signal and calculates its variance
%
% Inputs:
%           -data: data to calculate variance
%           -initial_time: time for data [s]
%           -conditions: cut conditions

cut = initial_time<conditions(2)&initial_time>conditions(1);
static_data.time = initial_time(cut);
static_data.data = data(cut,:);

var_data = var(static_data.data);
end

