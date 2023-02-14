function [new_data,new_time] = cut_resample(data,initial_time,resample_time,conditions)
%CUT_RESAMPLE Cuts a signal and resamples it
%
% Inputs:
%           -data: data to calculate variance
%           -initial_time: time for data [s]
%           -resample_time: time to resample [s]
%           -conditions: cut conditions

cut = initial_time<conditions(2)&initial_time>conditions(1);
cut2 = resample_time<conditions(2)&resample_time>conditions(1);
initial_time = initial_time(cut);
new_time = resample_time(cut2);
data = data(cut,:);

timeseries_data = resample(timeseries(data,initial_time), new_time);

new_data = timeseries_data.data;

end

