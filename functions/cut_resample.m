function [new_data,new_time] = cut_resample(data,initial_time,resample_time,conditions)
%CUT_RESAMPLE Summary of this function goes here
%   Detailed explanation goes here

cut = initial_time<conditions(2)&initial_time>conditions(1);
cut2 = resample_time<conditions(2)&resample_time>conditions(1);
initial_time = initial_time(cut);
new_time = resample_time(cut2);
data = data(cut,:);

timeseries_data = resample(timeseries(data,initial_time), new_time);

new_data = timeseries_data.data;

end

