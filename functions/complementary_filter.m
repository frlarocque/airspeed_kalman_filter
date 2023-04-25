function y_filt = complementary_filter(dt,y,y_dot,alpha)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Filter Airspeed data
fc = 0.1; %[Hz]

[b_low,a_low] = butter(4,2*fc*dt,'low');
y_low_filt = filtfilt(b_low,a_low,y_low);

[b_high,a_high] = butter(4,2*fc*dt,'high');
y_high_filt = filtfilt(b_high,a_high,y_high);

y_filt = y_low_filt+y_high_filt;

figure;
plot(y_low)
hold on
plot(y_high)
plot(y_filt)
legend('low','high','filt')

alpha = 1-5E-3;
% Apply complementary filter
y_filt = zeros(length(y),1);
y_filt(1) = y(1);
for i=2:length(y)-1
    y_filt(i) = alpha * (y_filt(i-1) + y_dot(i)) + (1-alpha) * y(i);
end
y_filt(end) = y_filt(end-1);

figure;
plot(y_low)
hold on
plot(y_high)
plot(y_filt)
legend('low','high','filt')


end