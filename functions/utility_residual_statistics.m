y_total = [];
y_total_valid_pitot = [];
y_total_no_convergence = [];
y_total_no_convergence_valid_pitot = [];
flight_times = [];
flight_times_valid_pitot = [];
flight_times_no_convergence = [];
flight_times_no_convergence_valid_pitot = [];
for i=1:length(kalman_res)
    y_total = [y_total kalman_res{i}.y];
    y_total_valid_pitot = [y_total_valid_pitot kalman_res{i}.y(:,kalman_res{i}.pitot_airspeed.flight.valid)];
    y_total_no_convergence = [y_total_no_convergence kalman_res{i}.y(:,EKF_AW_QUICK_CONVERGENCE_TIME*f_EKF:end)];
    
    temp_cond = [zeros(EKF_AW_QUICK_CONVERGENCE_TIME*f_EKF,1)];
    temp_cond = logical([temp_cond; ones(length(kalman_res{i}.t)-length(temp_cond),1)]) & kalman_res{i}.pitot_airspeed.flight.valid;
    y_total_no_convergence_valid_pitot = [y_total_no_convergence_valid_pitot kalman_res{i}.y(:,temp_cond)];
    
    flight_times = [flight_times kalman_res{i}.t(end)-kalman_res{i}.t(1)];
    flight_times_valid_pitot = [flight_times_valid_pitot length(kalman_res{i}.y(:,kalman_res{i}.pitot_airspeed.flight.valid)).*mean(diff(kalman_res{i}.t))];
    flight_times_no_convergence = [flight_times_no_convergence kalman_res{i}.t(end)-kalman_res{i}.t(EKF_AW_QUICK_CONVERGENCE_TIME.*f_EKF)];
    flight_times_no_convergence_valid_pitot = [flight_times_no_convergence_valid_pitot length(kalman_res{i}.y(:,temp_cond)).*mean(diff(kalman_res{i}.t))];
end

%residual_hist(y_total',200,dt,1,5)
%residual_hist(y_total_valid_pitot',200,dt,1,5)
%residual_hist(y_total_no_convergence',200,dt,1,5)

%nice_plot_residual_hist(y_total_no_convergence',200,dt,1,5)

%% Autocorrelation
filt_freq = 5;
[b,a] = butter(2,2*filt_freq*dt,'low');
signal = filtfilt(b,a,y_total_valid_pitot(7,:));
signal_diff = filtfilt(b,a,diff(y_total_valid_pitot(7,:))./dt);


ax1 = subplot(1,1,1);
[r,lags] = xcorr(signal_diff,'normalized',10./dt);
plot(lags*dt,r)
hold on
ax1.LineStyleOrderIndex = ax1.ColorOrderIndex;
[r,lags] = xcorr(signal,'normalized',10./dt);
plot(lags*dt,r)

xlabel('Lag [s]')
ylabel('Autocorrelation [-]')
xlim([-5 5])
grid on
xticks(linspace(-5,5,11))
legend('Residual Derivative','Residual','location','southeast')

set(findall(gcf,'-property','FontSize'),'FontSize',20)

% Get the current date and time
nowDateTime = datetime('now');

% Format the date and time in the "MM_DD_HH_MM" format
formattedDateTime = datestr(nowDateTime,'mm_dd_HH_MM');

% Export
fig_name = ['autocorrelation_res_deriv_',formattedDateTime,'.eps'];
exportgraphics(fig,fig_name,'BackgroundColor','none','ContentType','vector')

%% Find moments where detection would occur

threshold = 6;
time_treshold = 0.24;

signal = filtfilt(b,a,y_total_no_convergence_valid_pitot(7,:));
diff_signal = diff(signal)./dt;

aboveThreshold = signal > threshold | signal < -threshold ;

% Step 3: Find the starting indices of sequences
startIndices = find(diff([0 aboveThreshold 0]) == 1);
endIndices = find(diff([0 aboveThreshold 0]) == -1) - 1;

% Find the moments where the signal is continuously above 3 for 4 seconds
validIndices = find(endIndices - startIndices >= time_treshold/dt);
validStartIndices = startIndices(validIndices);
validEndIndices = endIndices(validIndices);

length(validIndices)

% Display the starting indices
temp_time = 0:dt:dt*length(signal)-dt;
figure;
plot(temp_time,signal)
hold on
plot(temp_time(validStartIndices),signal(validStartIndices),'*')
yline(threshold)
yline(-threshold)

last_flight_time = 0;
for i=1:length(flight_times_no_convergence_valid_pitot)
    xline(flight_times_no_convergence_valid_pitot(i)+last_flight_time)
    last_flight_time = flight_times_no_convergence_valid_pitot(i)+last_flight_time;
end

%% High Criteria
cauchy_cdf = @(x,x_0,gamma) 1/pi*atan2(x-x_0,gamma)+0.5;
x_0 = 6.21E-1;
gamma = 7.89E-01;

threshold = 5;
time_treshold = 0.25; 

f_EKF;
total_duration = 60*60;

p_cauchy_single = 1+ cauchy_cdf(-threshold,x_0,gamma) - cauchy_cdf(threshold,x_0,gamma);
p_cauchy_sequence = p_cauchy_single^(floor(time_treshold.*f_EKF));
intervals = total_duration/(time_treshold);


p_total_duration = 1-(1-p_cauchy_sequence).^intervals

%% High Criteria
threshold = 3;
time_treshold = 5; 

f_EKF;
total_duration = 60*60;

p_cauchy_single = 1+ cauchy_cdf(-threshold,x_0,gamma) - cauchy_cdf(threshold,x_0,gamma);
p_cauchy_sequence = p_cauchy_single^(floor(time_treshold.*f_EKF));
intervals = total_duration/(time_treshold);

p_total_duration = 1-(1-p_cauchy_sequence).^intervals

%%
x_0 = -4.8E-03;
gamma = 5.76E-01;

threshold = 9;
time_treshold = 0.12; 

f_EKF;
total_duration = 320;

p_cauchy_single = 1 + cauchy_cdf(-threshold,x_0,gamma) - cauchy_cdf(threshold,x_0,gamma);
p_cauchy_sequence = p_cauchy_single^(floor(time_treshold.*f_EKF));

intervals = total_duration/(time_treshold);

p_total_duration = 1-(1-p_cauchy_sequence).^intervals


%%  Low
time_treshold=0.04*50;
f_EKF
total_duration = 320;

p = 0.02+0.12; %3 threshold

p_sequence = p^(floor(time_treshold.*f_EKF));

intervals = total_duration/(time_treshold);

p_total_duration = 1-(1-p_sequence).^intervals;

p_total_duration*100

%%  High
time_treshold=0.04*6;
f_EKF
total_duration = 320;

p = 0.1; % Diff 0.006076+0.003717;

p_sequence = p^(floor(time_treshold.*f_EKF));

intervals = total_duration/(time_treshold);

p_total_duration = 1-(1-p_sequence).^intervals;

p_total_duration*100

%% Diff
time_treshold=3*0.04;
f_EKF
total_duration = 320;

p = 0.006076+0.003717;

p_sequence = p^(floor(time_treshold.*f_EKF));

intervals = total_duration/(time_treshold);

p_total_duration = 1-(1-p_sequence).^intervals;

p_total_duration*100