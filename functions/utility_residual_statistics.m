y_total = [];
y_total_valid_pitot = [];
y_total_no_convergence = [];
for i=1:length(kalman_res)
    y_total = [y_total kalman_res{i}.y];
    y_total_valid_pitot = [y_total_valid_pitot kalman_res{i}.y(:,kalman_res{i}.pitot_airspeed.flight.valid)];
    y_total_no_convergence = [y_total_no_convergence kalman_res{i}.y(:,EKF_AW_QUICK_CONVERGENCE_TIME*f_EKF:end)];
end

residual_hist(y_total',200,dt,1,1)
residual_hist(y_total_valid_pitot',200,dt,1,1)
residual_hist(y_total_no_convergence',200,dt,1,1)

nice_plot_residual_hist(y_total_no_convergence',200,dt,1,1)

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

%%  High
time_treshold=0.25;
f_EKF
total_duration = 320;

p = 0.124965+0.0086; % Diff 0.006076+0.003717;

p_sequence = p^(floor(time_treshold.*f_EKF));

intervals = total_duration/(time_treshold);

p_total_duration = 1-(1-p_sequence).^intervals;

p_total_duration*100

%% Diff
time_treshold=0.12;
f_EKF
total_duration = 320;

p = 0.006076+0.003717;

p_sequence = p^(floor(time_treshold.*f_EKF));

intervals = total_duration/(time_treshold);

p_total_duration = 1-(1-p_sequence).^intervals;

p_total_duration*100