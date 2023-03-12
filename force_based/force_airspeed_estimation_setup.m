% NEED:
% ac_data
% file
% graph
% beta_est
% recalculate_variance
% pitot_correction

%% Assign
fprintf('OPENED: %s\n',file)
conditions = split(file,'_');
conditions = conditions{1};

if strcmp(conditions,'WINDTUNNEL')
    % Obtained directly
    IMU_accel.raw.data = [ac_data.EFF_FULL_INDI.body_accel_x,ac_data.EFF_FULL_INDI.body_accel_y,ac_data.EFF_FULL_INDI.body_accel_z];IMU_accel.raw.time =ac_data.EFF_FULL_INDI.timestamp;
    airspeed_pitot.raw.data = ac_data.EFF_FULL_INDI.airspeed;airspeed_pitot.raw.time = ac_data.EFF_FULL_INDI.timestamp;
    IMU_rate.raw.data = deg2rad([ac_data.EFF_FULL_INDI.angular_rate_p ac_data.EFF_FULL_INDI.angular_rate_q ac_data.EFF_FULL_INDI.angular_rate_r]);IMU_rate.raw.time = ac_data.EFF_FULL_INDI.timestamp;
    IMU_angle.raw.data = deg2rad([ac_data.EFF_FULL_INDI.phi_alt ac_data.EFF_FULL_INDI.theta_alt ac_data.EFF_FULL_INDI.psi_alt]);IMU_angle.raw.time = ac_data.EFF_FULL_INDI.timestamp;

    %Tunnel referential
    position_NED_tunnel.raw.data =[ac_data.EFF_FULL_INDI.position_N,ac_data.EFF_FULL_INDI.position_E,ac_data.EFF_FULL_INDI.position_D];position_NED_tunnel.raw.time = ac_data.EFF_FULL_INDI.timestamp;
    Vg_NED_tunnel.raw.data = [ac_data.EFF_FULL_INDI.speed_N ac_data.EFF_FULL_INDI.speed_E ac_data.EFF_FULL_INDI.speed_D]; Vg_NED_tunnel.raw.time = ac_data.EFF_FULL_INDI.timestamp;
    
    % As if were moving
    [airspeed_pitot.raw.data,airspeed_pitot.raw.time] = cut_resample(airspeed_pitot.raw.data,airspeed_pitot.raw.time,Vg_NED_tunnel.raw.time,[Vg_NED_tunnel.raw.time(1)+1,Vg_NED_tunnel.raw.time(end)-1]);
    [position_NED_tunnel.raw.data,position_NED_tunnel.raw.time] = cut_resample(position_NED_tunnel.raw.data,position_NED_tunnel.raw.time,Vg_NED_tunnel.raw.time,[Vg_NED_tunnel.raw.time(1)+1,Vg_NED_tunnel.raw.time(end)-1]);
    [Vg_NED_tunnel.raw.data,Vg_NED_tunnel.raw.time] = cut_resample(Vg_NED_tunnel.raw.data,Vg_NED_tunnel.raw.time,Vg_NED_tunnel.raw.time,[Vg_NED_tunnel.raw.time(1)+1,Vg_NED_tunnel.raw.time(end)-1]);
    
    % Model wind
    max_wind = 8; %[m/s]
    rate_wind = 0.1; %[m/s /s]
    wind.time = Vg_NED_tunnel.raw.time;
    wind.data = zeros(length(Vg_NED_tunnel.raw.data),1);
    for i=1:length(wind.data)
        if wind.time(i)>ac_data.motors_on(end-1)
            wind.data(i) = min((wind.time(i)-ac_data.motors_on(end-1))*rate_wind,max_wind);
        end
    end
    
    Vg_NED.raw.data = Vg_NED_tunnel.raw.data+[airspeed_pitot.raw.data zeros(length(airspeed_pitot.raw.data),2) ]+[wind.data zeros(length(wind.data),2) ];Vg_NED.raw.time = Vg_NED_tunnel.raw.time;
    position_NED.raw.data = position_NED_tunnel.raw.data+cumtrapz(Vg_NED.raw.time,Vg_NED.raw.data);position_NED.raw.time = position_NED_tunnel.raw.time;

    %%% GET DATA FROM WIND TUNNEL FOR OFFICIAL AIRSPEED if possible

elseif strcmp(conditions,'OUTDOOR')
    % Assign values
    position_NED.raw.data = [ac_data.ROTORCRAFT_FP.north_alt,ac_data.ROTORCRAFT_FP.east_alt,-ac_data.ROTORCRAFT_FP.up_alt]; position_NED.raw.time = ac_data.ROTORCRAFT_FP.timestamp;
    Vg_NED.raw.data = [ac_data.ROTORCRAFT_FP.vnorth_alt,ac_data.ROTORCRAFT_FP.veast_alt,-ac_data.ROTORCRAFT_FP.vup_alt];Vg_NED.raw.time=ac_data.ROTORCRAFT_FP.timestamp;
    
    % Coming from EKF
    IMU_accel.raw.data = [ac_data.STAB_ATTITUDE_FULL_INDI.body_accel_x ac_data.STAB_ATTITUDE_FULL_INDI.body_accel_y ac_data.STAB_ATTITUDE_FULL_INDI.body_accel_z]; IMU_accel.raw.time = ac_data.STAB_ATTITUDE_FULL_INDI.timestamp;
    IMU_rate.raw.data = deg2rad([ac_data.STAB_ATTITUDE_FULL_INDI.angular_rate_p ac_data.STAB_ATTITUDE_FULL_INDI.angular_rate_q ac_data.STAB_ATTITUDE_FULL_INDI.angular_rate_r]); IMU_rate.raw.time = ac_data.STAB_ATTITUDE_FULL_INDI.timestamp;
    
    % Direct Measurement
    %IMU_rate.raw.data = deg2rad([ac_data.IMU_GYRO_SCALED.gp_alt ac_data.IMU_GYRO_SCALED.gq_alt ac_data.IMU_GYRO_SCALED.gr_alt]); IMU_rate.raw.time = ac_data.IMU_GYRO_SCALED.timestamp;
    %IMU_accel.raw.data = [ac_data.IMU_ACCEL_SCALED.ax_alt ac_data.IMU_ACCEL_SCALED.ay_alt ac_data.IMU_ACCEL_SCALED.az_alt]; IMU_accel.raw.time = ac_data.IMU_ACCEL_SCALED.timestamp;
    
    
    airspeed_pitot.raw.data = ac_data.AIR_DATA.airspeed; airspeed_pitot.raw.time=ac_data.AIR_DATA.timestamp;
    
    IMU_angle.raw.data = deg2rad([ac_data.ROTORCRAFT_FP.phi_alt,ac_data.ROTORCRAFT_FP.theta_alt,ac_data.ROTORCRAFT_FP.psi_alt]);IMU_angle.raw.time=ac_data.ROTORCRAFT_FP.timestamp;
    
    act_values = double(string(ac_data.ACTUATORS.values));
    control_surface_pprz.raw.data = act_values(:,2:4);control_surface_pprz.raw.time = ac_data.ACTUATORS.timestamp;
    hover_prop_pprz.raw.data = act_values(:,5:8);hover_prop_pprz.raw.time = ac_data.ACTUATORS.timestamp;
    pusher_prop_pprz.raw.data = act_values(:,9);pusher_prop_pprz.raw.time = ac_data.ACTUATORS.timestamp;
    
    hover_prop_rpm.raw.time = ac_data.ESC_PER_MOTOR.motor_0.timestamp;
    hover_prop_rpm.raw.data(:,1) = ac_data.ESC_PER_MOTOR.motor_0.rpm;
    temp_data = resample(timeseries(ac_data.ESC_PER_MOTOR.motor_1.rpm,ac_data.ESC_PER_MOTOR.motor_1.timestamp), hover_prop_rpm.raw.time);
    hover_prop_rpm.raw.data(:,2) = temp_data.data;
    temp_data = resample(timeseries(ac_data.ESC_PER_MOTOR.motor_2.rpm,ac_data.ESC_PER_MOTOR.motor_2.timestamp), hover_prop_rpm.raw.time);
    hover_prop_rpm.raw.data(:,3) = temp_data.data;
    temp_data = resample(timeseries(ac_data.ESC_PER_MOTOR.motor_3.rpm,ac_data.ESC_PER_MOTOR.motor_3.timestamp), hover_prop_rpm.raw.time);
    hover_prop_rpm.raw.data(:,4) = temp_data.data;

    pusher_prop_rpm.raw.data = ac_data.ESC_PER_MOTOR.motor_4.rpm;pusher_prop_rpm.raw.time = ac_data.ESC_PER_MOTOR.motor_4.timestamp;

    skew.raw.data = round(ac_data.ROT_WING_CONTROLLER.wing_angle_deg);skew.raw.time = ac_data.ROT_WING_CONTROLLER.timestamp;

else
    fprintf('Wrong or No Condition')
end

%% Correct Pitot data
airspeed_pitot.raw.data = pitot_correction.*airspeed_pitot.raw.data;    

%% Resample
% Resample Choice 
resample_time = airspeed_pitot.raw.time; %Airspeed has the lowest dt
cut_condition = [ac_data.motors_on(end-1),ac_data.motors_on(end)];

% Resampling
[IMU_accel.flight.data,IMU_accel.flight.time] = cut_resample(IMU_accel.raw.data,IMU_accel.raw.time,resample_time,cut_condition);
[airspeed_pitot.flight.data,airspeed_pitot.flight.time] = cut_resample(airspeed_pitot.raw.data,airspeed_pitot.raw.time,resample_time,cut_condition);
[IMU_rate.flight.data,IMU_rate.flight.time] = cut_resample(IMU_rate.raw.data,IMU_rate.raw.time,resample_time,cut_condition);
[Vg_NED.flight.data,Vg_NED.flight.time] = cut_resample(Vg_NED.raw.data,Vg_NED.raw.time,resample_time,cut_condition);
[IMU_angle.flight.data,IMU_angle.flight.time] = cut_resample(IMU_angle.raw.data,IMU_angle.raw.time,resample_time,cut_condition);
[position_NED.flight.data,position_NED.flight.time] = cut_resample(position_NED.raw.data,position_NED.raw.time,resample_time,cut_condition);
[control_surface_pprz.flight.data,control_surface_pprz.flight.time] = cut_resample(control_surface_pprz.raw.data,control_surface_pprz.raw.time,resample_time,cut_condition);
[hover_prop_pprz.flight.data,hover_prop_pprz.flight.time] = cut_resample(hover_prop_pprz.raw.data,hover_prop_pprz.raw.time,resample_time,cut_condition);
[pusher_prop_pprz.flight.data,pusher_prop_pprz.flight.time] = cut_resample(pusher_prop_pprz.raw.data,pusher_prop_pprz.raw.time,resample_time,cut_condition);

[hover_prop_rpm.flight.data,hover_prop_rpm.flight.time] = cut_resample(hover_prop_rpm.raw.data,hover_prop_rpm.raw.time,resample_time,cut_condition);
hover_prop_rpm.flight.data(isnan(hover_prop_rpm.flight.data)) = 0;

[pusher_prop_rpm.flight.data,pusher_prop_rpm.flight.time] = cut_resample(pusher_prop_rpm.raw.data,pusher_prop_rpm.raw.time,resample_time,cut_condition);
pusher_prop_rpm.flight.data(isnan(pusher_prop_rpm.flight.data)) = 0;

[skew.flight.data,skew.flight.time] = cut_resample(skew.raw.data,skew.raw.time,resample_time,cut_condition);
skew.flight.data(isnan(skew.flight.data)) = 0;


% Arbitrarly set so far
    % To add a square wave: +deg2rad(10*square(IMU_angle.flight.time./10))
alpha.flight.data = IMU_angle.flight.data(:,2);alpha.flight.time = IMU_angle.flight.time;
beta.flight.data = 0*ones(length(IMU_angle.flight.data),1);beta.flight.time = IMU_angle.flight.time;

%% Visualizing input data
if graph
    % Raw
    plot_3_2(IMU_accel.raw,airspeed_pitot.raw,IMU_rate.raw,Vg_NED.raw,IMU_angle.raw,position_NED.raw,cut_condition)
    
    % Flight
    plot_3_2(IMU_accel.flight,airspeed_pitot.flight,IMU_rate.flight,Vg_NED.flight,IMU_angle.flight,position_NED.flight)

    % Visualize Actuators
    x_axis_related_plot(IMU_accel.flight,airspeed_pitot.flight,pusher_prop_rpm.flight,IMU_angle.flight,0.5)

end
%% Estimating wind
[wind,airspeed_estimation] = wind_estimation(Vg_NED.flight,IMU_angle.flight,airspeed_pitot.flight,alpha.flight,beta.flight,graph);

if graph
    % Visualize Trajectory
    trajectory(position_NED.flight,Vg_NED.flight,IMU_angle.flight,10,wind.vect)
end
%% Beta estimation using Wind
if beta_est
    [beta.flight.data] = beta_estimation_wind(Vg_NED.flight,IMU_angle.flight,wind,graph);
end

beta.flight.valid = abs(beta.flight.data)<deg2rad(30);
valid_start = beta.flight.time(diff(beta.flight.valid)==1);
if beta.flight.valid(1); valid_start = [beta.flight.time(1); valid_start];end
valid_end = beta.flight.time(diff(beta.flight.valid)==-1);
if beta.flight.valid(end);valid_end = [valid_end; beta.flight.time(end)];end
for i=1:length(valid_start)
beta.flight.valid_times{i} = [valid_start(i) valid_end(i)];
end

%% Alpha estimation using Wind
if alpha_est
    [alpha_est] = alpha_estimation_wind(airspeed_pitot.flight,Vg_NED.flight,IMU_angle.flight,wind,graph);
end
alpha.flight.valid = abs(alpha.flight.data)<deg2rad(30);
valid_start = alpha.flight.time(diff(alpha.flight.valid)==1);
if alpha.flight.valid(1); valid_start = [alpha.flight.time(1); valid_start];end
valid_end = alpha.flight.time(diff(alpha.flight.valid)==-1);
if alpha.flight.valid(end);valid_end = [valid_end; alpha.flight.time(end)];end
for i=1:length(valid_start)
alpha.flight.valid_times{i} = [valid_start(i) valid_end(i)];
end

%% Pitot Valid: Alpha and beta valid
airspeed_pitot.flight.valid = abs(alpha.flight.data)<deg2rad(30) &abs(beta.flight.data)<deg2rad(30);
valid_start = airspeed_pitot.flight.time(diff(airspeed_pitot.flight.valid)==1);
if airspeed_pitot.flight.valid(1); valid_start = [airspeed_pitot.flight.time(1); valid_start];end
valid_end = airspeed_pitot.flight.time(diff(airspeed_pitot.flight.valid)==-1);
if airspeed_pitot.flight.valid(end);valid_end = [valid_end; airspeed_pitot.flight.time(end)];end
for i=1:length(valid_start)
airspeed_pitot.flight.valid_times{i} = [valid_start(i) valid_end(i)];
end

%% Comparing pitot airspeed and estimated pitot airspeed
ub_airspeed = airspeed_estimation.data.*(cos(alpha.flight.data).*cos(beta.flight.data)); 

if graph
    figure;
    plot(airspeed_estimation.time,ub_airspeed)
    hold on
    plot(airspeed_pitot.flight.time,airspeed_pitot.flight.data)
    
    for i=1:length(beta.flight.valid_times)
        patch([airspeed_pitot.flight.valid_times{i}(1) airspeed_pitot.flight.valid_times{i}(1) airspeed_pitot.flight.valid_times{i}(2) airspeed_pitot.flight.valid_times{i}(2)],...
              [min(airspeed_pitot.flight.data),max(airspeed_pitot.flight.data),max(airspeed_pitot.flight.data),min(airspeed_pitot.flight.data)],'green','LineStyle',"none",'FaceAlpha',.1);
    end

    title('Airspeed Estimation Using Constant Wind')
    xlabel('Time [s]')
    ylabel('Airspeed [m/s]')
    legend('Estimation','Measured','Valid Pitot')
    grid on
end
error_airspeed_all = error_quantification(ub_airspeed,airspeed_pitot.flight.data);
error_airspeed_valid = error_quantification(ub_airspeed(airspeed_pitot.flight.valid),airspeed_pitot.flight.data(airspeed_pitot.flight.valid));
