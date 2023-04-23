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
    
    if exist('ac_data', 'var') && ~isempty(ac_data)
        %Life is good
    else
        ac_data = temp_ac_data;
        clear temp_ac_data
    end

    % Obtained directly
    if all(ac_data.EFF_FULL_INDI.body_accel_x==0)
        IMU_accel.raw.data = [ac_data.INS.ins_xdd_alt,ac_data.INS.ins_ydd_alt,ac_data.INS.ins_zdd_alt]; IMU_accel.raw.time = ac_data.INS.timestamp;
        fprintf("LOW RATE ACCEL\n")
    else
        IMU_accel.raw.data = [ac_data.EFF_FULL_INDI.body_accel_x,ac_data.EFF_FULL_INDI.body_accel_y,ac_data.EFF_FULL_INDI.body_accel_z];IMU_accel.raw.time =ac_data.EFF_FULL_INDI.timestamp;
        fprintf("INDI ACCEL!!! \n")
    end
    
    airspeed_pitot.raw.data = ac_data.EFF_FULL_INDI.airspeed;airspeed_pitot.raw.time = ac_data.EFF_FULL_INDI.timestamp;
    % Filter Airspeed data
    filter_freq = 0.25; %[Hz]
    [b,a] = butter(4,2*filter_freq*mean(diff(airspeed_pitot.raw.time)),'low');
    airspeed_pitot.raw.data = filtfilt(b,a,airspeed_pitot.raw.data);
    
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
    wind.time = Vg_NED_tunnel.raw.time;
    wind.data = zeros(length(Vg_NED_tunnel.raw.data),1);
    
    Vg_NED.raw.data = Vg_NED_tunnel.raw.data+[airspeed_pitot.raw.data zeros(length(airspeed_pitot.raw.data),2) ]+[wind.data zeros(length(wind.data),2) ];Vg_NED.raw.time = Vg_NED_tunnel.raw.time;
    position_NED.raw.data = position_NED_tunnel.raw.data+cumtrapz(Vg_NED.raw.time,Vg_NED.raw.data);position_NED.raw.time = position_NED_tunnel.raw.time;
    
    % Actuators
    hover_prop_pwm.raw.data = ac_data.EFF_FULL_INDI.act(:,[1:4]); hover_prop_pwm.raw.time = ac_data.EFF_FULL_INDI.timestamp;
    pusher_prop_pwm.raw.data = ac_data.EFF_FULL_INDI.act(:,9); pusher_prop_pwm.raw.time = ac_data.EFF_FULL_INDI.timestamp;
    control_surface_pwm.raw.data = ac_data.EFF_FULL_INDI.act(:,[5:8]); control_surface_pwm.raw.time = ac_data.EFF_FULL_INDI.timestamp;
    
    % TODO: might need to be modified to SP
    skew.raw.data = measSkew2Real(deg2rad(ac_data.EFF_FULL_INDI.wing_angle_deg));skew.raw.time = ac_data.EFF_FULL_INDI.timestamp;

elseif strcmp(conditions,'OUTDOOR')
    % Assign values
    position_NED.raw.data = [ac_data.ROTORCRAFT_FP.north_alt,ac_data.ROTORCRAFT_FP.east_alt,-ac_data.ROTORCRAFT_FP.up_alt]; position_NED.raw.time = ac_data.ROTORCRAFT_FP.timestamp;
    position_NED.raw.data = position_NED.raw.data-position_NED.raw.data(1,:);
    Vg_NED.raw.data = [ac_data.ROTORCRAFT_FP.vnorth_alt,ac_data.ROTORCRAFT_FP.veast_alt,-ac_data.ROTORCRAFT_FP.vup_alt];Vg_NED.raw.time=ac_data.ROTORCRAFT_FP.timestamp;
    
    airspeed_pitot.raw.data = ac_data.AIR_DATA.airspeed; airspeed_pitot.raw.time=ac_data.AIR_DATA.timestamp;
    
    IMU_angle.raw.data = deg2rad([ac_data.ROTORCRAFT_FP.phi_alt,ac_data.ROTORCRAFT_FP.theta_alt,ac_data.ROTORCRAFT_FP.psi_alt]);IMU_angle.raw.time=ac_data.ROTORCRAFT_FP.timestamp;
    
    % Coming from EKF
    IMU_accel.raw.data = [ac_data.STAB_ATTITUDE_FULL_INDI.body_accel_x ac_data.STAB_ATTITUDE_FULL_INDI.body_accel_y ac_data.STAB_ATTITUDE_FULL_INDI.body_accel_z]; IMU_accel.raw.time = ac_data.STAB_ATTITUDE_FULL_INDI.timestamp;
    IMU_rate.raw.data = deg2rad([ac_data.STAB_ATTITUDE_FULL_INDI.angular_rate_p ac_data.STAB_ATTITUDE_FULL_INDI.angular_rate_q ac_data.STAB_ATTITUDE_FULL_INDI.angular_rate_r]); IMU_rate.raw.time = ac_data.STAB_ATTITUDE_FULL_INDI.timestamp;
    
    % Direct Measurement
    %IMU_rate.raw.data = deg2rad([ac_data.IMU_GYRO_SCALED.gp_alt ac_data.IMU_GYRO_SCALED.gq_alt ac_data.IMU_GYRO_SCALED.gr_alt]); IMU_rate.raw.time = ac_data.IMU_GYRO_SCALED.timestamp;
    %IMU_accel.raw.data = [ac_data.IMU_ACCEL_SCALED.ax_alt ac_data.IMU_ACCEL_SCALED.ay_alt ac_data.IMU_ACCEL_SCALED.az_alt]; IMU_accel.raw.time = ac_data.IMU_ACCEL_SCALED.timestamp;
    
    act_values = double(string(ac_data.ACTUATORS.values));
    control_surface_pprz.raw.data = act_values(:,[1:3,9]);control_surface_pprz.raw.time = ac_data.ACTUATORS.timestamp;
    hover_prop_pprz.raw.data = act_values(:,4:7);hover_prop_pprz.raw.time = ac_data.ACTUATORS.timestamp;
    pusher_prop_pprz.raw.data = act_values(:,8);pusher_prop_pprz.raw.time = ac_data.ACTUATORS.timestamp;
    clear act_values

    hover_prop_rpm.raw.time = ac_data.ESC_PER_MOTOR.motor_0.timestamp;
    hover_prop_rpm.raw.data(:,1) = ac_data.ESC_PER_MOTOR.motor_0.rpm;
    temp_data = resample(timeseries(ac_data.ESC_PER_MOTOR.motor_1.rpm,ac_data.ESC_PER_MOTOR.motor_1.timestamp), hover_prop_rpm.raw.time);
    hover_prop_rpm.raw.data(:,2) = temp_data.data;
    temp_data = resample(timeseries(ac_data.ESC_PER_MOTOR.motor_2.rpm,ac_data.ESC_PER_MOTOR.motor_2.timestamp), hover_prop_rpm.raw.time);
    hover_prop_rpm.raw.data(:,3) = temp_data.data;
    temp_data = resample(timeseries(ac_data.ESC_PER_MOTOR.motor_3.rpm,ac_data.ESC_PER_MOTOR.motor_3.timestamp), hover_prop_rpm.raw.time);
    hover_prop_rpm.raw.data(:,4) = temp_data.data;

    pusher_prop_rpm.raw.data = ac_data.ESC_PER_MOTOR.motor_4.rpm;pusher_prop_rpm.raw.time = ac_data.ESC_PER_MOTOR.motor_4.timestamp;

    skew.raw.data = deg2rad(round(ac_data.ROT_WING_CONTROLLER.wing_angle_deg));skew.raw.time = ac_data.ROT_WING_CONTROLLER.timestamp;


else
    fprintf('Wrong or No Condition')
end
%% Correct Pitot data
airspeed_pitot.raw.data = pitot_correction.*airspeed_pitot.raw.data;

%% Resample
% Resample Choice 
resample_time = airspeed_pitot.raw.time; %Airspeed has the lowest dt

if strcmp(conditions,'OUTDOOR')
    flight = in_flight(hover_prop_rpm.raw.time,mean(hover_prop_rpm.raw.data,2),Vg_NED.raw.time,-Vg_NED.raw.data(:,3),3000,0.1);
elseif strcmp(conditions,'WINDTUNNEL')
    flight = ac_data.motors_on(1);%in_flight(hover_prop_pwm.raw.time,mean(hover_prop_pwm.raw.data,2),Vg_NED.raw.time,-Vg_NED.raw.data(:,3),1150,0.1);
end
cut_condition = [flight(1),ac_data.motors_on(end)];


% Resampling
[IMU_accel.flight.data,IMU_accel.flight.time] = cut_resample(IMU_accel.raw.data,IMU_accel.raw.time,resample_time,cut_condition);
[airspeed_pitot.flight.data,airspeed_pitot.flight.time] = cut_resample(airspeed_pitot.raw.data,airspeed_pitot.raw.time,resample_time,cut_condition);
[IMU_rate.flight.data,IMU_rate.flight.time] = cut_resample(IMU_rate.raw.data,IMU_rate.raw.time,resample_time,cut_condition);
[Vg_NED.flight.data,Vg_NED.flight.time] = cut_resample(Vg_NED.raw.data,Vg_NED.raw.time,resample_time,cut_condition);
[IMU_angle.flight.data,IMU_angle.flight.time] = cut_resample(IMU_angle.raw.data,IMU_angle.raw.time,resample_time,cut_condition);
[position_NED.flight.data,position_NED.flight.time] = cut_resample(position_NED.raw.data,position_NED.raw.time,resample_time,cut_condition);

if strcmp(conditions,'OUTDOOR')
    [control_surface_pprz.flight.data,control_surface_pprz.flight.time] = cut_resample(control_surface_pprz.raw.data,control_surface_pprz.raw.time,resample_time,cut_condition);
    [hover_prop_pprz.flight.data,hover_prop_pprz.flight.time] = cut_resample(hover_prop_pprz.raw.data,hover_prop_pprz.raw.time,resample_time,cut_condition);
    [pusher_prop_pprz.flight.data,pusher_prop_pprz.flight.time] = cut_resample(pusher_prop_pprz.raw.data,pusher_prop_pprz.raw.time,resample_time,cut_condition);
    
    [hover_prop_rpm.flight.data,hover_prop_rpm.flight.time] = cut_resample(hover_prop_rpm.raw.data,hover_prop_rpm.raw.time,resample_time,cut_condition);
    hover_prop_rpm.flight.data(isnan(hover_prop_rpm.flight.data)) = 0;
    
    [pusher_prop_rpm.flight.data,pusher_prop_rpm.flight.time] = cut_resample(pusher_prop_rpm.raw.data,pusher_prop_rpm.raw.time,resample_time,cut_condition);
    pusher_prop_rpm.flight.data(isnan(pusher_prop_rpm.flight.data)) = 0;
    
    [skew.flight.data,skew.flight.time] = cut_resample(skew.raw.data,skew.raw.time,resample_time,cut_condition);
    skew.flight.data(isnan(skew.flight.data)) = 0;

elseif strcmp(conditions,'WINDTUNNEL')
    [hover_prop_pwm.flight.data,hover_prop_pwm.flight.time] = cut_resample(hover_prop_pwm.raw.data,hover_prop_pwm.raw.time,resample_time,cut_condition);
    [pusher_prop_pwm.flight.data,pusher_prop_pwm.flight.time] = cut_resample(pusher_prop_pwm.raw.data,pusher_prop_pwm.raw.time,resample_time,cut_condition);
    [control_surface_pwm.flight.data,control_surface_pwm.flight.time] = cut_resample(control_surface_pwm.raw.data,control_surface_pwm.raw.time,resample_time,cut_condition);
    [skew.flight.data,skew.flight.time] = cut_resample(skew.raw.data,skew.raw.time,resample_time,cut_condition);
end

% Arbitrarly set so far
alpha.flight.data = IMU_angle.flight.data(:,2);alpha.flight.time = IMU_angle.flight.time;
beta.flight.data = 0*ones(length(IMU_angle.flight.data),1);beta.flight.time = IMU_angle.flight.time;

%% Filtering
filter_freq = 0.25; %[Hz]
[b,a] = butter(4,2*filter_freq*mean(diff(resample_time)),'low');

airspeed_pitot.flight.data = filtfilt(b,a,airspeed_pitot.flight.data);

%% Visualizing input data
if graph
    % Raw
    plot_3_2(IMU_accel.raw,airspeed_pitot.raw,IMU_rate.raw,Vg_NED.raw,IMU_angle.raw,position_NED.raw,cut_condition)
    
    % Flight
    plot_3_2(IMU_accel.flight,airspeed_pitot.flight,IMU_rate.flight,Vg_NED.flight,IMU_angle.flight,position_NED.flight)
    
    % Actuator
    if strcmp(conditions,'OUTDOOR')
    x_axis_related_plot(IMU_accel.flight,airspeed_pitot.flight,pusher_prop_pwm.flight,IMU_angle.flight,5)
    end
end
%% Estimating wind
[wind,airspeed_estimation] = wind_estimation(Vg_NED.flight,IMU_angle.flight,airspeed_pitot.flight,alpha.flight,beta.flight,graph);

if graph
    % Visualize Trajectory
    trajectory(position_NED.flight,Vg_NED.flight,IMU_angle.flight,10,wind.vect)
end
%% Beta estimation using Wind
pitot_critical_angle = deg2rad(25);

if beta_est
    [beta.flight.data] = beta_estimation_wind(Vg_NED.flight,IMU_angle.flight,wind,graph);
    %[beta.flight.data] = beta_estimation_acc(4,IMU_accel.flight,airspeed_pitot.flight,0,2,0.1,graph);
end

beta.flight.valid = abs(beta.flight.data)<pitot_critical_angle;
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
alpha.flight.valid = abs(alpha.flight.data)<pitot_critical_angle;
valid_start = alpha.flight.time(diff(alpha.flight.valid)==1);
if alpha.flight.valid(1); valid_start = [alpha.flight.time(1); valid_start];end
valid_end = alpha.flight.time(diff(alpha.flight.valid)==-1);
if alpha.flight.valid(end);valid_end = [valid_end; alpha.flight.time(end)];end
for i=1:length(valid_start)
alpha.flight.valid_times{i} = [valid_start(i) valid_end(i)];
end

%% Pitot Valid: Alpha and beta valid
airspeed_pitot.flight.valid = beta.flight.valid & alpha.flight.valid;
valid_start = airspeed_pitot.flight.time(diff(airspeed_pitot.flight.valid)==1);
if airspeed_pitot.flight.valid(1); valid_start = [airspeed_pitot.flight.time(1); valid_start];end
valid_end = airspeed_pitot.flight.time(diff(airspeed_pitot.flight.valid)==-1);
if airspeed_pitot.flight.valid(end);valid_end = [valid_end; airspeed_pitot.flight.time(end)];end
for i=1:length(valid_start)
airspeed_pitot.flight.valid_times{i} = [valid_start(i) valid_end(i)];
end

%% Comparing pitot airspeed and estimated pitot airspeed
airspeed_estimation.ub_airspeed = airspeed_estimation.data.*(cos(alpha.flight.data).*cos(beta.flight.data)); 
airspeed_estimation.vb_airspeed = airspeed_estimation.data.*(cos(alpha.flight.data).*sin(beta.flight.data)); 

if graph
    figure;
    plot(airspeed_estimation.time,airspeed_estimation.ub_airspeed)
    hold on
    plot(airspeed_pitot.flight.time,airspeed_pitot.flight.data)
    
    for i=1:length(airspeed_pitot.flight.valid_times)
        patch([airspeed_pitot.flight.valid_times{i}(1) airspeed_pitot.flight.valid_times{i}(1) airspeed_pitot.flight.valid_times{i}(2) airspeed_pitot.flight.valid_times{i}(2)],...
              [min(airspeed_pitot.flight.data),max(airspeed_pitot.flight.data),max(airspeed_pitot.flight.data),min(airspeed_pitot.flight.data)],'green','LineStyle',"none",'FaceAlpha',.1);
    end

    title('Airspeed Estimation Using Constant Wind')
    xlabel('Time [s]')
    ylabel('Airspeed [m/s]')
    legend('Estimation','Measured','Valid Pitot')
    grid on
end
error_airspeed_all = error_quantification(airspeed_estimation.ub_airspeed,airspeed_pitot.flight.data);
error_airspeed_valid = error_quantification(airspeed_estimation.ub_airspeed(airspeed_pitot.flight.valid),airspeed_pitot.flight.data(airspeed_pitot.flight.valid));

%% Estimating Variance
static_conditions = [100,130];%[50,60];

if recalculate_variance
    fprintf('Recalculated Variance\n')
    [IMU_accel.var,IMU_accel.static] = cut_variance(IMU_accel.raw.data,IMU_accel.raw.time,static_conditions);
    [IMU_rate.var,IMU_rate.static] = cut_variance(IMU_rate.raw.data,IMU_rate.raw.time,static_conditions);
    [IMU_angle.var,IMU_angle.static] = cut_variance(IMU_angle.raw.data,IMU_angle.raw.time,static_conditions);
    [Vg_NED.var,Vg_NED.static] = cut_variance(Vg_NED.raw.data,Vg_NED.raw.time,static_conditions);
    
    if graph
        figure
        ax1 = subplot(3,1,1);
        plot(IMU_accel.static.time,IMU_accel.static.data)
        title('Accelerations')
        xlabel('Time [s]')
        ylabel('[m/s^2]')
        legend('a_x','a_y','a_z')
        grid on
        
        ax2 = subplot(3,1,2);
        plot(IMU_rate.static.time,IMU_rate.static.data)
        title('Angular Rates')
        xlabel('Time [s]')
        ylabel('[deg/s]')
        legend('p','q','r')
        grid on
        
        ax3 = subplot(3,1,3);
        plot(Vg_NED.static.time,Vg_NED.static.data)
        title('Ground Velocity')
        xlabel('Time [s]')
        ylabel('[m/s]')
        legend('V_N','V_E','V_D')
        grid on
        
        linkaxes([ax1,ax2,ax3],'x')
    end
else
    fprintf('Used Existing Variance\n')
    IMU_accel.var =1E-04.*[2.06700117078314 2.00274443026856 1.24583740531685];
    IMU_rate.var  =1E-09.*[1.09881143958005 0.41108108727572 0.67454749310375];
    Vg_NED.var    =1E-05.*[7.16084294336657 1.64270323320975 2.02350600895777];
    IMU_angle.var =1E-06.*[0.08797774563941 0.11256199212189 5.52264453785610];

    %IMU_accel.var = [0.955997973181906   0.984100194906034   0.621448262380474];
    %IMU_angle.var = 1.0e-05.*[0.019585145122639   0.003193244894540   0.688679737547505];
    %IMU_rate.var = 1.0e-06.*[0.523587921655034   0.869771590456105   0.267707266981007];
    %Vg_NED.var = 1.0e-04.*[0.101006686146742   0.168140606191843   0.051143405643740];
end
pusher_prop_rpm.var = 75.^2; %Taken from wind tunnel test data

%% Visualize data going in Kalman
if graph
plot_2_2(IMU_accel.flight,IMU_rate.flight,Vg_NED.flight,IMU_angle.flight)
end

function common_times = in_flight(t_RPM,RPM,t_vs,vs,RPM_crit,vs_crit)
    % Define common time grid
    t_common = [max(t_RPM(1), t_vs(1)): max(mean(diff(t_RPM)),mean(diff(t_vs))):min(t_RPM(end), t_vs(end))];
    
    % Interpolate vertical speed to common time grid
    vs_interp = interp1(t_vs, vs, t_common);
    
    % Interpolate RPM to common time grid
    rpm_interp = interp1(t_RPM, RPM, t_common);

    % Find indices where vertical speed is greater than threshold
    v_indices = find(vs_interp > vs_crit);
    
    % Find indices where RPM is greater than threshold
    rpm_indices = find(RPM > RPM_crit);
    
    % Find indices where both conditions are met
    common_indices = intersect(v_indices, rpm_indices);
    
    % Extract the times at which the conditions are met
    common_times = t_common(common_indices);
end