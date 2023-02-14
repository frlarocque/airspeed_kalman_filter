% NEED:
% ac_data
% file
% graph
% beta_est
% recalculate_variance

%% Assign
fprintf('OPENED: %s\n',file)
conditions = split(file,'_');
conditions = conditions{1};

if strcmp(conditions,'WINDTUNNEL')
    % Obtained directly
    IMU_accel.raw.data = [ac_data.EFF_FULL_INDI.body_accel_x,ac_data.EFF_FULL_INDI.body_accel_y,ac_data.EFF_FULL_INDI.body_accel_z];IMU_accel.raw.time =ac_data.EFF_FULL_INDI.timestamp;
    airspeed_pitot.raw.data = low_butter(ac_data.EFF_FULL_INDI.airspeed,0.4,1/mean(diff(ac_data.EFF_FULL_INDI.timestamp)),0,4);airspeed_pitot.raw.time = ac_data.EFF_FULL_INDI.timestamp;
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
    IMU_accel.raw.data = [ac_data.IMU_ACCEL_SCALED.ax_alt ac_data.IMU_ACCEL_SCALED.ay_alt ac_data.IMU_ACCEL_SCALED.az_alt]; IMU_accel.raw.time = ac_data.IMU_ACCEL_SCALED.timestamp;
    airspeed_pitot.raw.data = low_butter(ac_data.AIR_DATA.airspeed,0.4,1/mean(diff(ac_data.AIR_DATA.timestamp)),0,4); airspeed_pitot.raw.time=ac_data.AIR_DATA.timestamp;
    IMU_rate.raw.data = deg2rad([ac_data.IMU_GYRO_SCALED.gp_alt ac_data.IMU_GYRO_SCALED.gq_alt ac_data.IMU_GYRO_SCALED.gr_alt]); IMU_rate.raw.time = ac_data.IMU_GYRO_SCALED.timestamp;
    Vg_NED.raw.data = [ac_data.ROTORCRAFT_FP.vnorth_alt,ac_data.ROTORCRAFT_FP.veast_alt,-ac_data.ROTORCRAFT_FP.vup_alt];Vg_NED.raw.time=ac_data.ROTORCRAFT_FP.timestamp;
    IMU_angle.raw.data = deg2rad([ac_data.ROTORCRAFT_FP.phi_alt,ac_data.ROTORCRAFT_FP.theta_alt,ac_data.ROTORCRAFT_FP.psi_alt]);IMU_angle.raw.time=ac_data.ROTORCRAFT_FP.timestamp;
    
else
    fprintf('Wrong or No Condition')
end

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
    
    % Visualize Trajectory
    trajectory(position_NED.flight,Vg_NED.flight,IMU_angle.flight,10)
end
%% Estimating wind
[wind,airspeed_estimation] = wind_estimation(Vg_NED.flight,IMU_angle.flight,airspeed_pitot.flight,alpha.flight,beta.flight,graph);

%% Beta estimation using Wind
if beta_est
    [beta.flight] = beta_estimation(Vg_NED.flight,IMU_angle.flight,wind,0);
end
%% Comparing pitot airspeed and estimated pitot airspeed
if graph
    figure;
    plot(airspeed_estimation.time,airspeed_estimation.data.*(cos(alpha.flight.data).*cos(beta.flight.data)))
    hold on
    plot(airspeed_pitot.flight.time,airspeed_pitot.flight.data)
    title('Airspeed Estimation Using Constant Wind')
    xlabel('Time [s]')
    ylabel('Airspeed [m/s]')
    legend('Estimation','Measured')
    grid on
end
error_airspeed = error_quantification(airspeed_estimation.data.*(cos(alpha.flight.data).*cos(beta.flight.data)),airspeed_pitot.flight.data);

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
    IMU_accel.var =[0.0087 0.0011 0.0011]; %V3
    IMU_rate.var =1.0e-03 *[0.0664    0.1253    0.1044]; %V3
    Vg_NED.var =1.0e-04 * [0.2220    0.2786    0.1342]; %V3
    IMU_angle.var = 1.0e-05 *[0.0167    0.0105    0.9880];
end

%% Visualize data going in Kalman
if graph
plot_2_2(IMU_accel.flight,IMU_rate.flight,Vg_NED.flight,IMU_angle.flight)
end