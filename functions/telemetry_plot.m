%%
clear all
close all
clc

format long
% Add all paths
addpath('/home/frederic/Documents/thesis/tools/airspeed_estimation/functions/');

%% Load single filedata
[file,path] = uigetfile({'*.mat'},'Select a file');

load(fullfile(path,file))

%% Structure
% <message name="AIRSPEED_WIND_ESTIMATOR_EKF_FORCES" id="195">
% <field name="fuselage_x"     type="float" unit="N"/>
% <field name="fuselage_y"     type="float" unit="N"/>
% <field name="fuselage_z"     type="float" unit="N"/>
% <field name="wing_x"     type="float" unit="N"/>
% <field name="wing_y"     type="float" unit="N"/>
% <field name="wing_z"     type="float" unit="N"/>
% <field name="elevator_x"     type="float" unit="N"/>
% <field name="elevator_y"     type="float" unit="N"/>
% <field name="elevator_z"     type="float" unit="N"/>
% <field name="hover_x"     type="float" unit="N"/>
% <field name="hover_y"     type="float" unit="N"/>
% <field name="hover_z"     type="float" unit="N"/>
% <field name="pusher_x"     type="float" unit="N"/>
% <field name="pusher_y"     type="float" unit="N"/>
% <field name="pusher_z"     type="float" unit="N"/>
% <field name="skew"     type="float" unit="rad"/>
% <field name="elevator_angle"     type="float" unit="rad"/>
% <field name="pusher_prop"     type="float" unit="RPM"/>
% <field name="hover_prop"     type="float" unit="RPM"/>
% </message>
% 
% <message name="AIRSPEED_WIND_ESTIMATOR_EKF_COV" id="196">
% <field name="Q_accel"     type="float" unit="-"/>
% <field name="Q_gyro"     type="float" unit="-"/>
% <field name="Q_mu_x"     type="float" unit="-"/>
% <field name="Q_mu_y"      type="float" unit="-"/>
% <field name="Q_mu_z"      type="float" unit="-"/>
% <field name="Q_k"      type="float" unit="-"/>
% <field name="R_V_gnd"      type="float" unit="-"/>
% <field name="R_accel_filt_x"      type="float" unit="-"/>
% <field name="R_accel_filt_y"      type="float" unit="-"/>
% <field name="R_accel_filt_z"      type="float" unit="-"/>
% <field name="R_V_pitot"      type="float" unit="-"/>
% <field name="P_u"  type="float" unit="-"/>
% <field name="P_v"  type="float" unit="-"/>
% <field name="P_w"  type="float" unit="-"/>
% <field name="P_mu_x"  type="float" unit="-"/>
% <field name="P_mu_y"  type="float" unit="-"/>
% <field name="P_mu_z"  type="float" unit="-"/>
% <field name="P_k_x"  type="float" unit="-"/>
% <field name="P_k_y"  type="float" unit="-"/>
% <field name="P_k_z"  type="float" unit="-"/>
% </message>
% 
% <message name="AIRSPEED_WIND_ESTIMATOR_EKF" id="199">
% <field name="u_est"     type="float" unit="m/s"/>
% <field name="v_est"     type="float" unit="m/s"/>
% <field name="w_est"     type="float" unit="m/s"/>
% <field name="mu_N"      type="float" unit="m/s"/>
% <field name="mu_E"      type="float" unit="m/s"/>
% <field name="mu_D"      type="float" unit="m/s"/>
% <field name="offset_x"  type="float" unit="-"/>
% <field name="offset_y"  type="float" unit="-"/>
% <field name="offset_z"  type="float" unit="-"/>
% <field name="healthy"   type="uint8" unit="bool"/>
% <field name="crashes_n"  type="uint16" unit="-"/>
% <field name="innov_V_gnd_x"  type="float" unit="-"/>
% <field name="innov_V_gnd_y"  type="float" unit="-"/>
% <field name="innov_V_gnd_z"  type="float" unit="-"/>
% <field name="innov_acc_x"  type="float" unit="-"/>
% <field name="innov_acc_y"  type="float" unit="-"/>
% <field name="innov_acc_z"  type="float" unit="-"/>
% <field name="innov_V_pitot"  type="float" unit="-"/>
% <field name="debug_1"  type="float" unit="-"/>
% <field name="debug_2"  type="float" unit="-"/>
% <field name="debug_3"  type="float" unit="-"/>
% </message>

%% Resample


%% Create kalman res structure
% x = [u v w mu_x mu_y mu_z k_x k_y k_z];
% u = [a_x a_y a_z p q r phi theta psi RPM_pusher RPM_hover skew elevator_pprz];
% z = [V_x V_y V_z a_x a_y a_z pitot];

flight_res.t = ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.timestamp;
flight_res.x = [ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.u_est,ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.v_est,ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.w_est,...
                ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.mu_N,ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.mu_E,ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.mu_D,...
                ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.k_offset_x,ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.offset_y,ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.offset_z]; %[u,v,w,mu_x,mu_y,mu_z,k_x,k_y,k_z]
flight_res.y = [ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.innov_V_gnd_x,ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.innov_V_gnd_y,ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.innov_V_gnd_z,...
                ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.acc_x,ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.acc_y,ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.acc_z,...
                ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.V_pitot];

acc = resample([ac_data.STAB_ATTITUDE_FULL_INDI.body_accel_x ac_data.STAB_ATTITUDE_FULL_INDI.body_accel_y ac_data.STAB_ATTITUDE_FULL_INDI.body_accel_z],flight_res.t,ac_data.STAB_ATTITUDE_FULL_INDI.timestamp);
rate = resample(deg2rad([ac_data.ROTORCRAFT_FP.phi_alt,ac_data.ROTORCRAFT_FP.theta_alt,ac_data.ROTORCRAFT_FP.psi_alt]),flight_res.t,ac_data.ROTORCRAFT_FP.timestamp);
euler = resample(deg2rad([ac_data.ROTORCRAFT_FP.phi_alt,ac_data.ROTORCRAFT_FP.theta_alt,ac_data.ROTORCRAFT_FP.psi_alt]),flight_res.t,ac_data.ROTORCRAFT_FP.timestamp);
RPM_pusher = resample(ac_data.ESC_PER_MOTOR.motor_4.rpm,flight_res.t,ac_data.ESC_PER_MOTOR.motor_4.timestamp);

hover_prop_rpm.raw.time = ac_data.ESC_PER_MOTOR.motor_0.timestamp;
hover_prop_rpm.raw.data(:,1) = ac_data.ESC_PER_MOTOR.motor_0.rpm;
temp_data = resample(timeseries(ac_data.ESC_PER_MOTOR.motor_1.rpm,ac_data.ESC_PER_MOTOR.motor_1.timestamp), hover_prop_rpm.raw.time);
hover_prop_rpm.raw.data(:,2) = temp_data.data;
temp_data = resample(timeseries(ac_data.ESC_PER_MOTOR.motor_2.rpm,ac_data.ESC_PER_MOTOR.motor_2.timestamp), hover_prop_rpm.raw.time);
hover_prop_rpm.raw.data(:,3) = temp_data.data;
temp_data = resample(timeseries(ac_data.ESC_PER_MOTOR.motor_3.rpm,ac_data.ESC_PER_MOTOR.motor_3.timestamp), hover_prop_rpm.raw.time);
hover_prop_rpm.raw.data(:,4) = temp_data.data;
RPM_hover = resample(mean(hover_prop_rpm.raw.data,2),flight_res.t,hover_prop_rpm.raw.time);

skew = resample(deg2rad(round(ac_data.ROT_WING_CONTROLLER.wing_angle_deg)),flight_res.t,ac_data.ROT_WING_CONTROLLER.timestamp);

act_values = double(string(ac_data.ACTUATORS.values));
elevator = resample(elevator_pprz2angle(act_values(:,9)),flight_res.t,ac_data.ACTUATORS.timestamp);

flight_res.u = [acc rate euler RPM_pusher RPM_hover skew elevator];

%flight_res.z = zeros(size(y));

%flight_res.P = P;
%flight_res.Q = Q_variable;
%flight_res.R = R_variable;
%flight_res.K = K;
%flight_res.S = S;

