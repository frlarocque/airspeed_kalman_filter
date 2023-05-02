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
cut_condition = [flight_res.t(1)+1 flight_res.t(end)-1];

flight_res.x = [ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.u_est,ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.v_est,ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.w_est,...
                ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.mu_N,ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.mu_E,ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.mu_D,...
                ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.offset_x,ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.offset_y,ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.offset_z]'; %[u,v,w,mu_x,mu_y,mu_z,k_x,k_y,k_z]
flight_res.y = [ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.innov_V_gnd_x,ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.innov_V_gnd_y,ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.innov_V_gnd_z,...
                ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.innov_acc_x,ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.innov_acc_y,ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.innov_acc_z,...
                ac_data.AIRSPEED_WIND_ESTIMATOR_EKF.innov_V_pitot]';

acc = cut_resample([ac_data.STAB_ATTITUDE_FULL_INDI.body_accel_x ac_data.STAB_ATTITUDE_FULL_INDI.body_accel_y ac_data.STAB_ATTITUDE_FULL_INDI.body_accel_z],...
                    ac_data.STAB_ATTITUDE_FULL_INDI.timestamp,flight_res.t,cut_condition);
rate = cut_resample(deg2rad([ac_data.ROTORCRAFT_FP.phi_alt,ac_data.ROTORCRAFT_FP.theta_alt,ac_data.ROTORCRAFT_FP.psi_alt]),...
                    ac_data.ROTORCRAFT_FP.timestamp,flight_res.t,cut_condition);
euler = cut_resample(deg2rad([ac_data.ROTORCRAFT_FP.phi_alt,ac_data.ROTORCRAFT_FP.theta_alt,ac_data.ROTORCRAFT_FP.psi_alt]),...
                    ac_data.ROTORCRAFT_FP.timestamp,flight_res.t,cut_condition);
RPM_pusher = cut_resample(ac_data.ESC_PER_MOTOR.motor_4.rpm,...
                    ac_data.ESC_PER_MOTOR.motor_4.timestamp,flight_res.t,cut_condition);

hover_prop_rpm.raw.time = ac_data.ESC_PER_MOTOR.motor_0.timestamp;
hover_prop_rpm.raw.data(:,1) = ac_data.ESC_PER_MOTOR.motor_0.rpm;
temp_data = resample(timeseries(ac_data.ESC_PER_MOTOR.motor_1.rpm,ac_data.ESC_PER_MOTOR.motor_1.timestamp), hover_prop_rpm.raw.time);
hover_prop_rpm.raw.data(:,2) = temp_data.data;
temp_data = resample(timeseries(ac_data.ESC_PER_MOTOR.motor_2.rpm,ac_data.ESC_PER_MOTOR.motor_2.timestamp), hover_prop_rpm.raw.time);
hover_prop_rpm.raw.data(:,3) = temp_data.data;
temp_data = resample(timeseries(ac_data.ESC_PER_MOTOR.motor_3.rpm,ac_data.ESC_PER_MOTOR.motor_3.timestamp), hover_prop_rpm.raw.time);
hover_prop_rpm.raw.data(:,4) = temp_data.data;

RPM_hover = cut_resample(mean(hover_prop_rpm.raw.data,2),...
                    hover_prop_rpm.raw.time,flight_res.t,cut_condition);

skew = cut_resample(deg2rad(round(ac_data.ROT_WING_CONTROLLER.wing_angle_deg)),ac_data.ROT_WING_CONTROLLER.timestamp,flight_res.t,cut_condition);

act_values = double(string(ac_data.ACTUATORS.values));
elevator = cut_resample(elevator_pprz2angle(act_values(:,9)),ac_data.ACTUATORS.timestamp,flight_res.t,cut_condition);

flight_res.u = [acc rate euler RPM_pusher RPM_hover skew elevator]';

%flight_res.z = zeros(size(y));

Q_temp = cut_resample([ac_data.AIRSPEED_WIND_ESTIMATOR_EKF_COV.Q_accel ac_data.AIRSPEED_WIND_ESTIMATOR_EKF_COV.Q_gyro ac_data.AIRSPEED_WIND_ESTIMATOR_EKF_COV.Q_mu_x ac_data.AIRSPEED_WIND_ESTIMATOR_EKF_COV.Q_mu_y ac_data.AIRSPEED_WIND_ESTIMATOR_EKF_COV.Q_mu_z ac_data.AIRSPEED_WIND_ESTIMATOR_EKF_COV.Q_k],...
                    ac_data.AIRSPEED_WIND_ESTIMATOR_EKF_COV.timestamp,flight_res.t,cut_condition);
R_temp = cut_resample([ac_data.AIRSPEED_WIND_ESTIMATOR_EKF_COV.R_V_gnd ac_data.AIRSPEED_WIND_ESTIMATOR_EKF_COV.R_accel_filt_x ac_data.AIRSPEED_WIND_ESTIMATOR_EKF_COV.R_accel_filt_y ac_data.AIRSPEED_WIND_ESTIMATOR_EKF_COV.R_accel_filt_z ac_data.AIRSPEED_WIND_ESTIMATOR_EKF_COV.R_V_pitot],...
                    ac_data.AIRSPEED_WIND_ESTIMATOR_EKF_COV.timestamp,flight_res.t,cut_condition);
P_temp = cut_resample([ac_data.AIRSPEED_WIND_ESTIMATOR_EKF_COV.P_u ac_data.AIRSPEED_WIND_ESTIMATOR_EKF_COV.P_v ac_data.AIRSPEED_WIND_ESTIMATOR_EKF_COV.P_w ac_data.AIRSPEED_WIND_ESTIMATOR_EKF_COV.P_mu_x ac_data.AIRSPEED_WIND_ESTIMATOR_EKF_COV.P_mu_y ac_data.AIRSPEED_WIND_ESTIMATOR_EKF_COV.P_mu_z ac_data.AIRSPEED_WIND_ESTIMATOR_EKF_COV.P_k_x ac_data.AIRSPEED_WIND_ESTIMATOR_EKF_COV.P_k_y ac_data.AIRSPEED_WIND_ESTIMATOR_EKF_COV.P_k_z],...
                    ac_data.AIRSPEED_WIND_ESTIMATOR_EKF_COV.timestamp,flight_res.t,cut_condition);

for i=1:length(flight_res.t)
    flight_res.P{i} = nan.*ones(9,9);
    for j=1:size(R_temp,2)
        flight_res.P{i}(j,j) = R_temp(i,j);
    end
    flight_res.Q{i} = diag([ones(1,3).*Q_temp(i,1) ones(1,3).*Q_temp(i,2) Q_temp(i,3:5) ones(1,3).*Q_temp(i,6)]);
    flight_res.R{i} = diag([ones(1,3).*R_temp(i,1) R_temp(i,2) R_temp(i,3) R_temp(i,4) R_temp(i,5)]);
end

%%
airspeed.time = 0;
airspeed.data = 0;
wind.raw.time = 0;
wind.raw.data = 0;
wind.vect = [0,0,0];

plot_EKF_result(flight_res,airspeed,wind)

