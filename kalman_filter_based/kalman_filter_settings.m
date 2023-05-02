%% Kalman Filter
% x = [u v w mu_x mu_y mu_z k_x k_y k_z];
% 𝑢, 𝑣, 𝑤 = Velocity in the Body Frame
% 𝜇𝑥, 𝜇𝑦, 𝜇𝑧 = Wind Velocity in Earth Fixed Frame
% k_x, k_y, k_z = Offset states for identification of parameters

% u = [a_x a_y a_z p q r phi theta psi RPM_pusher RPM_hover skew elevator_pprz];
% 𝑎x, 𝑎𝑦, 𝑎𝑧 = Accelerations Measured by Accelerometers
% 𝑝, 𝑞, 𝑟 = Angular Rates Measured by Gyroscopes
% phi, theta, psi = euler angles
% RPM_pusher = pusher rpm
% RPM_hover = mean RPM of hover propellers
% skew = skew angle of wing
% elevator pprz = elevator position

% z = [Vx Vy Vz a_x_filt a_y_filt a_z_filt V_pitot]
% 𝑉𝑥 , 𝑉𝑦, 𝑉𝑧 = Velocity in Earth Fixed Frame
% a_x_filt a_y_filt a_z_filt = body accelerations filtered
% V_pitot = airspeed measured by pitot

global EKF_AW_AZ_QUICK_CONV_ACCEL_GAIN EKF_AW_AZ_QUICK_CONV_MU_GAIN EFK_AW_QUICK_CONVERGENCE EKF_AW_QUICK_CONVERGENCE_TIME
global EKF_AW_USE_MODEL_BASED EKF_AW_USE_BETA EKF_AW_WING_INSTALLED EKF_AW_PROPAGATE_OFFSET EKF_AW_VEHICLE_MASS EKF_AW_USE_PITOT 
global EKF_AW_AZ_SCHED_GAIN EKF_AW_AZ_SCHED_START_DEG EKF_AW_AZ_SCHED_END_DEG
global EKF_AW_AX_SCHED_GAIN EKF_AW_AX_SCHED_START_DEG EKF_AW_AX_SCHED_END_DEG
global EKF_AW_Q_accel EKF_AW_Q_gyro EKF_AW_Q_mu EKF_AW_Q_offset
global EKF_AW_R_V_gnd EKF_AW_R_accel_filt_x EKF_AW_R_accel_filt_y EKF_AW_R_accel_filt_z EKF_AW_R_V_pitot
global EKF_AW_P0_V_body EKF_AW_P0_mu EKF_AW_P0_offset
global filter_low_freq filter_high_freq f_EKF

EKF_AW_USE_MODEL_BASED = true;
EKF_AW_USE_BETA = true;
EKF_AW_WING_INSTALLED = true;
EKF_AW_PROPAGATE_OFFSET = false;
EKF_AW_USE_PITOT = false;

EFK_AW_QUICK_CONVERGENCE = true;
EKF_AW_QUICK_CONVERGENCE_TIME = 20;
EKF_AW_AZ_QUICK_CONV_MU_GAIN = 2;
EKF_AW_AZ_QUICK_CONV_ACCEL_GAIN = 0;

EKF_AW_AZ_SCHED_GAIN = 2;
EKF_AW_AZ_SCHED_START_DEG = 60;
EKF_AW_AZ_SCHED_END_DEG = 70;
EKF_AW_AX_SCHED_GAIN = 0;
EKF_AW_AX_SCHED_START_DEG = 40;
EKF_AW_AX_SCHED_END_DEG = 60;

if EKF_AW_WING_INSTALLED
    EKF_AW_VEHICLE_MASS = 6.5;
else
    EKF_AW_VEHICLE_MASS = 5.75;
end



EKF_AW_Q_accel = 1E-04;
EKF_AW_Q_gyro = 1E-09;
EKF_AW_Q_mu = 1E-6; %1E-5
EKF_AW_Q_offset = 1E-8;

EKF_AW_R_V_gnd = 1E-05;
EKF_AW_R_accel_filt_x = 1E-5;
EKF_AW_R_accel_filt_y = 1E-5;
EKF_AW_R_accel_filt_z = 1E-5;
EKF_AW_R_V_pitot = 1E-5;

EKF_AW_P0_V_body = 1E-2;
EKF_AW_P0_mu = 1E1*EKF_AW_Q_mu;
EKF_AW_P0_offset = EKF_AW_Q_offset;

filter_low_freq = 0.2; %[Hz]
filter_high_freq = 24; %[Hz]
f_EKF = 20; %[Hz]

f_fh = str2func('f_4');
g_fh = str2func('g_13_cst');

epsi = 1E-2;

% Initial conditions
x_0 = [0 0 0   0 0 0   0 0 0]';