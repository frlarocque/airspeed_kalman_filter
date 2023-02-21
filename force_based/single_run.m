%%
clear all
close all
clc

% Add all paths
addpath('/home/frederic/Documents/thesis/tools/airspeed_estimation/functions/');

%% Load single filedata
[file,path] = uigetfile({'*.mat'},'Select a file');

load(fullfile(path,file))

%% Setup
% Setup Options
graph = 0;
beta_est = 0;
recalculate_variance = false;

% Run setup
force_airspeed_estimation_setup

%% Estimation parameters

density = 1.225;
m = 6.5; % [m]
kf_hover_pitch = [0.0280  -29.2535]; %[N/pwm] Based on RCTest Bench T-motor MN3515 400kV/Zubax 6S/APC 16x10'
kf_hover_roll = [0.0452  -52.8497];  %[N/pwm] Based on RCTest Bench T-motor MN3520 400kv - Tarot 17x5CF - 6S - Zubax 6S
kf_pusher = [0.0452  -52.8497];      %[N/pwm] Based on RCTest Bench T-motor MN3520 400kv - Tarot 17x5CF - 6S - Zubax 6S

kf_motors = [obj.kf_hover_pitch;obj.kf_hover_roll;obj.kf_hover_pitch;obj.kf_hover_roll;obj.kf_pusher]; %[front right back left push];

CL_body = [-0.6580 26.4910 0.0138]; % [1/rad] [CL_0 CL_alpha S_body] 


%% Run
V_list = [];
for i=1:length(airspeed_pitot)
    act = actuators.flight.data(i,1:4);
    AoA = alpha.flight.data(i);
    a_z = IMU_accel.flight.data(i,3);
    
    % Inputs: accelerations, angle of attack, actuator commands
    Fz_prop_per_motor = max(pprz2pwm(act').*kf_motors([1:4],1)+kf_motors([1:4],2),0);
    
    Fz_prop = sum(Fz_prop_per_motor);
    
    Fz_body_no_V = 0.5*density*CL_body(3)*(CL_body(1)+CL_body(2)*AoA); % 1/2 rho S V^2 (CL_0+CL_alpha*alpha)
    
    V(i) = sqrt((m*a_z-Fz_prop)./Fz_body_no_V);
end

function pwm = pprz2pwm(pprz)
    pwm = ((pprz-1000).*0.1391)+1000;
end