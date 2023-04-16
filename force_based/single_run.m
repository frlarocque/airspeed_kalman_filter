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
graph = 1;
beta_est = 1;
alpha_est = 1;
pitot_correction = 1.0;

% Run setup
force_airspeed_estimation_setup

%% 
m = 5; %[kg]

%periodogram(IMU_accel.flight.data(:,1),rectwin(length(IMU_accel.flight.data(:,1))),length(IMU_accel.flight.data(:,1)),1/mean(diff(IMU_accel.flight.time)))
%periodogram(pusher_prop.flight.data(:,1),rectwin(length(pusher_prop.flight.data(:,1))),length(pusher_prop.flight.data(:,1)),1/mean(diff(IMU_accel.flight.time)))

t = IMU_accel.flight.time;

% Filtering
filter_freq = 0.5; %[Hz]
[b,a] = butter(4,2*filter_freq*mean(diff(resample_time)),'low');

airspeed_pitot.flight.data(isnan(airspeed_pitot.flight.data))=0;
IMU_accel.flight.data(isnan(IMU_accel.flight.data))=0;
pusher_prop_rpm.flight.data(isnan(pusher_prop_rpm.flight.data))=0;
hover_prop_rpm.flight.data(isnan(hover_prop_rpm.flight.data))=0;
IMU_angle.flight.data(isnan(IMU_angle.flight.data))=0;
skew.flight.data(isnan(skew.flight.data))=0;

V_pitot = filtfilt(b,a,airspeed_pitot.flight.data);
a_x = filtfilt(b,a,IMU_accel.flight.data(:,1));
pusher_rpm = filtfilt(b,a,pusher_prop_rpm.flight.data);
hover_rpm = mean(filtfilt(b,a,hover_prop_rpm.flight.data),2);
AoA = filtfilt(b,a,IMU_angle.flight.data(:,2));
skew_rad = filtfilt(b,a,skew.flight.data);

figure
subplot(2,2,1)
plot(t,a_x)
title('Accel')
subplot(2,2,2)
plot(t,V_pitot)
title('pitot')
subplot(2,2,3)
plot(t,pusher_rpm)
title('Pusher rpm')
subplot(2,2,4)
plot(t,rad2deg(AoA))
title('AoA')

%% Pusher prop Fx Force
% Fx = K1*rpm^2+K2*V*rpm+K3*V
%pusher_RPM2Fx_coeff = [3.703544293799073e-07 -6.005961822003111e-05 0]; %without offset
pusher_RPM2Fx_coeff = [3.680278471152158e-07 -4.391797235636297e-05 -8.651770063289748E-2];

pusher_RPM2Fx_no_V2 = @ (RPM,V) pusher_RPM2Fx_coeff(1) .* RPM.^2   +...
                          pusher_RPM2Fx_coeff(2) .* RPM .* V +...
                          pusher_RPM2Fx_coeff(3) .* V;
pusher_RPM2Fx_V2 = 0;

%% Body Fx Drag Force
%Fx = (k1*cos(skew)+k2+k3*alpha+k4*alpha^2)*V^2

%drag_body_coeff = [0                     -3.0061458379662E-2   3.497371379286E-3    1.46865971330522E-1];%all airspeeds without hover props without skew
%drag_body_coeff = [-9.013405108486905E-3 -1.988035608425628E-2 9.850048188379294E-4 1.443975207474472E-1]; %all airspeeds without hover props with skew

drag_body_coeff = [0                     -3.2663240761150E-2   2.720617256903E-3    1.53265498955988E-1];%low airspeeds without hover props without skew

drag_body_no_V2 = 0;
drag_body_V2 = @(alpha,skew) drag_body_coeff(1)  .*  cos(skew)+...
                              drag_body_coeff(2)+...
                              drag_body_coeff(3)  .*  alpha+...
                              drag_body_coeff(4)  .*  alpha.^2;

%% Hover Prop Fx Drag
%Fx_hover = K1*V^2+K2*sqrt(V)*RPM^2
%drag_hover_coeff = [-6.435825732350349E-3 -1.180349532783032E-7];
%drag_hover_props_no_V2 = @(V,RPM) drag_hover_coeff(2).*sqrt(V).*RPM.^2;
%drag_hover_props_V2 = drag_hover_coeff(1);

%Fx = K1*V^2+K2*RPM^2
%Fx hover props = 
drag_hover_coeff = [-1.188293370185022E-2 -3.133468238831906E-7];

drag_hover_props_no_V2 = @(RPM) drag_hover_coeff(2).*RPM.^2;
drag_hover_props_V2 = drag_hover_coeff(1);

%% Wing Fx Drag
%Fx1 = (k1+(k2*alpha+k3*alpha^2))*(sin(skew)^2+k4)*V^2

drag_wing_coeff = [-0.007875783089360 0.154235238409942 0.697527217260195 -0.041059621726089];

drag_wing_V2 = @(alpha,skew) (drag_wing_coeff(1)+(drag_wing_coeff(2)*alpha+drag_wing_coeff(3)*alpha^2))*(sin(skew)^2+drag_wing_coeff(4));
drag_wing_no_V2 = 0;

%% Airspeed Estimation
wing = 0;
with_accel = 1;

results = struct();

V_pitot;
a_x;
pusher_rpm;
hover_rpm;
AoA;
skew_rad;

last_V = 0;
for i=1:length(t)

    results.no_V2.pusher(i) = pusher_RPM2Fx_no_V2(pusher_rpm(i),last_V);
    results.no_V2.drag_body(i) = drag_body_no_V2;
    results.no_V2.drag_hover_props(i) = drag_hover_props_no_V2(hover_rpm(i));
    results.no_V2.wing(i) = wing.*drag_wing_no_V2; 

    results.V2.pusher(i) = pusher_RPM2Fx_V2;
    results.V2.drag_body(i) = drag_body_V2(AoA(i),skew_rad(i));
    results.V2.drag_hover_props(i) = drag_hover_props_V2;
    results.V2.wing(i) = wing.*drag_wing_V2(AoA(i),skew_rad(i)); 

    F_no_V2 = pusher_RPM2Fx_no_V2(pusher_rpm(i),last_V)+drag_body_no_V2+drag_hover_props_no_V2(hover_rpm(i))+wing.*drag_wing_no_V2;
    F_V2 = pusher_RPM2Fx_V2+drag_body_V2(AoA(i),skew_rad(i))+drag_hover_props_V2+wing.*drag_wing_V2(AoA(i),skew_rad(i));
    
    results.V_est(i) = real(sqrt((with_accel.*m.*a_x(i)-F_no_V2)./(F_V2)));
    
    last_V = V_pitot(i);%results.V_est(i);

    %if t(i)>259
    %    fprintf('Here');
    %end

end

figure;
plot(t,results.V_est)
hold on
plot(t,V_pitot)
legend('Estimation','Pitot')

%figure
%subplot(2,2,1)
%plot(t,results.no_V2.pusher)

%% Evolution of forces in flight

figure

subplot(4,1,1)
plot(t,pusher_RPM2Fx_no_V2(pusher_rpm,V_pitot))
hold on
plot(t,drag_body_no_V2)
plot(t,drag_hover_props_no_V2(hover_rpm))
legend('Pusher Force','Body drag','Hover Props Drag')
xlabel('Time [s]')
ylabel('Force [N]')
title('No V2')

subplot(4,1,2)
plot(t,pusher_RPM2Fx_V2.*V_pitot.^2)
hold on
plot(t,drag_body_V2(AoA,skew_rad).*V_pitot.^2)
plot(t,drag_hover_props_V2.*V_pitot.^2)
legend('Pusher Force','Body drag','Hover Props Drag')
xlabel('Time [s]')
ylabel('Force [N]')
title('No V2')

subplot(4,1,3)
plot(t,V_pitot)
xlabel('Time [s]')
ylabel('Airspeed [m/s]')

subplot(4,1,4)
plot(t,pusher_RPM2Fx_no_V2(pusher_rpm,V_pitot)+drag_hover_props_no_V2(hover_rpm)+drag_body_V2(AoA,skew_rad).*V_pitot.^2+drag_hover_props_V2.*V_pitot.^2)
hold on
%% Linear Fit on flight test data
% Fx_pusher+Dx = 0
% pprz2Fx(pusher_pprz,V_pitot) = K*V

y = [pusher_RPM2Fx_no_V2(pusher_rpm,V_pitot)];
X  = [V_pitot];

b1 = (X'*X)\X'*y;

% Fx_pusher+Dx = 0
% pprz2Fx(pusher_pprz,V_pitot) = K*V^2

y2 = [pusher_RPM2Fx_no_V2(pusher_rpm,V_pitot)];
X2  = [V_pitot.^2];

b2 = (X2'*X2)\X2'*y2;

%% 
figure
plot(t,V_pitot)
hold on
plot(t,pusher_RPM2Fx_no_V2(pusher_rpm,V_pitot)./b1)
plot(t,sqrt(pusher_RPM2Fx_no_V2(pusher_rpm,V_pitot)./b2))
k(1) = pusher_RPM2Fx_coeff(1);
k(2)= pusher_RPM2Fx_coeff(2);
k(3) = pusher_RPM2Fx_coeff(3);
k(4) = b2;
x = pusher_rpm;
V_est_quad = @(x,k) -(sqrt((k(2).*x+k(3)).^2-4.*k(1).*-k(4).*x.^2)+k(2).*x+k(3))./(2*-k(4));
plot(t,V_est_quad(x,k))
legend('Pitot','V1','V2','Direct')

RPM_est_quad = @(y,k) (sqrt(y.*(k(2).^2.*y-4.*k(1).*(k(3)-k(4).*y)))-k(2).*y)./(2.*k(1));
figure
plot(t,pusher_rpm)
hold on
plot(t,RPM_est_quad(V_pitot,k))
%% Run online

V_est_with_accel = zeros(size(t));
V_est_without_accel = zeros(size(t));
V_est_lin = zeros(size(t));
V_est_quad = zeros(size(t));

last_V_est_with_accel = 0;last_V_est_without_accel = 0;
last_V_est_lin = 0;last_V_est_quad = 0;

for i=1:length(t)

V_est_with_accel(i) = real(sqrt((m*a_x(i)+--pusher_RPM2Fx(pusher_rpm(i),last_V_est_without_accel)-drag_hover_props(pusher_rpm(i),last_V_est_without_accel))./drag_body(AoA(i),skew_rad(i))));
V_est_without_accel(i) = real(sqrt((-pusher_RPM2Fx(pusher_rpm(i),last_V_est_without_accel)-drag_hover_props(pusher_rpm(i),last_V_est_without_accel))./drag_body(AoA(i),skew_rad(i))));

%V_est_lin(i) = -pusher_RPM2Fx(pusher_rpm(i),last_V_est_lin)-drag_hover_props(pusher_rpm(i),last_V_est_lin)./b;
%V_est_quad(i) = real(sqrt(-pusher_RPM2Fx(pusher_rpm(i),last_V_est_quad)-drag_hover_props(pusher_rpm(i),last_V_est_quad)./b2));

last_V_est_with_accel = V_est_with_accel(i);last_V_est_without_accel = V_est_without_accel(i);
last_V_est_lin = V_est_lin(i);last_V_est_quad = V_est_quad(i);

end
error_V_est_with_accel = error_quantification(V_est_with_accel,V_pitot);
error_V_est_without_accel = error_quantification(V_est_without_accel,V_pitot);
%error_V_est_lin = error_quantification(V_est_lin,V_pitot);
%error_V_est_quad = error_quantification(V_est_quad,V_pitot);

%lgd_lbl = {sprintf('With accel RMS=%2.2f',error_V_est_with_accel.error_RMS),...
%       sprintf('Without accel RMS=%2.2f',error_V_est_without_accel.error_RMS),...
%       sprintf('Linear RMS=%2.2f',error_V_est_lin.error_RMS),...
%       sprintf('Quadratic RMS=%2.2f',error_V_est_quad.error_RMS),...
%       'Pitot'};
%lgd_lbl = {sprintf('Wind Tunnel Coeff Quadratic AoA Quadratic V RMS=%2.2f',error_V_est_without_accel.error_RMS),...
%       sprintf('Test Coeff Linear V RMS=%2.2f',error_V_est_lin.error_RMS),...
%       sprintf('Test Coeff Quadratic V RMS=%2.2f',error_V_est_quad.error_RMS),...
%       'Pitot'};

%V_GPS = sqrt(Vg_NED.flight.data(:,2).^2+Vg_NED.flight.data(:,1).^2);
%psi_GPS = atan2(Vg_NED.flight.data(:,1),Vg_NED.flight.data(:,2));
