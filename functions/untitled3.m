%% Plot A_x

cond = t>0 & t<12275;
figure
ax1 = subplot(2,2,1);
plot(t(cond),pusher_prop_pwm_filt(cond))
grid on
ylabel('PWM')

ax2 = subplot(2,2,2);
plot(t(cond),a_x_filt(cond))
hold on
%plot(t,IMU_accel.flight.data(:,1))
ylabel('Acc')

ax3 = subplot(2,2,3);
plot(t(cond),airspeed_pitot.flight.data(cond))
ylabel('u')

ax4 = subplot(2,2,4);
% a = F/a = F_hover_prop
m = 4.2;
u = airspeed_pitot.flight.data(cond);
Fx_pusher_coeff = [2.9E-5 0 0];%[3.96222948E-07 -5.2930351318E-05 -2.68843366027904E-01];
Fx_push =     Fx_pusher_coeff(1) .* (pusher_prop_pwm_filt(cond)-1000).^2   +...
              Fx_pusher_coeff(2) .* (pusher_prop_pwm_filt(cond)-1000) .* u +...
              Fx_pusher_coeff(3) .* u;
Fx_push(pusher_prop_pwm_filt(cond)<1050) = 0;

plot(t(cond),a_x_filt(cond)-(Fx_push + -8E-2.*u - 6E-2.*u.*u.*sign(u))./m)

%% Data from manufacturer
pct = [30 40 50 60 70 80 90 100];
pwm = pct/100*1000+1000;
RPM = [5425 7362 9238 10904 12423 14262 15826 16416];
T = [250 470 780 1120 1480 1990 2490 2750]*9.81/1000;
pwm2T = polyfit(pwm(1:end-1),T(1:end-1),2);
pwm2RPM = polyfit(pwm(1:end-1),RPM(1:end-1),1);

rho = 1.225; %kg/m^2
d = 9.*2.54/100; %m
V = 5; %m/s

CT = T./(rho*(RPM/60).^2*d.^4);

J = V./((RPM/60).*d); 
T_est = @ (RPM,rho,d,J) (-0.2*J+0.105).*rho.*(RPM/60).^2.*d.^4;

figure
subplot(2,1,1)
plot(pwm,T)
axis([1000 2000 0 inf])
hold on
plot([1000:100:2000],polyval(pwm2T,[1000:100:2000]))
plot(pwm,T_est(polyval(pwm2RPM,pwm),rho,d,J))
legend('data','polyfit','Ct est')
xlabel('PWM Signal')
ylabel('Thrust [N]')
grid on


subplot(2,1,2)
plot(pwm,RPM)
hold on
plot([1000:100:2000],polyval(pwm2RPM,[1000:100:2000]))
legend('Data','Linear Fit')
xlabel('PWM Signal')
ylabel('RPM')
grid on
axis([1000 2000 0 inf])

%% Fit drag
x = [pusher_prop_pwm_filt(cond),airspeed_pitot.flight.data(cond)];
y = a_x_filt(cond);

fit = @(k,x)  (k(1).*Fx_pusher(polyval(pwm2RPM,x(:,1)),x(:,2)) + k(2).* x(:,2) + k(3).*x(:,2).^2.*sign(x(:,2)) )./4.2;          % Function to fit
fcn = @(k) sqrt(mean((fit(k,x) - y).^2));  % Least-Squares cost function
[s,RMS] = fminsearchbnd(fcn,[1       -6E-1  -7E-2],...
                            [0.9      -1E0  -1E0 ],...
                            [1.1       0      0  ])  

%'WINDTUNNEL_80_01_08__20_25_02_SD_3_short.mat'
% s =
% 
%    0.900000000000729  -0.647866234080218  -0.000000000013941

% 'WINDTUNNEL_80_01_08__20_47_28_SD_2_short.mat'
% s =
% 
%    0.900000000011104  -0.200200112668264  -0.067925885187740

figure
plot(t(cond),y)
hold on
plot(t(cond),fit(s,x))
legend('y','fit')

%% A_z with hover prop only

cond = t>800 & t<900;

x = [hover_prop_pwm_filt(cond)];
y = a_z_filt(cond);

fit = @(k,x)  (k(1).*(x(:,1)-1000).^2)./4.2;          % Function to fit
fcn = @(k) sqrt(mean((fit(k,x) - y).^2));  % Least-Squares cost function
[s,RMS] = fminsearchbnd(fcn,[-5.6e-05],[-1],[0])  

figure
plot(t(cond),y)
hold on
plot(t(cond),fit(s,x))

%ax3 = subplot(2,2,4);
% a = F/a = F_hover_prop
%m = 4.2;
%F_hover_prop = (-4*1.94E-5.*(hover_prop_pwm_filt-1000).^2);
%plot(t,(F_hover_prop./m)-a_z_filt);

%% A_z wing only

%
cond = t>550 & t<560;
figure
ax1 = subplot(2,2,1);
plot(t(cond),hover_prop_pwm_filt(cond))
grid on
ylabel('PWM')

ax2 = subplot(2,2,2);
plot(t(cond),a_z_filt(cond))
ylabel('Accel')

ax3 = subplot(2,2,3);
plot(t(cond),rad2deg(IMU_angle.flight.data(cond,2)));
ylabel('theta')

ax4 = subplot(2,2,4);
plot(t(cond),airspeed_pitot.flight.data(cond))
ylabel('u')

cl_max = 1.3;  % [-]                          
S_wing = 1.56 * 0.235; % [m^2]
rho = 1.225; % [kg/m^3]
alpha = IMU_angle.flight.data(cond,2);
V = airspeed_pitot.flight.data(cond);


lift_w = @ (V,aoa,lambda)  -0.5.*rho.*S_wing.*V.^2.*(-1.8847.*aoa.*sin(lambda).^2 + ...
                                              1.1.*-0.2780.*sin(lambda).^2+ ...
                                              -1.5037.*aoa + ...
                                              -0.0043.*1);

lift_w_without_v2 = @ (aoa,lambda) -0.5.*rho.*S_wing.*(-1.8847.*aoa.*sin(lambda).^2 + ...
                                              1.1.*-0.2780.*sin(lambda).^2+ ...
                                              -1.5037.*aoa + ...
                                              -0.0043.*1);

L = lift_w(V,alpha,deg2rad(90));
D = V.^2.*0.02 + L.^2./(pi*0.8*8);

figure
subplot(2,1,1)
plot(t(cond),(-L.*cos(alpha))./4.2)
hold on
%plot(t(cond),-D.*sin(alpha)./4.2)
plot(t(cond),a_z_filt(cond))
legend('L','Az')

subplot(2,1,2)
plot(t(cond),V)
hold on
plot(t(cond),sqrt(a_z_filt(cond)*4.2./-lift_w_without_v2(alpha,deg2rad(90))))


%% Ay

cond = t>535 & t<585;

figure
ax1 = subplot(2,2,1);
plot(t(cond),a_y_filt(cond))
ylabel('a_y')

ax2 = subplot(2,2,2);
plot(t(cond),rad2deg(IMU_angle.flight.data((cond),2)))
hold on
plot(t(cond),rad2deg(IMU_angle.flight.data((cond),3)))
ylabel('angle')
legend('Theta','Psi')

ax3 = subplot(2,2,3);
plot(t(cond),rad2deg(skew_filt(cond)))
hold on
plot(t(cond),ac_data.EFF_FULL_INDI.wing_angle_deg_sp(cond))
ylabel('skew')
legend('Skew','SP')

ax4 = subplot(2,2,4);
plot(t(cond),airspeed_pitot.flight.data(cond))
ylabel('u')

linkaxes([ax1,ax2,ax3,ax4],'x')

%%

x = [IMU_angle.flight.data(cond,2),airspeed_pitot.flight.data(cond),deg2rad(ac_data.EFF_FULL_INDI.wing_angle_deg_sp(cond)),tan(IMU_angle.flight.data(cond,3)).*airspeed_pitot.flight.data(cond)];
%x = [IMU_angle.flight.data(cond,2),airspeed_pitot.flight.data(cond),skew_filt(cond),tan(IMU_angle.flight.data(cond,3)).*airspeed_pitot.flight.data(cond)];

y = a_y_filt(cond);

% ((K1+K2*aoa)*sin(skew)*cos(skew))*u^2+K3*v^2*sign(v)+K4*cos(skew)+K5*sin(skew)
fit = @(k,x)  ((k(1)+k(2).*x(:,1)).*cos(x(:,3)).*sin(x(:,3))).*x(:,2).^2+k(3).*x(:,4).^2.*sign(x(:,4))+(cos(x(:,3)).*k(4)+sin(x(:,3)).*k(5)).*x(:,2).^2;          % Function to fit
fcn = @(k) sqrt(mean((fit(k,x) - y).^2));  % Least-Squares cost function
[s,RMS] = fminsearchbnd(fcn,[0 0 0 0 0],[-10 -10 -10 0 -10],[10 10 10 0 10])  

figure
plot(t(cond),y)
hold on
plot(t(cond),fit(s,x))
legend('a_y','fit')


% a_y
