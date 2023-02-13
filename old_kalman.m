%%
% 𝑢, 𝑣, 𝑤 = Velocity in the Body Frame
% 𝜙, 𝜃, 𝜓 = Attitude
% 𝜇𝑥, 𝜇𝑦, 𝜇𝑧 = Wind Velocity in Earth Fixed Frame
% 𝑝, 𝑞, 𝑟 = Angular Rates Measured by Gyroscopes
% 𝑎
% 𝑥, 𝑎𝑦, 𝑎𝑧 = Accelerations Measured by Accelerometers
% 𝑉𝑥 , 𝑉𝑦, 𝑉𝑧 = Velocity in Earth Fixed Frame


%x = [u v w phi theta psi mu_x mu_y mu_z];
%u = [a_x a_y a_z p q r];
%y = [V_x V_y V_z alpha beta];

%w_w = [0;0;0]; %noise angular rate
%w_a = [0;0;0]; %noise accelerations
%w_mu =[0;0;0]; %noise wind

%% State dynamics
% F = df/dx | x,u

%x = [u    v  w   phi theta psi mu_x mu_y mu_z]
x =  [100, 0, 0,  0,  0,    0,  10,  0,   0]';

%u = [a_x a_y a_z     p  q r];
u =  [0   0   -9.81   0  0 0]';

%y = [V_x V_y V_z alpha beta];
y = [110,0,0,0,0]';

epsi = 1E-1;

f_val = f(x,u);

g_val = g(x,u);

%% Extended Kalman Filter
clear x_list u_list y_list z_list P K

%x = [u    v  w   phi theta psi mu_x mu_y mu_z]
x_0 =  [0, 0, 0,  0,  0,    0,  0,  0,   0]';

%u = [a_x a_y a_z     p  q r];
u_0 =  [0   0   -9.81   0  0 0]';

%y = [V_x V_y V_z alpha beta];
y_0 = [0,0,0,0,0]';

t = new_time;
dt = mean(new_time(2:end)-new_time(1:end-1));

wind_var = [1 1 1]*1E-6;
Q = diag([accel_var,angular_accel_var,wind_var]); %process noise
R = diag([V_XYZ_var 1E-6 1E-6]); %measurement noise
P = {diag([1 1 1 1 1 1 wind_var])}; %covariance
K = {};

x_list(:,1) = [airspeed_pitot(1,1) 0 0 deg2rad(IMU_angle(1,:)) 3.26 0 0]';
u_list = [IMU_accel deg2rad(IMU_rate)]';
y_list = zeros(length(y_0),length(t));
z_list = [V_XYZ zeros(length(V_XYZ),2)]'; %measurement

epsi = 1E-2;

for k=1:length(t)
    x=x_list(:,k);
    u=u_list(:,k);
    z=z_list(:,k);

    F_val = F(x,u,epsi);
    G_val = G(x,u,epsi);

    % Prediction
    x_pred = x + dt*f(x,u);   
    P_pred = F_val*P{k}*F_val'+Q;

    % Update
    y_list(:,k+1) = z-g(x,u);

    %Kalman gain
    K{k} = P_pred*G_val'*inv(G_val*P_pred*G_val'+R);

    x_list(:,k+1) = x_pred+K{k}*y_list(:,k+1);
    P{k+1} = (eye(length(x_0))-K{k}*G_val)*P_pred;

end


%% Visualizing data

temp_psi = mod(x_list(6,:),2*pi);

figure
ax1 = subplot(2,2,1);
plot(t,x_list([1:3],1:end-1));
hold on
plot(new_time,airspeed_pitot)
title('uvw')
legend('u','v','w')
grid on

ax2 = subplot(2,2,2);
plot(t,rad2deg(x_list([4:6],1:end-1)));
title('ptp')
legend('phi','theta','psi')
grid on

ax3 = subplot(2,2,3);
plot(t,x_list([7:9],1:end-1));
title('mu')
legend('N','E','D')
grid on

linkaxes([ax1,ax2,ax3],'x')