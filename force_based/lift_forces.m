clear all

options = optimset('TolFun',1E-5,'TolX',1E-5);

% Add all paths
addpath('/home/frederic/Documents/thesis/tools/airspeed_estimation/functions/');

%% Import all database
[file,path] = uigetfile({'*.xlsx'},'Select a file');

full_db = readtable(fullfile(path,file));

%% Select test
% AA: Angle of attack
% AE: Actuator effectiveness
% DT: Drag test
% EP: Elevator precision
% ES: Elevator stall
% LP: Lift test (pitch changes)


% LP1: a/c w/o pusher
% LP2: a/c w/o pusher w/o elevator
% LP3: a/c w/o pusher w/o wing
% LP4: a/c w/o pusher w/o wing w/o hover props
% LP5: a/c w/o pusher w/o elevator w/o hover props
% LP6: a/c w/o pusher w/o hover props

test_codes = unique(full_db.Code);

test = 'LP6';
idx = contains(full_db.Code,test);

test_db = full_db(idx,:);

% Transform all angles in rad
deg_columns = {'Turn_Table','Skew','Skew_sp','Pitch','AoA','std_AoA'};
test_db{:,deg_columns} = deg2rad(test_db{:,deg_columns});

% Save
save(['db_',test,'.mat'],'test_db')

%% Remove entries

test_db = test_db(test_db.Mot_F<1000,:);
test_db(abs(test_db.Turn_Table)>deg2rad(15),:) = []; %remove higher than 15 deg angle of attack points

%% Add Lift and Drag Entries
% Defining lift as perpendicular to speed vector

% Fz points up instead of down
for i=1:size(test_db,1)
    temp_A = [sin(test_db.Turn_Table(i)) -cos(test_db.Turn_Table(i));-cos(test_db.Turn_Table(i)) -sin(test_db.Turn_Table(i))];
    temp_b = [test_db.Fx(i);-test_db.Fz(i)];

    temp_x = inv(temp_A)*temp_b;

    test_db.Lift(i) = temp_x(1);
    test_db.Drag(i) = temp_x(2);
end

%% Plot
figure
subplot(2,2,1)
plot(rad2deg(test_db.Turn_Table),test_db.Fz,'*')
xlabel('Turn table angle (pitch) [deg]')
ylabel('F_z [N]')
grid on

subplot(2,2,2)
plot(rad2deg(test_db.Skew_sp),test_db.Fz,'*')
xlabel('Skew Setpoint [deg]')
ylabel('F_z [N]')

subplot(2,2,3)
plot(rad2deg(test_db.Turn_Table),test_db.Fz./(test_db.Windspeed.^2),'*')
xlabel('Turn table angle (pitch) [deg]')
ylabel('F_z/Airspeed^2 [N/((m/s)^2)]')
grid on

subplot(2,2,4)
plot(rad2deg(test_db.Skew_sp),test_db.Fz./(test_db.Windspeed.^2),'*')
xlabel('Skew Setpoint [deg]')
ylabel('F_z/Airspeed^2 [N/((m/s)^2)]')
sgtitle('Initial Data from Wind Tunnel tests')

%% Find Lift and drag of vehicle at 90 skew (forward flight)
ff_db = test_db(test_db.Skew_sp==deg2rad(90),:);

% Remove data at 5 m/s, which has a different CL_0 (Reason: pusher props?)
%ff_db = ff_db(ff_db.Windspeed>5 & ff_db.Windspeed<15,:);

figure
subplot(1,2,1)
plot(rad2deg(ff_db.Turn_Table),ff_db.Fz,'*')
xlabel('Turn table angle (pitch) [deg]')
ylabel('F_z [N]')
grid on

subplot(1,2,2)
plot(rad2deg(ff_db.Turn_Table),ff_db.Fz./(ff_db.Windspeed.^2),'*')
xlabel('Turn table angle (pitch) [deg]')
ylabel('F_z/Airspeed^2 [N/((m/s)^2)]')
grid on

% L = 1/2 rho S * (CL_0+CL_alpha*alpha)*V^2
% k  = [CL_0 CL_alpha 1/2rhoS]
% x = [theta,V]
x = [ff_db.Turn_Table,ff_db.Windspeed];
y = [ff_db.Fz];

fit = @(k,x)  k(3).*x(:,2).^2.*(k(1)+k(2).*x(:,1));    % Function to fit
fcn = @(k) sqrt(mean((fit(k,x) - y).^2));           % Least-Squares cost function
[s_ff,RMS] = fminsearch(fcn,[0.3;5;0.5])

subplot(1,2,2)
hold on
plot(rad2deg(linspace(min(ff_db.Turn_Table),max(ff_db.Turn_Table),10)),s_ff(3).*(s_ff(1)+s_ff(2)*linspace(min(ff_db.Turn_Table),max(ff_db.Turn_Table),10)))
legend('Data','Fit')

subplot(1,2,1)
hold on
plot(rad2deg(ff_db.Turn_Table),ff_db.Windspeed.^2.*s_ff(3).*(s_ff(1)+s_ff(2)*ff_db.Turn_Table),'*')
legend('Data','Fit')
sgtitle(sprintf('Quadratic airspeed fit (Skew=90 deg) on pitch angle with C_{L_{0}}=%2.2f C_{L_{a}}=%2.2f S = %2.2f \n RMS %2.2f',s_ff(1),s_ff(2),s_ff(3)/(0.5*1.225),RMS))

%% Find Lift and drag of vehicle at 0 skew (quad mode)
quad_db = test_db(test_db.Skew_sp==deg2rad(0),:);

% Remove data at 5 m/s, which has a different CL_0 (Reason?)
quad_db = quad_db(quad_db.Windspeed>5 & quad_db.Windspeed<15,:);

figure
subplot(1,2,1)
plot(rad2deg(quad_db.Turn_Table),quad_db.Fz,'*')
xlabel('Turn table angle (pitch) [deg]')
ylabel('F_z [N]')
grid on

subplot(1,2,2)
plot(rad2deg(quad_db.Turn_Table),quad_db.Fz./(quad_db.Windspeed.^2),'*')
xlabel('Turn table angle (pitch) [deg]')
ylabel('F_y/Airspeed^2 [N/((m/s)^2)]')
grid on

% L = 1/2 rho S * (CL_0+CL_alpha*alpha)*V^2
% k  = [CL_0 CL_alpha 1/2rhoS]
% x = [theta,V]
x = [quad_db.Turn_Table,quad_db.Windspeed];
y = [quad_db.Fz];

fit = @(k,x)  k(3).*x(:,2).^2.*(k(1)+k(2).*x(:,1));    % Function to fit
fcn = @(k) sqrt(mean((fit(k,x) - y).^2));           % Least-Squares cost function
[s_quad,RMS] = fminsearch(fcn,[0.3;5;0.5])

subplot(1,2,2)
hold on
plot(rad2deg(linspace(min(quad_db.Turn_Table),max(quad_db.Turn_Table),10)),s_quad(3).*(s_quad(1)+s_quad(2)*linspace(min(quad_db.Turn_Table),max(quad_db.Turn_Table),10)))
legend('Data','Fit')

subplot(1,2,1)
hold on
plot(rad2deg(quad_db.Turn_Table),quad_db.Windspeed.^2.*s_quad(3).*(s_quad(1)+s_quad(2)*quad_db.Turn_Table),'*')
legend('Data','Fit')
sgtitle(sprintf('Quadratic airspeed fit (Skew=0 deg) on pitch angle with C_{L_{0}}=%2.2f C_{L_{a}}=%2.2f S = %2.2f \n RMS %2.2f',s_quad(1),s_quad(2),s_quad(3)/(0.5*1.225),RMS))

%% Compare forward flight vs quad mode

s_ff-s_quad


% LP1 
% ans =
% 
%     0.0024
%    -2.2688
%     0.3125
% Pretty much the same CL_0
% CL_alpha is much higher for quad mode
% Surface is bigger for forward flight mode

%% Fit for fixed airspeed, variable skew
% Select fixed airspeed
skew_db = test_db(test_db.Windspeed>0 & test_db.Windspeed<15,:);

figure
subplot(1,2,1)
plot(rad2deg(skew_db.Turn_Table),skew_db.Fz,'*')
xlabel('Turn table angle (pitch) [deg]')
ylabel('F_z [N]')
grid on

subplot(1,2,2)
plot(rad2deg(skew_db.Turn_Table),skew_db.Fz./(skew_db.Windspeed.^2),'*')
xlabel('Turn table angle (pitch) [deg]')
ylabel('F_y/Airspeed^2 [N/((m/s)^2)]')
grid on

% L = 1/2 rho S * (CL_0+CL_alpha*alpha) *V^2*(m_1*sin(skew)^2+k_1)
% k  = [CL_0 CL_alpha 1/2rhoS m_1 k_1]
% x = [theta,skew,V]
x = [skew_db.Turn_Table,skew_db.Skew_sp,skew_db.Windspeed];
y = [skew_db.Fz];

fit = @(k,x)  k(3).*x(:,3).^2.*(k(4).*sin(x(:,2)).^2+k(5)).*(k(1)+k(2).*x(:,1));    % Function to fit
fcn = @(k) sqrt(mean((fit(k,x) - y).^2));           % Least-Squares cost function
[s_skew,RMS] = fminsearch(fcn,[0.3;5;0.4;1;1E-1],options)

subplot(1,2,2)
hold on
plot(rad2deg(linspace(min(skew_db.Turn_Table),max(skew_db.Turn_Table),10)),(s_skew(4).*sin(skew_db.Skew_sp).^2+s_skew(5)).*s_skew(3).*(s_skew(1)+s_skew(2)*linspace(min(skew_db.Turn_Table),max(ff_db.Turn_Table),10)))
legend('Data','Fit')

subplot(1,2,1)
hold on
plot(rad2deg(skew_db.Turn_Table),(s_skew(4).*sin(skew_db.Skew_sp).^2+s_skew(5)).*skew_db.Windspeed.^2.*s_skew(3).*(s_skew(1)+s_skew(2)*skew_db.Turn_Table),'*')
legend('Data','Fit')
sgtitle(sprintf('Quadratic airspeed fit on pitch angle with C_{L_{0}}=%2.2f C_{L_{a}}=%2.2f S = %2.2f \n RMS %2.2f',s_skew(1),s_skew(2),s_skew(3)/(0.5*1.225),RMS))


%% Fit for all
% L = 1/2 rho S v^2 * (m_1 sin(skew)^2+k_1)(m_2 theta +k_2)
% L = 1/2 rho S v^2 * (lambda_1 theta sin(skew)^2+lambda_2 sin(skew)^2+
% lambda_3 theta +lambda_4)
% L = lambda_5 v^2 * (lambda_1 theta sin(skew)^2+lambda_2 sin(skew)^2+
% lambda_3 theta +lambda_4)

% L = 1/2 rho S * (CL_0+CL_alpha*alpha) *V^2*(m_1*sin(skew)^2+k_1)

% k  = [CL_0 CL_alpha 1/2rhoS m_1 k_1]
% x = [theta,skew,V]
x = [test_db.Turn_Table,test_db.Skew_sp,test_db.Windspeed];
y = [test_db.Fz];

fit = @(k,x)  k(3).*x(:,3).^2.*(k(4).*sin(x(:,2)).^2+k(5)).*(k(1)+k(2).*x(:,1));    % Function to fit
fcn = @(k) sqrt(mean((fit(k,x) - y).^2));           % Least-Squares cost function
[s,RMS] = fminsearch(fcn,[0.3;5;0.4;1;1E-1],options) 

%% Fit for body only (no influence of skew)
% L = 1/2 rho S * (CL_0+CL_alpha*alpha) *V^2*(m_1*sin(skew)^2+k_1)

% k  = [CL_0 CL_alpha 1/2rhoS]
% x = [theta,skew,V]
x = [test_db.Turn_Table,test_db.Skew_sp,test_db.Windspeed];
y = [test_db.Fz];

fit = @(k,x)  k(3).*x(:,3).^2.*(k(1)+k(2).*x(:,1));    % Function to fit
fcn = @(k) sqrt(mean((fit(k,x) - y).^2));           % Least-Squares cost function
[s,RMS] = fminsearch(fcn,[0.3;5;0.4;],options) 


figure
subplot(1,2,1)
plot(rad2deg(test_db.Turn_Table),test_db.Fz,'*')
xlabel('Turn table angle (pitch) [deg]')
ylabel('F_z [N]')
grid on

subplot(1,2,2)
plot(rad2deg(test_db.Turn_Table),test_db.Fz./(test_db.Windspeed.^2),'*')
xlabel('Turn table angle (pitch) [deg]')
ylabel('F_y/Airspeed^2 [N/((m/s)^2)]')
grid on

subplot(1,2,2)
hold on
plot(rad2deg(linspace(min(test_db.Turn_Table),max(test_db.Turn_Table),10)),s(3).*(s(1)+s(2)*linspace(min(test_db.Turn_Table),max(test_db.Turn_Table),10)))
legend('Data','Fit')

subplot(1,2,1)
hold on
plot(rad2deg(test_db.Turn_Table),test_db.Windspeed.^2.*s(3).*(s(1)+s(2)*test_db.Turn_Table),'*')
legend('Data','Fit')
sgtitle(sprintf('Quadratic airspeed fit (Skew=0 deg) on pitch angle with C_{L_{0}}=%2.2f C_{L_{a}}=%2.2f S = %2.2f \n RMS %2.2f',s(1),s(2),s(3)/(0.5*1.225),RMS))


