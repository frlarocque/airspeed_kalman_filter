clear all

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
test_codes = unique(full_db.Code);

test = 'DT';
idx = contains(full_db.Code,test);

test_db = full_db(idx,:);

% Transform all angles in rad
deg_columns = {'Turn_Table','Skew','Skew_sp','Pitch','AoA','std_AoA'};
test_db{:,deg_columns} = deg2rad(test_db{:,deg_columns});

% Save
save(['db_',test,'.mat'],'test_db')

%% Delete unecessary columns
all_columns = test_db.Properties.VariableNames;
columnsToKeep = {'Code','ID','Windspeed','Skew','Skew_sp','Pitch','Turn_Table','Mx','My','Mz','Fx','Fy','Fz','std_Mx','std_My','std_Mz','std_Fx','std_Fy','std_Fz','std_AoA'};
test_db(:,~ismember(all_columns,columnsToKeep)) = [];

%% Select only sideslip smaller than +-90 deg

test_db = test_db(abs(test_db.Turn_Table)<deg2rad(90),:);

%% Plot
figure
subplot(2,2,1)
plot(rad2deg(test_db.Turn_Table),test_db.Fy,'*')
xlabel('Turn table angle [deg]')
ylabel('F_y [N]')
grid on

subplot(2,2,2)
plot(rad2deg(test_db.Skew_sp),test_db.Fy,'*')
xlabel('Skew Setpoint [deg]')
ylabel('F_y [N]')

subplot(2,2,3)
plot(rad2deg(test_db.Turn_Table),test_db.Fy./(test_db.Windspeed.^2),'*')
xlabel('Turn table angle [deg]')
ylabel('F_y/Airspeed^2 [N/((m/s)^2)]')
grid on

subplot(2,2,4)
plot(rad2deg(test_db.Skew_sp),test_db.Fy./(test_db.Windspeed.^2),'*')
xlabel('Skew Setpoint [deg]')
ylabel('F_y/Airspeed^2 [N/((m/s)^2)]')
sgtitle('Initial Data from Wind Tunnel tests')

%% Fit on all data
% k  = [k_beta,k_skew]
% x = [beta,skew,V]

x = [test_db.Turn_Table,test_db.Skew_sp,test_db.Windspeed];
y = [test_db.Fy];

fit_quad = @(k,x)  (k(1).*x(:,1)+k(2).*x(:,2)).*x(:,3).^2;    % Function to fit
fcn_quad = @(k) sqrt(mean((fit_quad(k,x) - y).^2));           % Least-Squares cost function
[s_quad,RMS_all_quad] = fminsearch(fcn_quad,[1E-1;1E-1])  

% s_quad =
% 
%     0.1394
%     0.0023
% 
% 
% RMS_all_quad =
% 
%     0.5249

k_beta_quad = s_quad(1); %N/(rad (m/s)^2)
k_skew_quad = s_quad(2); %N/(rad (m/s)^2)

fit_lin = @(k,x)  (k(1).*x(:,1)+k(2).*x(:,2)).*x(:,3);    % Function to fit
fcn_lin = @(k) sqrt(mean((fit_lin(k,x) - y).^2));               % Least-Squares cost function
[s_lin,RMS_all_lin] = fminsearch(fcn_lin,[1E-1;1E-1])  

% s_lin =
% 
%     1.5898
%     0.0783
% 
% 
% RMS_all_lin =
% 
%     3.3391

k_beta_lin = s_lin(1); %N/(rad (m/s))
k_skew_lin = s_lin(2); %N/(rad (m/s))

fit_zero = @(k,x)  (k(1).*x(:,1)+k(2).*x(:,2));    % Function to fit
fcn_zero = @(k) sqrt(mean((fit_zero(k,x) - y).^2));               % Least-Squares cost function
[s_zero,RMS_all_zero] = fminsearch(fcn_zero,[1E-1;1E-1])  

% s_zero =
% 
%    13.5926
%     1.4531
% 
% 
% RMS_all_zero =
% 
%     7.4743

k_beta_zero = s_zero(1); %N/(rad (m/s))
k_skew_zero = s_zero(2); %N/(rad (m/s))

% HENCE:
% Fy = (beta [rad] *1.3939*1E-1+skew [rad]*2.2888*1E-3 )* V [m/s]^2
% or
% Fy = (beta [rad] *1.5898*1E0+skew [rad]*7.8329*1E-2 )* V [m/s]
% or
% Fy = (beta [rad] *1.35926*1E1+ skew [rad]*1.4531*1E0)

%% Visual Verification
figure
subplot(1,2,1)
plot3(rad2deg(test_db.Turn_Table),rad2deg(test_db.Skew_sp),test_db.Fy./(test_db.Windspeed.^2),'*')
xlabel('Turn table angle [deg]')
ylabel('Skew Setpoint [deg]')
zlabel('F_y/Airspeed^2 [N/((m/s)^2)]')
grid on

hold on
beta = linspace(min(test_db.Turn_Table),max(test_db.Turn_Table),10);
skew = linspace(min(test_db.Skew_sp),max(test_db.Skew_sp),10);
[BETA,SKEW] = meshgrid(beta,skew);
surf(rad2deg(BETA),rad2deg(SKEW),(k_beta_quad*BETA+k_skew_quad*SKEW))

subplot(1,2,2)
plot3(rad2deg(test_db.Turn_Table),test_db.Windspeed,test_db.Fy,'*')
xlabel('Turn table angle [deg]')
ylabel('Wind Speed [m/s]')
zlabel('F_y [N]')
grid on
hold on
beta = linspace(min(test_db.Turn_Table),max(test_db.Turn_Table),10);
wind = linspace(min(test_db.Windspeed),max(test_db.Windspeed),10);
[BETA,WIND] = meshgrid(beta,wind);
surf(rad2deg(BETA),WIND,(k_beta_quad*BETA.*WIND.^2))
sgtitle(sprintf('Quadratic airspeed fit on skew and turn table with k_{beta}=%2.2e [N/(rad*(m/s)^2)] and k_{skew}=%2.2e [N/(rad*(m/s)^2)]\n RMS %2.2f [N]',k_beta_quad,k_skew_quad,RMS_all_quad))

%Fy_est = test_db.Windspeed.^2.*(k_beta_quad.*test_db.Turn_Table+k_skew.*test_db.Skew_sp);
%[error] = error_quantification(Fy_est,test_db.Fy);

%% Fit skew on sideslip=0
skew_db = test_db(test_db.Turn_Table==0,:);

% k  = [k_skew]
% x = [skew,V]
x = [skew_db.Skew_sp,skew_db.Windspeed];
y = [skew_db.Fy];
    
fit_skew = @(k,x)  k(1).*x(:,1).*x(:,2).^2;          % Function to fit
fcn_skew = @(k) sqrt(mean((fit_skew(k,x) - y).^2));  % Least-Squares cost function
[s_skew,RMS_skew] = fminsearch(fcn_skew,[1E-1])  

% s_skew =
% 
%     0.0010
% 
% 
% RMS_skew =
% 
%     0.3186

figure
plot(rad2deg(skew_db.Skew_sp),skew_db.Fy./(skew_db.Windspeed.^2),'*')
hold on
plot(rad2deg(linspace(min(skew_db.Skew_sp),max(skew_db.Skew_sp),10)),s_skew.*linspace(min(skew_db.Skew_sp),max(skew_db.Skew_sp),10))
xlabel('Turn table angle [deg]')
ylabel('F_y/Airspeed^2 [N/((m/s)^2)]')
grid on
title(sprintf('Quadratic airspeed fit on skew alone with k_{skew}=%2.2e [N/(rad*(m/s)^2]\n RMS [N] %2.2f',s_skew,RMS_skew))
legend('Data','Fit')

%% Fit sideslip on skew=0
sideslip_db = test_db(test_db.Skew_sp==0,:);

% k  = [k_beta]
% x = [beta,V]
x = [sideslip_db.Turn_Table,sideslip_db.Windspeed];
y = [sideslip_db.Fy];

fit_beta = @(k,x)  k(1).*x(:,1).*x(:,2).^2;         % Function to fit
fcn_beta = @(k) sqrt(mean((fit_beta(k,x) - y).^2)); % Least-Squares cost function
[s_beta,RMS_beta] = fminsearch(fcn_beta,[1E-2])  

% s_beta =
% 
%     0.1384
% 
% 
% RMS_beta =
% 
%     0.2512

figure
plot(rad2deg(sideslip_db.Turn_Table),sideslip_db.Fy./(sideslip_db.Windspeed.^2),'*')
hold on
plot(rad2deg(linspace(min(sideslip_db.Turn_Table),max(sideslip_db.Turn_Table),10)),s_beta.*linspace(min(sideslip_db.Turn_Table),max(sideslip_db.Turn_Table),10))
title(sprintf('Quadratic airspeed fit on beta alone with k_{beta}=%2.2e [N/(rad*(m/s)^2]\n RMS [N] %2.2f',s_beta,RMS_beta))
legend('Data','Fit')