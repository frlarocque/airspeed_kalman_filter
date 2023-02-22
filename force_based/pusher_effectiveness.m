%% NOT VALID FOR USE ONLY FOR PUSHER ONLY AS IT DOES INCLUDES VEHICLE DRAG

%% Init
clear all

options = optimset('TolFun',1E-5,'TolX',1E-5);
show_error_bar = true;

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

test = 'AE';
idx = contains(full_db.Code,test);

pusher_db = full_db(idx,:);

% Transform all angles in rad
deg_columns = {'Turn_Table','Skew','Skew_sp','Pitch','AoA','std_AoA'};
pusher_db{:,deg_columns} = deg2rad(pusher_db{:,deg_columns});

% Save
save(['db_',test,'.mat'],'pusher_db')

%% Drag caused by body
test = 'LP1';
idx = contains(full_db.Code,test);

body_db = full_db(idx,:);

% Transform all angles in rad
deg_columns = {'Turn_Table','Skew','Skew_sp','Pitch','AoA','std_AoA'};
body_db{:,deg_columns} = deg2rad(body_db{:,deg_columns});

% Remove all entries with actuators active
body_db = body_db(body_db.Turn_Table==0 &body_db.Rud==0 & body_db.Elev==0 & body_db.Ail_R==0 & body_db.Ail_L==0 & body_db.Mot_F<1000 &body_db.Mot_R<1000 &body_db.Mot_B<1000 & body_db.Mot_L<1000,:);

% Plot results
figure
skew_bins = deg2rad(unique(round(rad2deg(body_db.Skew_sp),0)));
body_db.skew_bin = deg2rad(round(rad2deg(body_db.Skew_sp),0));
legend_lbl = {};
col=linspecer(length(skew_bins));
hdls = [];
for i=1:length(skew_bins)
    temp_db = body_db(body_db.skew_bin==skew_bins(i),:);

    if show_error_bar; hdls(i) = errorbar(temp_db.Windspeed,temp_db.Fx,temp_db.std_Fx,'*','color',col(i,:));
    else; hdls(i) = plot(temp_db.Windspeed,temp_db.Fx,'*','color',col(i,:)); end
    hold on
    legend_lbl{i} = ['Skew = ',mat2str(rad2deg(skew_bins(i))),' deg'];
end
legend(hdls,legend_lbl,'location','best')
xlabel('Winspeed [m/s]')
ylabel('Fx [N]')
grid on

% Select skew = 0 deg
body_db = body_db(body_db.Skew_sp==0,:);

% Fit data
% Fx = 0.5*rho*S*CD0*V^2
% Fx = K1*V^2
% k  = [k1]
% x = [V]
x = [body_db.Windspeed];
y = [body_db.Fx];

fit_body = @(k,x)  k(1).*x(:,1).^2;    % Function to fit
fcn_body = @(k) sqrt(mean((fit_body(k,x) - y).^2));           % Least-Squares cost function
[s_body,RMS_body] = fminsearch(fcn_body,[-1E-1])

% s_body =
% 
%    -0.0466
% 
% 
% RMS_body =
% 
%     0.2598

% Plot fit
figure
if show_error_bar; errorbar(body_db.Windspeed,body_db.Fx,body_db.std_Fx,'*')
else; plot(body_db.Windspeed,body_db.Fx,'*'); end
hold on
plot(linspace(min(body_db.Windspeed),max(body_db.Windspeed),10),fit_body(s_body,linspace(min(body_db.Windspeed),max(body_db.Windspeed),10)'),'--')
axis([0 inf -inf 0])
xlabel('Winspeed [m/s]')
ylabel('Fx [N]')
grid on
sgtitle(sprintf('Skew = 0 deg\nFbody = K1*V^2  |  K1 = %2.2e  |  RMS = %2.2f',s_body(1),RMS_body))

%Hence
%
% Fx_body = -4.6641E-2*V^2

%% Remove entries from pusher db
% Remove all entries where actuators are used, except Mot_Push
pusher_db = pusher_db(pusher_db.Rud==0 & pusher_db.Elev==0 & pusher_db.Ail_R==0 & pusher_db.Ail_L==0 & pusher_db.Mot_F<1000 &pusher_db.Mot_R<1000 &pusher_db.Mot_B<1000 & pusher_db.Mot_L<1000,:);

% Select skew = 0
pusher_db = pusher_db(pusher_db.Skew_sp==0,:);

% Remove 0 command from fit
pusher_db = pusher_db(pusher_db.Mot_Push>0,:);

%% Plot Pusher Thrust depending on windspeed and pprz signal
figure
windspeed_bins = unique(round(pusher_db.Windspeed,0));
pusher_db.Windspeed_bin = round(pusher_db.Windspeed,0);
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = pusher_db(pusher_db.Windspeed_bin==windspeed_bins(i),:);
    if show_error_bar; hdls(i) = errorbar(temp_db.Mot_Push,temp_db.Fx,temp_db.std_Fx,'*','color',col(i,:));
    else; hdls(i) = plot(temp_db.Mot_Push,temp_db.Fx,'*','color',col(i,:)); end
    
    hold on
    legend_lbl{i} = ['Airspeed = ',mat2str(windspeed_bins(i))];
end
legend(hdls,legend_lbl,'location','best')
xlabel('Pusher Command [pprz]')
ylabel('F_x [N]')
grid on
title('Fx (pusher thrust+body drag) depending on airspeed and command')
axis([0 inf -inf inf])

%% Correct for body drag
% Pusher_db.Fx = body drag + pusher force
% Pusher force = Pusher_db.fx-body drag
pusher_db.Fx_pusher = pusher_db.Fx-fit_body(s_body,pusher_db.Windspeed);

% Calculating std dev of new Fx_pusher
% Mean C = mean A - mean B --> Variance C = Variance A + Variance B - 2*Correlation(A,B)*SD A * SD B
% Assumming uncorrelated
pusher_db.std_Fx_pusher = sqrt(pusher_db.std_Fx.^2+pusher_db.std_Fx.^2);

figure
windspeed_bins = unique(round(pusher_db.Windspeed,0));
pusher_db.Windspeed_bin = round(pusher_db.Windspeed,0);
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = pusher_db(pusher_db.Windspeed_bin==windspeed_bins(i),:);

    if show_error_bar; hdls(i) = errorbar(temp_db.Mot_Push,temp_db.Fx_pusher,temp_db.std_Fx_pusher,'*','color',col(i,:));
    else; hdls(i) = plot(temp_db.Mot_Push,temp_db.Fx_pusher,'*','color',col(i,:)); end
    
    hold on
    legend_lbl{i} = ['Airspeed = ',mat2str(windspeed_bins(i))];
end
legend(hdls,legend_lbl,'location','best')
xlabel('Pusher Command [pprz]')
ylabel('Pusher F_x [N]')
grid on
title('Corrected Pusher Thrust depending on airspeed and command')
axis([0 inf 0 inf])

%% Fit on pprz to RPM

x = [pusher_db.Mot_Push];
y = [pusher_db.rpm_Mot_Push];

fit_pprz2rpm_lin = @(k,x)  k(1).*x(:,1);    % Function to fit
fcn_pprz2rpm_lin = @(k) sqrt(mean((fit_pprz2rpm_lin(k,x) - y).^2));           % Least-Squares cost function
[s_pprz2rpm_lin,RMS_pprz2pwm_lin] = fminsearch(fcn_pprz2rpm_lin,[1E0])

% s_pprz2rpm_lin =
% 
%    1.129296875000000
% 
% 
% RMS_pprz2pwm_lin =
% 
%      3.803132878893272e+02

fit_pprz2rpm_quad = @(k,x)  k(1).*x(:,1)+k(2).*x(:,1).^2;    % Function to fit
fcn_pprz2rpm_quad = @(k) sqrt(mean((fit_pprz2rpm_quad(k,x) - y).^2));           % Least-Squares cost function
[s_pprz2rpm_quad,RMS_pprz2pwm_quad] = fminsearch(fcn_pprz2rpm_quad,[1E0;0])

% s_pprz2rpm_quad =
% 
%    1.378672356359907
%   -0.000037895059856
% 
% 
% RMS_pprz2pwm_quad =
% 
%      1.091531803162169e+02

figure
if show_error_bar; hdls(i) = errorbar(pusher_db.Mot_Push,pusher_db.rpm_Mot_Push,pusher_db.rpm_Mot_Push_std,'*');
    else; plot(pusher_db.Mot_Push,pusher_db.rpm_Mot_Push,'*'); end
    
hold on
plot(linspace(0,max(pusher_db.Mot_Push),10),fit_pprz2rpm_quad(s_pprz2rpm_quad,linspace(0,max(pusher_db.Mot_Push),10)'),'--')
plot(linspace(0,max(pusher_db.Mot_Push),10),fit_pprz2rpm_lin(s_pprz2rpm_lin,linspace(0,max(pusher_db.Mot_Push),10)'),'--')
xlabel('pprz signal [pprz]')
ylabel('RPM [rotation per minute]')
legend('Data','Quadratic Fit','Linear Fit')
grid on
axis([0 inf 0 inf])

% Hence
% RPM = 1.378672356359907 * pprz + -3.789505985611778e-05 * pprz^2
% RMS = 109.1532
%
% or 
%
% RPM = 1.129296875000000 * pprz
% RMS = 380

%% Fit on all RPM
% Delete 0 command signals, as propeller is windmilling
%pusher_db = pusher_db(pusher_db.Mot_Push~=0,:);


% Fx = k1*V*RPM+k_2*RPM^2
% k  = [k1 k_2]
% x = [pprz,V]
x = [pusher_db.rpm_Mot_Push,pusher_db.Windspeed];
y = [pusher_db.Fx_pusher];

fit_all_rpm = @(k,x)  k(1).*(x(:,1)+k(3)).^2+k(2).*(x(:,1)+k(3)).*x(:,2);%k(1).*(x(:,1)+k(2)).^2+k(3).*(x(:,1)+k(2)).*x(:,2)+k(4);    % Function to fit
fcn_all_rpm = @(k) sqrt(mean((fit_all_rpm(k,x) - y).^2));           % Least-Squares cost function
[s_all_rpm,RMS_all_rpm] = fminsearch(fcn_all_rpm,[1E-7;-6E-5;-1000])

% s_all_rpm =
% 
%    1.0e-04 *
% 
%    0.003703544293799
%   -0.600596182200311
% 
% 
% RMS_all_rpm =
% 
%    0.439420256812782

figure
windspeed_bins = unique(round(pusher_db.Windspeed,0));
pusher_db.Windspeed_bin = round(pusher_db.Windspeed,0);
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = pusher_db(pusher_db.Windspeed_bin==windspeed_bins(i),:);
    temp_x = [linspace(0,max(temp_db.rpm_Mot_Push),10)',linspace(0,max(temp_db.Windspeed),10)'];

    if show_error_bar; hdls(i) = errorbar(temp_db.rpm_Mot_Push,temp_db.Fx_pusher,temp_db.std_Fx_pusher,'*','color',col(i,:));
    else; hdls(i) = plot(temp_db.rpm_Mot_Push,temp_db.Fx_pusher,'*','color',col(i,:)); end
    
    hold on
    plot(linspace(0,max(temp_db.rpm_Mot_Push),10),fit_all_rpm(s_all_rpm,temp_x),'--','color',col(i,:))
    legend_lbl{i} = ['Airspeed = ',mat2str(windspeed_bins(i))];
end
legend(hdls,legend_lbl,'location','best')
sgtitle(sprintf('Fx = k1*V*rpm+k_2*rpm^2  |  K1 = %2.2e K2 = %2.2e  |  RMS = %2.2f',s_all_rpm(1),s_all_rpm(2),RMS_all_rpm))
xlabel('RPM [rotation per minute]')
ylabel('Pusher F_x [N]')
axis([0 inf 0 inf])
grid on

% Hence:
%
% Fx pusher =  3.703544293799073e-07 * rpm^2 [rpm] + -6.005961822003111e-05*rpm [rpm] * v [m/s]

%% Plot pprz-->RPM-->T
figure
windspeed_bins = unique(round(pusher_db.Windspeed,0));
pusher_db.Windspeed_bin = round(pusher_db.Windspeed,0);
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = pusher_db(pusher_db.Windspeed_bin==windspeed_bins(i),:);
    temp_x = [linspace(0,max(temp_db.Mot_Push),10)',linspace(0,max(temp_db.Windspeed),10)'];

    if show_error_bar; hdls(i) = errorbar(temp_db.Mot_Push,temp_db.Fx_pusher,temp_db.std_Fx_pusher,'*','color',col(i,:));
    else; hdls(i) = plot(temp_db.Mot_Push,temp_db.Fx_pusher,'*','color',col(i,:)); end
    
    hold on
    plot(linspace(0,max(temp_db.Mot_Push),10),fit_all_rpm(s_all_rpm,[fit_pprz2rpm_quad(s_pprz2rpm,temp_x(:,1)),temp_x(:,2)]),'--','color',col(i,:))
    legend_lbl{i} = ['Airspeed = ',mat2str(windspeed_bins(i))];
end
RMS_pprz2rpm2fx = sqrt(mean((fit_all_rpm(s_all_rpm,[fit_pprz2rpm_quad(s_pprz2rpm,pusher_db.Mot_Push),pusher_db.Windspeed])-pusher_db.Fx_pusher).^2));
legend(hdls,legend_lbl,'location','best')
sgtitle(sprintf(['RPM = K1 * pprz + K2 * pprz^2  |  K1 = %2.2e K2 = %2.2e' ...
    '  |  RMS = %2.2f\nFx = k1*V*rpm+k_2*rpm^2  |  K1 = %2.2e K2 = %2.2e' ...
    '  |  RMS = %2.2f\nOverall RMS = %2.2f'],s_pprz2rpm(1),s_pprz2rpm(2) ...
    ,RMS_pprz2pwm_quad,s_all_rpm(1),s_all_rpm(2),RMS_all_rpm,RMS_pprz2rpm2fx))
xlabel('pprz signal [pprz]')
ylabel('Pusher F_x [N]')
axis([0 inf 0 inf])
grid on

%% Fit on all pprz
% Delete 0 command signals
%pusher_db = pusher_db(pusher_db.Mot_Push~=0,:);

% Fx = k1*V*pprz+k_2*pprz^2
% k  = [k1 k_2]
% x = [pprz,V]
x = [pusher_db.Mot_Push,pusher_db.Windspeed];
y = [pusher_db.Fx_pusher];

%fit_all_pprz = @(k,x)  k(1).*(x(:,1)+k(2)).^2+k(3).*(x(:,1)+k(2)).*x(:,2)+k(4);    % Function to fit
fit_all_pprz = @(k,x)  k(1).*x(:,1).^2+k(2).*x(:,1).*x(:,2);    % Function to fit
fcn_all_pprz = @(k) sqrt(mean((fit_all_pprz(k,x) - y).^2));           % Least-Squares cost function
[s_all_pprz,RMS_all_pprz] = fminsearch(fcn_all_pprz,[1E-5;1E-5])

% s_all_pprz =
% 
%    1.0e-04 *
% 
%    0.004002013847311
%   -0.186387227588874
% 
% 
% RMS_all_pprz =
% 
%    1.036952334246392

figure
windspeed_bins = unique(round(pusher_db.Windspeed,0));
pusher_db.Windspeed_bin = round(pusher_db.Windspeed,0);
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = pusher_db(pusher_db.Windspeed_bin==windspeed_bins(i),:);
    temp_x = [linspace(0,max(temp_db.Mot_Push),10)',linspace(0,max(temp_db.Windspeed),10)'];

    if show_error_bar; hdls(i) = errorbar(temp_db.Mot_Push,temp_db.Fx_pusher,temp_db.std_Fx_pusher,'*','color',col(i,:));
    else; hdls(i) = plot(temp_db.Mot_Push,temp_db.Fx_pusher,'*','color',col(i,:)); end
    
    hold on
    plot(linspace(0,max(temp_db.Mot_Push),10),fit_all_pprz(s_all_pprz,temp_x),'--','color',col(i,:))
    legend_lbl{i} = ['Airspeed = ',mat2str(windspeed_bins(i))];
end
legend(hdls,legend_lbl,'location','best')
sgtitle(sprintf('Fx = k1*V*pprz+k_2*pprz^2  |  K1 = %2.2e K2 = %2.2e  |  RMS = %2.2f',s_all_pprz(1),s_all_pprz(2),RMS_all_pprz))
xlabel('Pusher Command [pprz]')
ylabel('Pusher F_x [N]')
axis([0 inf 0 inf])
grid on

% Hence:
%
% Fx =  4.002013847311274e-07 * pprz^2 [pprz] + -1.863872275888742e-05*pprz [pprz] * v [m/s]
