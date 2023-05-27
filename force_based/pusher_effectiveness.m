%% Init
clear all

options = optimset('TolFun',1E-5,'TolX',1E-5);
show_error_bar = false;

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

test_AE = 'AE';
test_LP1 = 'LP1';
idx_1 = contains(full_db.Code,test_AE);
idx_2 = contains(full_db.Code,test_LP1);

test_db_AE = full_db(idx_1,:);
% Remove all entries with actuators active
test_db_AE = test_db_AE(test_db_AE.Turn_Table==0 &test_db_AE.Rud==0 & test_db_AE.Elev==0 & test_db_AE.Ail_R==0 & test_db_AE.Ail_L==0 & test_db_AE.Mot_F<1000 &test_db_AE.Mot_R<1000 &test_db_AE.Mot_B<1000 & test_db_AE.Mot_L<1000,:);
% Select skew = 0
test_db_AE = test_db_AE(test_db_AE.Skew_sp==0,:);
% Remove 0 command from fit
test_db_AE = test_db_AE(test_db_AE.Mot_Push>0,:);

test_db_LP1 = full_db(idx_2,:);

forces_columns = {'Mx','My','Mz','Fx','Fy','Fz',};

% Substract db2 and db3 from db1
test_db = table();
tol = 1;
for i=1:size(test_db_AE,1)
    idx_LP1 = find(abs(test_db_LP1.Windspeed-test_db_AE.Windspeed(i))<tol & abs(test_db_LP1.Turn_Table-test_db_AE.Turn_Table(i))<tol & abs(test_db_LP1.Skew_sp-test_db_AE.Skew_sp(i))<tol,1,'first');
    if ~isempty(idx_LP1)
        test_db(end+1,:) = test_db_AE(i,:);
        test_db{end,forces_columns} = test_db_AE{i,forces_columns} -test_db_LP1{idx_LP1,forces_columns};
        test_db.Code{end} = [test_AE '-' test_LP1];
    end
end

% Transform all angles in rad
deg_columns = {'Turn_Table','Skew','Skew_sp','Pitch','AoA','std_AoA'};
test_db{:,deg_columns} = deg2rad(test_db{:,deg_columns});

% Save
%save(['db_',test_1,'.mat'],'test_db')

pusher_db = test_db;
pusher_db.Fx_pusher = pusher_db.Fx;
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
    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
end
lgd1 = legend(hdls,legend_lbl,'location','best');
title(lgd1,'Airspeed') % add legend title
xlabel('Pusher Command [pprz]')
ylabel('F_x [N]')
grid on
title({'Fx depending on airspeed and pusher command'})
axis([0 inf -inf inf])

%% Fit on pprz to RPM

x = [pusher_db.Mot_Push];
y = [pusher_db.rpm_Mot_Push];

fit_pprz2rpm_lin = @(k,x)  k(1).*x(:,1);    % Function to fit
fcn_pprz2rpm_lin = @(k) sqrt(mean((fit_pprz2rpm_lin(k,x) - y).^2));           % Least-Squares cost function
[s_pprz2rpm_lin,RMS_pprz2pwm_lin] = fminsearch(fcn_pprz2rpm_lin,[1E0])

% s_pprz2rpm_lin =
% 
%     1.1293
% 
% 
% RMS_pprz2pwm_lin =
% 
%   439.1480

fit_pprz2rpm_quad = @(k,x)  k(1).*x(:,1)+k(2).*x(:,1).^2;    % Function to fit
fcn_pprz2rpm_quad = @(k) sqrt(mean((fit_pprz2rpm_quad(k,x) - y).^2));           % Least-Squares cost function
[s_pprz2rpm_quad,RMS_pprz2pwm_quad] = fminsearch(fcn_pprz2rpm_quad,[1E0;0])

% s_pprz2rpm_quad =
% 
%     1.3787
%    -0.0000
% 
% 
% RMS_pprz2pwm_quad =
% 
%   126.0392

figure
if show_error_bar; hdls(i) = errorbar(pusher_db.Mot_Push,pusher_db.rpm_Mot_Push,pusher_db.rpm_Mot_Push_std,'*');
    else; plot(pusher_db.Mot_Push,pusher_db.rpm_Mot_Push,'*'); end
    
hold on
plot(linspace(0,max(pusher_db.Mot_Push),10),fit_pprz2rpm_quad(s_pprz2rpm_quad,linspace(0,max(pusher_db.Mot_Push),10)'),'--')
plot(linspace(0,max(pusher_db.Mot_Push),10),fit_pprz2rpm_lin(s_pprz2rpm_lin,linspace(0,max(pusher_db.Mot_Push),10)'),'--')
xlabel('pprz signal [pprz]')
ylabel('RPM [rotation per minute]')
legend('Data','Quadratic Fit','Linear Fit','location','best')
grid on
axis([0 inf 0 inf])

% Hence
% RPM = 1.378672356359907 * pprz + -3.789505985611778e-05 * pprz^2
% RMS = 126
%
% or 
%
% RPM = 1.129296875000000 * pprz
% RMS = 380

%% Fit on all RPM

% Fx = k1*V*RPM+k_2*RPM^2+k_3*V
% k  = [k1 k_2]
% x = [pprz,V]
x = [pusher_db.rpm_Mot_Push,pusher_db.Windspeed];
y = [pusher_db.Fx_pusher];

fit_all_rpm = @(k,x)  k(1).*(x(:,1)).^2+k(2)*x(:,1).*x(:,2)+k(3).*x(:,2);
fcn_all_rpm = @(k) sqrt(mean((fit_all_rpm(k,x) - y).^2));           % Least-Squares cost function
[s_all_rpm,RMS_all_rpm] = fminsearch(fcn_all_rpm,[1E-7;-6E-4;-2])

% s_all_rpm =
% 
%    0.000000368865338
%   -0.000045422156524
%   -0.084830356821562
% 
% 
% RMS_all_rpm =
% 
%    0.419639350722040

figure
windspeed_bins = unique(round(pusher_db.Windspeed,0));
pusher_db.Windspeed_bin = round(pusher_db.Windspeed,0);
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = pusher_db(pusher_db.Windspeed_bin==windspeed_bins(i),:);
    temp_x = [linspace(0,max(temp_db.rpm_Mot_Push),15)',ones(15,1).*windspeed_bins(i)];

    if show_error_bar; hdls(i) = errorbar(temp_db.rpm_Mot_Push,temp_db.Fx_pusher,temp_db.std_Fx_pusher,'*','color',col(i,:));
    else; hdls(i) = plot(temp_db.rpm_Mot_Push,temp_db.Fx_pusher,'*','color',col(i,:)); end
    
    hold on
    plot(linspace(0,max(temp_db.rpm_Mot_Push),15),fit_all_rpm(s_all_rpm,temp_x),'--','color',col(i,:))
    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
end
lgd1 = legend(hdls,legend_lbl,'location','best');
title(lgd1,'Airspeed') % add legend title
title(sprintf('Fx = K1*rpm^2+K2*V*rpm+K3*V\nK1 = %2.2e K2 = %2.2e K3 = %2.2e  |  RMS = %2.2f',s_all_rpm(1),s_all_rpm(2),s_all_rpm(3),RMS_all_rpm))
xlabel('RPM [rotation per minute]')
ylabel('Pusher F_x [N]')
axis([0 inf 0 inf])
grid on

%% Plot pprz-->RPM-->T
figure
windspeed_bins = unique(round(pusher_db.Windspeed,0));
pusher_db.Windspeed_bin = round(pusher_db.Windspeed,0);
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = pusher_db(pusher_db.Windspeed_bin==windspeed_bins(i),:);
    temp_x = [linspace(0,max(temp_db.Mot_Push),10)',ones(10,1).*windspeed_bins(i)];

    if show_error_bar; hdls(i) = errorbar(temp_db.Mot_Push,temp_db.Fx_pusher,temp_db.std_Fx_pusher,'*','color',col(i,:));
    else; hdls(i) = plot(temp_db.Mot_Push,temp_db.Fx_pusher,'*','color',col(i,:)); end
    
    hold on
    plot(linspace(0,max(temp_db.Mot_Push),10),fit_all_rpm(s_all_rpm,[fit_pprz2rpm_quad(s_pprz2rpm_quad,temp_x(:,1)),temp_x(:,2)]),'--','color',col(i,:))
    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
end
RMS_pprz2rpm2fx = sqrt(mean((fit_all_rpm(s_all_rpm,[fit_pprz2rpm_quad(s_pprz2rpm_quad,pusher_db.Mot_Push),pusher_db.Windspeed])-pusher_db.Fx_pusher).^2));
lgd1 = legend(hdls,legend_lbl,'location','best');
title(lgd1,'Airspeed') % add legend title
title(sprintf(['RPM = K1 * pprz + K2 * pprz^2  |  K1 = %2.2e K2 = %2.2e' ...
    '  |  RMS = %2.2f\nFx = K1*rpm^2+K2*V*rpm+K3*V  |  K1 = %2.2e K2 = %2.2e K3 = %2.2e' ...
    '  |  RMS = %2.2f\nOverall RMS = %2.2f'],s_pprz2rpm_quad(1),s_pprz2rpm_quad(2) ...
    ,RMS_pprz2pwm_quad,s_all_rpm(1),s_all_rpm(2),s_all_rpm(3),RMS_all_rpm,RMS_pprz2rpm2fx))
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

fit_all_pprz = @(k,x)  k(1).*x(:,1).^2+k(2).*x(:,1).*x(:,2);    % Function to fit
fcn_all_pprz = @(k) sqrt(mean((fit_all_pprz(k,x) - y).^2));           % Least-Squares cost function
[s_all_pprz,RMS_all_pprz] = fminsearch(fcn_all_pprz,[1E-5;1E-5])


figure
windspeed_bins = unique(round(pusher_db.Windspeed,0));
pusher_db.Windspeed_bin = round(pusher_db.Windspeed,0);
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = pusher_db(pusher_db.Windspeed_bin==windspeed_bins(i),:);
    temp_x = [linspace(0,max(temp_db.Mot_Push),10)',ones(10,1).*windspeed_bins];

    if show_error_bar; hdls(i) = errorbar(temp_db.Mot_Push,temp_db.Fx_pusher,temp_db.std_Fx_pusher,'*','color',col(i,:));
    else; hdls(i) = plot(temp_db.Mot_Push,temp_db.Fx_pusher,'*','color',col(i,:)); end
    
    hold on
    plot(linspace(0,max(temp_db.Mot_Push),10),fit_all_pprz(s_all_pprz,temp_x),'--','color',col(i,:))
    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
end
lgd1 = legend(hdls,legend_lbl,'location','best');
title(lgd1,'Airspeed') % add legend title
title(sprintf('Fx = K1*pprz^2+K2*V*pprz  |  K1 = %2.2e K2 = %2.2e  |  RMS = %2.2f',s_all_pprz(1),s_all_pprz(2),RMS_all_pprz))
xlabel('Pusher Command [pprz]')
ylabel('Pusher F_x [N]')
axis([0 inf 0 inf])
grid on

%% Nice plot
set(gcf, 'Renderer', 'Painters');

line_width = 2;
font_size = 20;
marker_size = 15;
AR = 1.5;
fig_height = 750;
fig_width = fig_height*AR;

screen = get(0, 'ScreenSize');

if fig_width>screen(3)
    fig_width = screen(3);
    fig_height = fig_width/AR;
end
fprintf('Exporting as %.0fx%.0f \n',fig_width,fig_height);

% Get the current date and time
nowDateTime = datetime('now');

% Format the date and time in the "MM_DD_HH_MM" format
formattedDateTime = datestr(nowDateTime,'mm_dd_HH_MM');

fig = figure('position',[0 0 fig_width fig_height]);

% Store the default line width value
origLineWidth = get(groot, 'DefaultLineLineWidth');

% Set a new default line width value
set(groot, 'DefaultLineLineWidth', line_width);

% Set colors and line styles
mycolors = linspecer(3,'qualitative');
mylinestyles = {'-', '--', ':'};
mymarkerstyles = {'o','+','*','x','square','diamond','^'};
set(gcf,'DefaultAxesColorOrder',mycolors, ...
        'DefaultAxesLineStyleOrder',mylinestyles)

windspeed_bins = unique(round(pusher_db.Windspeed,0));
pusher_db.Windspeed_bin = round(pusher_db.Windspeed,0);
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = pusher_db(pusher_db.Windspeed_bin==windspeed_bins(i),:);
    temp_x = [linspace(0,max(temp_db.rpm_Mot_Push),15)',ones(15,1).*windspeed_bins(i)];

    hdls(i) = plot(temp_db.rpm_Mot_Push,temp_db.Fx_pusher,mymarkerstyles{i},'color',col(i,:),'MarkerSize',marker_size);
    
    hold on
    plot(linspace(0,max(temp_db.rpm_Mot_Push),15),fit_all_rpm(s_all_rpm,temp_x),'-','color',col(i,:))
    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
end
lgd1 = legend(hdls,legend_lbl,'Location', 'northoutside', 'Orientation', 'horizontal');
xlabel('RPM [rev per minute]')
ylabel('Pusher F_x [N]')
axis([0 9500 0 30])
grid on

% Change font size
set(findall(gcf,'-property','FontSize'),'FontSize',font_size) 

fig_name = ['PUSHER_FX_',formattedDateTime,'.eps'];
exportgraphics(fig,fig_name,'BackgroundColor','none','ContentType','vector')
grid on
