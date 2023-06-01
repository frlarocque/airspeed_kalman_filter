clear all

options = optimset('TolFun',1E-5,'TolX',1E-5);
show_error_bar = false;
format long

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

test_1 = 'LP1';
test_2 = 'LP3';
test_3 = 'LP5';
idx_1 = contains(full_db.Code,test_1);
idx_2 = contains(full_db.Code,test_2);
idx_3 = contains(full_db.Code,test_3);

test_db_1 = full_db(idx_1,:);
test_db_2 = full_db(idx_2,:);
test_db_3 = full_db(idx_3,:);

forces_columns = {'Mx','My','Mz','Fx','Fy','Fz',};

% Substract db2 and db3 from db1
test_db = table();
for i=1:size(test_db_1,1)
    id = test_db_1.ID{i}(length(test_1)+2:end);
    if any(contains(test_db_2.ID,id)) && any(contains(test_db_3.ID,id))
        test_db(end+1,:) = test_db_1(i,:);
        test_db{end,forces_columns} = -(test_db_1{i,forces_columns} -test_db_2{contains(test_db_2.ID,id),forces_columns} -test_db_3{contains(test_db_3.ID,id),forces_columns});
        test_db.Code{end} = ['-(' test_1 '-' test_2 '-' test_3 ')'];
    end
end

% Transform all angles in rad
deg_columns = {'Turn_Table','Skew','Skew_sp','Pitch','AoA','std_AoA'};
test_db{:,deg_columns} = deg2rad(test_db{:,deg_columns});

% Save
%save(['db_',test_1,'.mat'],'test_db')

%% Remove entries

% Removing entries with non-zero control surfaces
test_db = test_db(test_db.Rud==0 & test_db.Elev==0 & test_db.Ail_R==0 & test_db.Ail_L==0 ,:);
% Removing entries with non-zero pusher motor
test_db = test_db(test_db.Mot_Push==0,:);
% Removing entries with non-zero hover motor command
test_db = test_db(test_db.Mot_F==0 & test_db.Mot_R==0 & test_db.Mot_B==0 & test_db.Mot_L==0,:);
% Removing entries with angle of attack higher than 15 deg
test_db(abs(test_db.Turn_Table)>deg2rad(15),:) = []; 

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

save('db_Fz_fuselage.mat','test_db')
%% Plot Fz
windspeed_bins = unique(round(test_db.Windspeed,0));
test_db.Windspeed_bin = round(test_db.Windspeed,0);

figure
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = test_db(test_db.Windspeed_bin==windspeed_bins(i),:);

    subplot(2,2,1)
    hdls(1,i) = plot(rad2deg(temp_db.Turn_Table),temp_db.Fz,'*','color',col(i,:));
    hold on

    subplot(2,2,2)
    hdls(2,i) = plot(rad2deg(temp_db.Skew_sp),temp_db.Fz,'*','color',col(i,:));
    hold on

    subplot(2,2,3)
    hdls(3,i) = plot(rad2deg(temp_db.Turn_Table),temp_db.Fz./temp_db.Windspeed.^2,'*','color',col(i,:));
    hold on

    subplot(2,2,4)
    hdls(4,i) = plot(rad2deg(temp_db.Skew_sp),temp_db.Fz./temp_db.Windspeed.^2,'*','color',col(i,:));
    hold on

    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
end

subplot(2,2,1)
xlabel('Turn table angle (angle of attack) [deg]')
ylabel('F_z [N]')
%lgd1 = legend(hdls(1,:),legend_lbl,'location','southeast');
%title(lgd1,'Airspeed') % add legend title
subplot(2,2,2)
xlabel('Skew Setpoint [deg]')
ylabel('F_z [N]')
%lgd1 = legend(hdls(2,:),legend_lbl,'location','southeast');
%title(lgd1,'Airspeed') % add legend title
subplot(2,2,3)
xlabel('Turn table angle (angle of attack) [deg]')
ylabel('F_z/Airspeed^2 [N/((m/s)^2)]')
%lgd1 = legend(hdls(3,:),legend_lbl,'location','southeast');
%title(lgd1,'Airspeed') % add legend title
grid on
subplot(2,2,4)
xlabel('Skew Setpoint [deg]')
ylabel('F_z/Airspeed^2 [N/((m/s)^2)]')
lgd1 = legend(hdls(4,:),legend_lbl,'location','southeast');
title(lgd1,'Airspeed') % add legend title
sgtitle('Fz Drag Data from Wind Tunnel tests')

%% Plot Lift
windspeed_bins = unique(round(test_db.Windspeed,0));
test_db.Windspeed_bin = round(test_db.Windspeed,0);

figure
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = test_db(test_db.Windspeed_bin==windspeed_bins(i),:);

    subplot(2,2,1)
    hdls(1,i) = plot(rad2deg(temp_db.Turn_Table),temp_db.Lift,'*','color',col(i,:));
    hold on

    subplot(2,2,2)
    hdls(2,i) = plot(rad2deg(temp_db.Skew_sp),temp_db.Lift,'*','color',col(i,:));
    hold on

    subplot(2,2,3)
    hdls(3,i) = plot(rad2deg(temp_db.Turn_Table),temp_db.Lift./temp_db.Windspeed.^2,'*','color',col(i,:));
    hold on

    subplot(2,2,4)
    hdls(4,i) = plot(rad2deg(temp_db.Skew_sp),temp_db.Lift./temp_db.Windspeed.^2,'*','color',col(i,:));
    hold on

    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
end

subplot(2,2,1)
xlabel('Turn table angle (angle of attack) [deg]')
ylabel('Lift [N]')
%lgd1 = legend(hdls(1,:),legend_lbl,'location','southeast');
%title(lgd1,'Airspeed') % add legend title
subplot(2,2,2)
xlabel('Skew Setpoint [deg]')
ylabel('Lift [N]')
%lgd1 = legend(hdls(2,:),legend_lbl,'location','southeast');
%title(lgd1,'Airspeed') % add legend title
subplot(2,2,3)
xlabel('Turn table angle (angle of attack) [deg]')
ylabel('Lift/Airspeed^2 [N/((m/s)^2)]')
%lgd1 = legend(hdls(3,:),legend_lbl,'location','southeast');
%title(lgd1,'Airspeed') % add legend title
grid on
subplot(2,2,4)
xlabel('Skew Setpoint [deg]')
ylabel('Lift/Airspeed^2 [N/((m/s)^2)]')
lgd1 = legend(hdls(4,:),legend_lbl,'location','southeast');
title(lgd1,'Airspeed') % add legend title
sgtitle('Initial Data from Wind Tunnel tests')

%% Find Fz Lift of vehicle at 0 skew (quad mode), all airspeeds
quad_db = test_db(test_db.Skew_sp==deg2rad(0),:);

% Fz = (k1+k2*alpha+k3*alpha^2)*V^2
% k  = [k1 k2 k3 k4]
% x = [AoA,V]
x = [quad_db.Turn_Table,quad_db.Windspeed];
y = [quad_db.Fz];

fit_quad = @(k,x)  (k(1)+k(2).*x(:,1)+k(3).*x(:,1).^2).*x(:,2).^2; % Function to fit
fcn_quad = @(k) sqrt(mean((fit_quad(k,x) - y).^2));           % Least-Squares cost function
[s_quad,RMS_quad,~,~] = fminsearch(fcn_quad,[0;0;0],options) %bound first coefficient to negative value

figure
windspeed_bins = unique(round(quad_db.Windspeed,0));
quad_db.Windspeed_bin = round(quad_db.Windspeed,0);
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = quad_db(quad_db.Windspeed_bin==windspeed_bins(i),:);
    temp_x = [linspace(min(temp_db.Turn_Table),max(temp_db.Turn_Table),10)',ones(10,1).*windspeed_bins(i)];

    if show_error_bar; hdls(i) = errorbar(rad2deg(temp_db.Turn_Table),temp_db.Fz,temp_db.std_Fz,'*','color',col(i,:));
    else; hdls(i) = plot(rad2deg(temp_db.Turn_Table),temp_db.Fz,'*','color',col(i,:)); end
    
    hold on
    plot(rad2deg(linspace(min(temp_db.Turn_Table),max(temp_db.Turn_Table),10)),fit_quad(s_quad,temp_x),'--','color',col(i,:))
    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
end
lgd1 = legend(hdls,legend_lbl,'location','southwest');
title(lgd1,'Airspeed') % add legend title
title(sprintf('All Airspeed Fit\nFz = (k1+k2*alpha+k3*alpha^2)*V^2\nK1 = %2.2e K2 = %2.2e K3 = %2.2e |  RMS = %2.2f',s_quad(1),s_quad(2),s_quad(3),RMS_quad))
xlabel('Turn table angle (angle of attack) [deg]')
ylabel('Body Lift F_z [N]')
axis([-inf inf -inf inf])
grid on

%% Fz Lift for all airspeeds, for all skews
skew_db = test_db;

% Fz= (k1*cos(skew)+k2+k3*alpha+k4*alpha^2)*V^2
% k  = [k1 k2 k3 k4]
% x = [AoA,V,Skew]
x = [skew_db.Turn_Table,skew_db.Windspeed,skew_db.Skew_sp];
y = [skew_db.Fz];

fit_skew = @(k,x)  (k(1)*cos(x(:,3))+k(2)+k(3).*x(:,1)+k(4).*x(:,1).^2).*x(:,2).^2; % Function to fit
fcn_skew = @(k) sqrt(mean((fit_skew(k,x) - y).^2));           % Least-Squares cost function
[s_skew,RMS_skew] = fminsearch(fcn_skew,[0;0;0;0],options) %bound first coefficient to negative value

% s_skew =
% 
%   -0.004010655576495
%   -0.000721901291023
%   -0.112383214091401
%    0.077819486465484
% 
% 
% RMS_skew =
% 
%    0.918013367932760


windspeed_bins = unique(round(skew_db.Windspeed,0));
skew_db.Windspeed_bin = round(skew_db.Windspeed,0);

skew_bins = deg2rad(unique(round(rad2deg(skew_db.Skew_sp),0)));
skew_db.Skew_sp_bin = deg2rad(round(rad2deg(skew_db.Skew_sp),0));
skew_bins = skew_bins(1:2:length(skew_bins)); % Select only 4

figure
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];

for j=1:length(skew_bins)

    subplot(2,2,j)

    for i=1:length(windspeed_bins)
        temp_db = skew_db(skew_db.Windspeed_bin==windspeed_bins(i) & skew_db.Skew_sp_bin==skew_bins(j),:);
        temp_x = [linspace(min(temp_db.Turn_Table),max(temp_db.Turn_Table),10)',ones(10,1).*windspeed_bins(i),ones(10,1).*skew_bins(j)];
    
        if show_error_bar; hdls(i) = errorbar(rad2deg(temp_db.Turn_Table),temp_db.Fz,temp_db.std_Fz,'*','color',col(i,:));
        else; hdls(i) = plot(rad2deg(temp_db.Turn_Table),temp_db.Fz,'*','color',col(i,:)); end
        
        hold on
        plot(rad2deg(linspace(min(temp_db.Turn_Table),max(temp_db.Turn_Table),10)),fit_skew(s_skew,temp_x),'--','color',col(i,:))
        legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
    end
    
    xlabel('Turn table angle (angle of attack) [deg]')
    ylabel('Body Lift F_z [N]')
    axis([-inf inf -inf inf])
    grid on
    title(sprintf('Skew Angle %2.0f deg',rad2deg(skew_bins(j))))
end
lgd1 = legend(hdls,legend_lbl,'location','northwest');
title(lgd1,'Airspeed') % add legend title
sgtitle(sprintf('Fit for all airspeed, AoA and skew\nFz = (K1*cos(skew)+K2+K3*alpha+K4*alpha^2)*V^2\nK1 = %2.2e K2 = %2.2e K3 = %2.2e K4 = %2.2e|  RMS = %2.2f',s_skew(1),s_skew(2),s_skew(3),s_skew(4),RMS_skew))

%% Analyze result of fit
RMS = sqrt(mean((fit_skew(s_skew,x) - y).^2))
range = max(y)-min(y)
RMS_percentage_range = RMS./range
max_error = max(abs((fit_skew(s_skew,x) - y)))
max_error_percentage_range = max_error./range

%% Plot nice
nice_db = test_db(test_db.Skew_sp==0,:);

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

windspeed_bins = unique(round(nice_db.Windspeed,0));
nice_db.Windspeed_bin = round(nice_db.Windspeed,0);

legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
Ax(1) = axes(fig);
error_vec = [];
for i=1:length(windspeed_bins)
    temp_db = nice_db(nice_db.Windspeed_bin==windspeed_bins(i),:);
    temp_group = groupsummary(temp_db, ['Turn_Table'], 'mean', 'Fz');

    temp_x = [linspace(min(temp_db.Turn_Table),max(temp_db.Turn_Table),10)',ones(10,1).*windspeed_bins(i),ones(10,1).*0];
    hdls(i,1) = plot(rad2deg(temp_group.Turn_Table),temp_group.mean_Fz,mymarkerstyles{i},'color',col(i,:),'MarkerSize',marker_size);
    hold on
    %hdls(i,2) = plot(rad2deg(linspace(min(temp_db.Turn_Table),max(temp_db.Turn_Table),10)),fit_nice(s_nice,temp_x),'-','color',col(i,:));
    
    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
    error_vec = [error_vec; temp_group.mean_Fz-0];
end
lgd1 = legend(hdls(:,1),legend_lbl,'Location', 'northoutside', 'Orientation', 'horizontal');
%title(lgd1,'Airspeed') % add legend title


xlabel('Angle of attack [deg]')
ylabel('Fuselage F_z [N]')
%axis([min(rad2deg(nice_db.Turn_Table))-1 max(rad2deg(nice_db.Turn_Table))+1 1.1*min(nice_db.Fx) 1.1*max(nice_db.Fx)])
grid on

% Change font size
set(findall(gcf,'-property','FontSize'),'FontSize',font_size) 

fig_name = ['FUSELAGE_FZ_',formattedDateTime,'.eps'];
exportgraphics(fig,fig_name,'BackgroundColor','none','ContentType','vector')

% RMS
fprintf('Overall RMS %2.2f\n',rms(error_vec))