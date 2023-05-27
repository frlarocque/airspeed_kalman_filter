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
% ES: Elevator stall (wing attached)
% LP: Lift test (pitch changes)


% LP1: a/c w/o pusher
% LP2: a/c w/o pusher w/o elevator
% LP3: a/c w/o pusher w/o wing
% LP4: a/c w/o pusher w/o wing w/o hover props
% LP5: a/c w/o pusher w/o elevator w/o hover props
% LP6: a/c w/o pusher w/o hover props

test_codes = unique(full_db.Code);

test_1 = 'LP3';
test_2 = 'LP4';
idx_1 = contains(full_db.Code,test_1);
idx_2 = contains(full_db.Code,test_2);

test_db_1 = full_db(idx_1,:);
test_db_2 = full_db(idx_2,:);

forces_columns = {'Mx','My','Mz','Fx','Fy','Fz',};

% Substract db2 from db1
test_db = table();

for i=1:size(test_db_1,1)
    id_1_mat = strsplit(test_db_1.ID{i}, '_');
    id_1_mat = strjoin(id_1_mat([2:4,6:7]),'_');

    for j=1:size(test_db_2,1)
        id_2_mat = strsplit(test_db_2.ID{j}, '_');
        id_2_mat = strjoin(id_2_mat([2:4,6:7]),'_');
        
        if strcmp(id_1_mat,id_2_mat)
            test_db(end+1,:) = test_db_1(i,:);
            test_db{end,forces_columns} = test_db_1{i,forces_columns} -test_db_2{j,forces_columns};
            test_db.Code{end} = [test_1 '-' test_2];
            break
        end
    end
end

% Transform all angles in rad
deg_columns = {'Turn_Table','Skew','Skew_sp','Pitch','AoA','std_AoA'};
test_db{:,deg_columns} = deg2rad(test_db{:,deg_columns});

%% Remove entries
% Removing entries with non-zero control surfaces
test_db = test_db(test_db.Rud==0 & test_db.Elev==0 & test_db.Ail_R==0 & test_db.Ail_L==0 ,:);
% Removing entries with non-zero pusher motor
test_db = test_db(test_db.Mot_Push==0,:);
% Removing entries with angle of attack higher than 15 deg
test_db = test_db(abs(test_db.Turn_Table)<deg2rad(15),:); 
% Select skew = 0 deg
test_db = test_db(test_db.Skew_sp==deg2rad(0),:);

%% Replace NAN entry in RPM by estimation
RPM_columns = find(strcmp(test_db.Properties.VariableNames, 'rpm_Mot_F') |...
    strcmp(test_db.Properties.VariableNames, 'rpm_Mot_L') |...
    strcmp(test_db.Properties.VariableNames, 'rpm_Mot_R') |...
    strcmp(test_db.Properties.VariableNames, 'rpm_Mot_B'), 4);

correction = 0;
for i=1:size(test_db,1)
    for j=1:length(RPM_columns)
        if isnan(test_db{i,RPM_columns(j)})
            test_db{i,RPM_columns(j)} = mean(test_db{i,RPM_columns(RPM_columns~=RPM_columns(j)& ~isnan(test_db{i,RPM_columns}))});            
            test_db{i,RPM_columns(j)} = 100.*round(test_db{i,RPM_columns(j)}./100);
            correction=correction+1;
        end
    end
end
fprintf(sprintf('Corrected RPM %d times\n',correction))

%% Obtaining hover prop lift
% Fz = body lift + Hover_motors lift
% Hover_motors lift = Fz-body lift
%Drag without hover props (only body drag)

test_db.Fz_hover = test_db.Fz;

% Calculating std dev of new Fz
% Mean C = mean A - mean B --> Variance C = Variance A + Variance B - 2*Correlation(A,B)*SD A * SD B
% Assumming uncorrelated
test_db.std_Fz_hover = sqrt(test_db.std_Fz.^2+test_db.std_Fz.^2);

save('db_Fz_hover_prop.mat','test_db')
%% Plot

% Hover prop lift function:
% RPM, crossflow (V*cos(alpha))

windspeed_bins = unique(round(test_db.Windspeed,0));
test_db.Windspeed_bin = round(test_db.Windspeed,0);

figure
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = test_db(test_db.Windspeed_bin==windspeed_bins(i),:);

    hdls(i) = plot(temp_db.rpm_Mot_R,temp_db.Fz_hover,'*','color',col(i,:));
    hold on

    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
end

xlabel('Hover Prop R RPM [RPM]')
ylabel('Hover Prop F_z [N]')
lgd1 = legend(hdls,legend_lbl,'location','southwest');
title(lgd1,'Airspeed') % add legend title
sgtitle('Hover Prop Fz Data from Wind Tunnel tests')

%% Fit hover prop
% Select entries with same hover motor commands
all_motors_db = test_db(test_db.Mot_F==test_db.Mot_R & test_db.Mot_R==test_db.Mot_B & test_db.Mot_R==test_db.Mot_B & test_db.Mot_B==test_db.Mot_L,:);

% Fz = K1*RPM^2

x = [all_motors_db.rpm_Mot_R,all_motors_db.Windspeed];
y = [all_motors_db.Fz_hover];

fit_same = @(k,x)  k(1)*x(:,1).^(2); % Function to fit
fcn_same = @(k) sqrt(mean((fit_same(k,x) - y).^2));           % Least-Squares cost function
[s_same,RMS_same] = fminsearch(fcn_same,[-1E-6],options) %bound first coefficient to negative value

% s_same =
% 
%     -3.650781249999991e-06
% 
% 
% RMS_same =
% 
%    2.634024249118863

windspeed_bins = unique(round(all_motors_db.Windspeed,0));
all_motors_db.Windspeed_bin = round(all_motors_db.Windspeed,0);
figure
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = all_motors_db(all_motors_db.Windspeed_bin==windspeed_bins(i),:);
    temp_x = [linspace(0,max(temp_db.rpm_Mot_R),10)',ones(10,1).*windspeed_bins(i)];
    if show_error_bar; hdls(i) = errorbar(temp_db.rpm_Mot_R,temp_db.Fz_hover,temp_db.std_Fz_hover,'*','color',col(i,:));
    else; hdls(i) = plot(temp_db.rpm_Mot_R,temp_db.Fz_hover,'*','color',col(i,:)); end
    
    hold on
    plot(linspace(0,max(temp_db.rpm_Mot_R),10),fit_same(s_same,temp_x),'--','color',col(i,:))
    
    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
end

xlabel('Hover Prop RPM [RPM]')
ylabel('Hover Prop F_z [N]')
lgd1 = legend(hdls,legend_lbl,'location','southeast');
title(lgd1,'Airspeed') % add legend title
title(sprintf('4 motors at the same time\nFz hover props = K1*RPM^{2}\nK1 = %2.2e |  RMS = %2.2f',s_same(1),RMS_same))
axis([0 inf -inf 0])
grid on

%% Analyze result of fit
RMS = sqrt(mean((fit_same(s_same,x) - y).^2))
range = max(y)-min(y)
RMS_percentage_range = RMS./range
max_error = max(abs((fit_same(s_same,x) - y)))
max_error_percentage_range = max_error./range

%% Fit all motor separately
% Fz = K1*RPM_F^2+K2*RPM_R^2+K3*RPM_B^2+K4*RPM_L^2

x = [test_db.rpm_Mot_F,test_db.rpm_Mot_R,test_db.rpm_Mot_B,test_db.rpm_Mot_L,test_db.Windspeed];
y = [test_db.Fz_hover];

fit_all = @(k,x)  k(1)*x(:,1).^(2)+k(2)*x(:,2).^(2)+k(3)*x(:,3).^(2)+k(4)*x(:,4).^(2); % Function to fit
fcn_all = @(k) sqrt(mean((fit_all(k,x) - y).^2));           % Least-Squares cost function
[s_all,RMS_all] = fminsearch(fcn_all,[-1E-6;-1E-6;-1E-6;-1E-6],options) %bound first coefficient to negative value

% s_all =
% 
%    1.0e-06 *
% 
%   -0.873870581108021
%   -0.951740938617989
%   -0.894621788336263
%   -0.852055641614473
% 
% 
% RMS_all =
% 
%    2.117587085260848

%% Nice plot
nice_db = test_db;

rpm_bins = unique(200.*round(nice_db.rpm_Mot_F./200,0));
nice_db.rpm_bin = 200.*round(nice_db.rpm_Mot_F./200,0);
windspeed_bins = unique(round(nice_db.Windspeed,0));
nice_db.windspeed_bin = round(nice_db.Windspeed,0);

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


legend_lbl = {};
col=linspecer(length(windspeed_bins)+1);
hdls = [];
Ax(1) = axes(fig); 

for i=1:length(windspeed_bins)
    temp_db = nice_db(nice_db.windspeed_bin==windspeed_bins(i),:);
    

    temp_group = groupsummary(temp_db, ['rpm_bin'], 'mean', 'Fz_hover');
    
    hdls(i,1) = plot(temp_group.rpm_bin,temp_group.mean_Fz_hover,mymarkerstyles{i},'color',col(i,:),'MarkerSize',marker_size);
    hold on
    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];

end
temp_x = [linspace(0,max(rpm_bins),20)',ones(20,1).*0];
plot(linspace(0,max(rpm_bins),20),fit_same(s_same,temp_x),'-','color',col(i+1,:))
    
set(Ax(1), 'Box','off')
lgd1 = legend(hdls(:,1),legend_lbl,'Location', 'northoutside', 'Orientation', 'horizontal');

xlabel('RPM [Rev per minute]')
ylabel('Hover Motor F_z [N]')
axis([0 inf -inf 0])
grid on

% Change font size
set(findall(gcf,'-property','FontSize'),'FontSize',font_size) 

fig_name = ['HOVER_FZ_',formattedDateTime,'.eps'];
exportgraphics(fig,fig_name,'BackgroundColor','none','ContentType','vector')
grid on
