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
test_db(abs(test_db.Turn_Table)>deg2rad(15),:) = []; 

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

%% Obtaining hover prop drag

test_db.Fx_hover = test_db.Fx;

% Calculating std dev of new Fx_pusher
% Mean C = mean A - mean B --> Variance C = Variance A + Variance B - 2*Correlation(A,B)*SD A * SD B
% Assumming uncorrelated
test_db.std_Fx_hover = sqrt(test_db.std_Fx.^2+test_db.std_Fx.^2);

save('db_Fx_hover_prop.mat','test_db')

%% Fit Hover Prop Fx Drag depending on airspeed and RPM
%Different airspeed and different hover prop values
hover_prop_db = test_db(test_db.Turn_Table==deg2rad(0) & test_db.Skew_sp==deg2rad(0),:);
%hover_prop_db = test_db(test_db.Turn_Table==deg2rad(0),:);


x = [hover_prop_db.rpm_Mot_R,hover_prop_db.Windspeed];
y = [hover_prop_db.Fx_hover];

fit_hover = @(k,x)  k(1)*x(:,2).^(2)+k(2).*sqrt(x(:,2)).*x(:,1).^2; % Function to fit
fcn_hover = @(k) sqrt(mean((fit_hover(k,x) - y).^2));           % Least-Squares cost function
[s_hover,RMS_hover] = fminsearch(fcn_hover,[-6E-3;-1.18E-7],options) %bound first coefficient to negative value

windspeed_bins = unique(round(hover_prop_db.Windspeed,0));
hover_prop_db.Windspeed_bin = round(hover_prop_db.Windspeed,0);
figure
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = hover_prop_db(hover_prop_db.Windspeed_bin==windspeed_bins(i),:);
    temp_x = [linspace(0,max(temp_db.rpm_Mot_R),10)',ones(10,1).*windspeed_bins(i)];
    if show_error_bar; hdls(i) = errorbar(temp_db.rpm_Mot_R,temp_db.Fx_hover,temp_db.std_Fx_hover,'*','color',col(i,:));
    else; hdls(i) = plot(temp_db.rpm_Mot_R,temp_db.Fx_hover,'*','color',col(i,:)); end
    
    hold on
    plot(linspace(0,max(temp_db.rpm_Mot_R),10),fit_hover(s_hover,temp_x),'--','color',col(i,:))
    
    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
end

xlabel('Hover Prop RPM [RPM]')
ylabel('Hover Prop F_x [N]')
lgd1 = legend(hdls,legend_lbl,'location','southwest');
title(lgd1,'Airspeed') % add legend title
title(sprintf('AoA = 0 deg\nFx hover props = K1*V^{2}+K2*V^{1/2}*RPM^{2}\nK1 = %2.2e K2 = %2.2e |  RMS = %2.2f',s_hover(1),s_hover(2),RMS_hover))
axis([0 inf -inf 0])
grid on


% Fx hover props = -3.614335398771809E-3*V^2+-1.180220188976032E-7*V^{1/2}*RPM^2
% RMS = 0.24 (AoA=0)
% RMS = 0.4 (all AoA)

%% Verify on all AoA
hover_prop_db = test_db(test_db.Skew_sp==deg2rad(0),:);

windspeed_bins = unique(round(hover_prop_db.Windspeed,0));
hover_prop_db.Windspeed_bin = round(hover_prop_db.Windspeed,0);
figure
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = hover_prop_db(hover_prop_db.Windspeed_bin==windspeed_bins(i),:);
    temp_x = [linspace(0,max(temp_db.rpm_Mot_R),10)',ones(10,1).*windspeed_bins(i)];
    if show_error_bar; hdls(i) = errorbar(temp_db.rpm_Mot_R,temp_db.Fx_hover,temp_db.std_Fx_hover,'*','color',col(i,:));
    else; hdls(i) = plot(temp_db.rpm_Mot_R,temp_db.Fx_hover,'*','color',col(i,:)); end
    
    hold on
    plot(linspace(0,max(temp_db.rpm_Mot_R),10),fit_hover(s_hover,temp_x),'--','color',col(i,:))
    
    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
end
RMS = sqrt(mean((fit_hover(s_hover,[hover_prop_db.rpm_Mot_R,hover_prop_db.Windspeed]) - hover_prop_db.Fx_hover).^2));

xlabel('Hover Prop RPM [RPM]')
ylabel('Hover Prop F_x [N]')
lgd1 = legend(hdls,legend_lbl,'location','southeast');
title(lgd1,'Airspeed') % add legend title
title(sprintf('All AOA\nFx hover props = K1*V^{2}+K2*V^{1/2}*RPM^{2}\nK1 = %2.2e K2 = %2.2e |  RMS = %2.2f',s_hover(1),s_hover(2),RMS))
axis([0 inf -inf 0])
grid on

%% Analyze result of fit
RMS = sqrt(mean((fit_hover(s_hover,x) - y).^2))
range = max(y)-min(y)
RMS_percentage_range = RMS./range
max_error = max(abs((fit_hover(s_hover,x) - y)))
max_error_percentage_range = max_error./range

%% Fit hover prop Fx Drag depending on V^2 and RPM alone
%Different airspeed and different hover prop values
hover_prop_db = test_db(test_db.Turn_Table==deg2rad(0) & test_db.Skew_sp==deg2rad(0),:);

x = [hover_prop_db.rpm_Mot_R,hover_prop_db.Windspeed];
y = [hover_prop_db.Fx_hover];

% Fx = K1*V^2+K2*RPM^2
fit_hover_no_V12 = @(k,x)  k(1)*x(:,2).^(2)+k(2).*x(:,1).^2; % Function to fit
fcn_hover_no_V12 = @(k) sqrt(mean((fit_hover_no_V12(k,x) - y).^2));           % Least-Squares cost function
[s_hover_no_V12,RMS_hover_no_V12] = fminsearch(fcn_hover_no_V12,[-6E-3;-1.18E-7],options) %bound first coefficient to negative value

windspeed_bins = unique(round(hover_prop_db.Windspeed,0));
hover_prop_db.Windspeed_bin = round(hover_prop_db.Windspeed,0);
figure
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = hover_prop_db(hover_prop_db.Windspeed_bin==windspeed_bins(i),:);
    temp_x = [linspace(0,max(temp_db.rpm_Mot_R),10)',ones(10,1).*windspeed_bins(i)];
    if show_error_bar; hdls(i) = errorbar(temp_db.rpm_Mot_R,temp_db.Fx_hover,temp_db.std_Fx_hover,'*','color',col(i,:));
    else; hdls(i) = plot(temp_db.rpm_Mot_R,temp_db.Fx_hover,'*','color',col(i,:)); end
    
    hold on
    plot(linspace(0,max(temp_db.rpm_Mot_R),10),fit_hover_no_V12(s_hover_no_V12,temp_x),'--','color',col(i,:))
    
    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
end

xlabel('Hover Prop RPM [RPM]')
ylabel('Hover Prop F_x [N]')
lgd1 = legend(hdls,legend_lbl,'location','southeast');
title(lgd1,'Airspeed') % add legend title
title(sprintf('AoA = 0 deg\nFx hover props = K1*V^{2}+K2*RPM^{2}\nK1 = %2.2e K2 = %2.2e |  RMS = %2.2f',s_hover_no_V12(1),s_hover_no_V12(2),RMS_hover_no_V12))
axis([0 inf -inf 0])
grid on

% Check on all aoa
hover_prop_db = test_db(test_db.Skew_sp==deg2rad(0),:);
RMS = sqrt(mean((fit_hover_no_V12(s_hover_no_V12,[hover_prop_db.rpm_Mot_R,hover_prop_db.Windspeed]) - hover_prop_db.Fx_hover).^2))


% Fx hover props = -1.188293370185022E-2*V^2+- -3.133468238831906E-7*RPM^2
% RMS = 0.57 (AoA=0)
% RMS = 0.67 (all AoA)
%% Compare with and without V^(1/2)
hover_prop_db = test_db(test_db.Turn_Table==deg2rad(0) & test_db.Skew_sp==deg2rad(0),:);

windspeed_bins = unique(round(hover_prop_db.Windspeed,0));
hover_prop_db.Windspeed_bin = round(hover_prop_db.Windspeed,0);

f1 = figure();
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
Ax(1) = axes(f1); 

for i=1:length(windspeed_bins)
    temp_db = hover_prop_db(hover_prop_db.Windspeed_bin==windspeed_bins(i),:);
    temp_x = [linspace(0,max(temp_db.rpm_Mot_R),10)',ones(10,1).*windspeed_bins(i)];

    % Points
    
    if show_error_bar; hdls(i) = errorbar(temp_db.rpm_Mot_R,temp_db.Fx_hover,temp_db.std_Fx_hover,'*','color',col(i,:));
    else; hdls(i) = plot(temp_db.rpm_Mot_R,temp_db.Fx_hover,'*','color',col(i,:)); end
    
    hold on
        
    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];

    % With V^(1/2)
    plot(linspace(0,max(temp_db.rpm_Mot_R),10),fit_hover(s_hover,temp_x),'--','color',col(i,:))

    % Without V^(1/2)
    plot(linspace(0,max(temp_db.rpm_Mot_R),10),fit_hover_no_V12(s_hover_no_V12,temp_x),'-','color',col(i,:))

end
set(Ax(1), 'Box','off')
lgd1 = legend(hdls,legend_lbl,'location','southeast');
title(lgd1,'Airspeed') % add legend title

% copy axes 
Ax(2) = copyobj(Ax(1),gcf);
delete(get(Ax(2), 'Children') )

% plot helper data, but invisible
hold on
H1 = plot([NaN NaN],[NaN NaN], '-', 'LineWidth', 1, 'Color', [0 0 0], 'Parent', Ax(2));
H2 = plot([NaN NaN],[NaN NaN], '--', 'LineWidth', 1, 'Color', [0 0 0], 'Parent', Ax(2));
hold off
% make second axes invisible
set(Ax(2), 'Color', 'none', 'XTick', [], 'YAxisLocation', 'right', 'Box', 'Off', 'Visible', 'off')
% add linestyle legend
lgd2 = legend([H1 H2], 'With V^{1/2}', 'Without V^{1/2}', 'Location', 'southwest');
title(lgd2,'Fit Type') % add legend title
set(lgd2,'color','none')
title(sprintf('Comparison of fit at AoA = 0 deg\nWith V^{1/2}  |  Fx = K1*V^{2}+K2*V^{1/2}*RPM^{2} K1 = %2.2e K2 = %2.2e |  RMS = %2.2f\nWithout V^{1/2} |  Fx = K1*V^{2}+K2*RPM^{2} K1 = %2.2e K2 = %2.2e |  RMS = %2.2f',s_hover(1),s_hover(2),RMS_hover,s_hover_no_V12(1),s_hover_no_V12(2),RMS_hover_no_V12))
xlabel('Turn table angle (angle of attack) [deg]')
ylabel('Body Drag F_x [N]')
axis([-inf inf -inf inf])
grid on

%% Fit on maximum RPM
RPM_db = test_db(test_db.Turn_Table==deg2rad(0) & test_db.rpm_Mot_R>3500,:);

x = [RPM_db.Windspeed];
y = [RPM_db.Fx_hover];

fit = @(k,x)  k(1)*x(:,1); % Function to fit
fcn = @(k) sqrt(mean((fit(k,x) - y).^2));           % Least-Squares cost function
[s,RMS] = fminsearch(fcn,[1],options) %bound first coefficient to negative value

% s =
% 
%   -0.560675048828126
% 
% 
% RMS =
% 
%    1.433388187467630

figure
plot(x,y,'*')
hold on
plot(linspace(min(x),max(x),15),fit(s,linspace(min(x),max(x),15)'))

%% Nice plot

nice_db = test_db(test_db.Skew<deg2rad(10),:);

% Choose rpm above 3900 RPM
rpm_bins = unique(100.*round(nice_db.rpm_Mot_F./100,0));
rpm_bins = rpm_bins(rpm_bins>3700 & rpm_bins<4300);
nice_db.rpm_bin = 100.*round(nice_db.rpm_Mot_F./100,0);

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
col=linspecer(length(rpm_bins));
hdls = [];
Ax(1) = axes(fig); 
error_vec = [];
for i=1:length(rpm_bins)
    temp_db = nice_db(nice_db.rpm_bin==rpm_bins(i),:);
    

    temp_group = groupsummary(temp_db, ['windspeed_bin'], 'mean', 'Fx_hover');
    
    % Fit data
    x = [temp_group.windspeed_bin];
    y = [temp_group.mean_Fx_hover];
    
    fit = @(k,x)  k(1)*x(:,1); % Function to fit
    fcn = @(k) sqrt(mean((fit(k,x) - y).^2));           % Least-Squares cost function
    [s,RMS] = fminsearch(fcn,[-0.6],options);

    hdls(i,1) = plot(temp_group.windspeed_bin,temp_group.mean_Fx_hover,mymarkerstyles{i},'color',col(i,:),'MarkerSize',marker_size);
    hold on
    hdls(i,2) = plot(linspace(0,max(windspeed_bins),20),s.*linspace(0,max(windspeed_bins),20),'-','color',col(i,:)); 

    legend_lbl{i} = [mat2str(rpm_bins(i)),' RPM'];
    error_vec = [error_vec; temp_group.mean_Fx_hover- -0.75.*temp_group.windspeed_bin];
end

lgd1 = legend(hdls(:,1),legend_lbl,'Location', 'northoutside', 'Orientation', 'horizontal');

xlabel('Windspeed [m/s]')
ylabel('Hover Motor F_x [N]')
axis([0 inf -inf 0])
grid on

% Change font size
set(findall(gcf,'-property','FontSize'),'FontSize',font_size) 

fig_name = ['HOVER_FX_',formattedDateTime,'.eps'];
exportgraphics(fig,fig_name,'BackgroundColor','none','ContentType','vector')
grid on

% RMS
fprintf('Overall RMS %2.2f\n',rms(error_vec))