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

test = 'LP3';
idx = contains(full_db.Code,test);

test_db = full_db(idx,:);

% Transform all angles in rad
deg_columns = {'Turn_Table','Skew','Skew_sp','Pitch','AoA','std_AoA'};
test_db{:,deg_columns} = deg2rad(test_db{:,deg_columns});

% Save
save(['db_',test,'.mat'],'test_db')

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
            correction=correction+1;
        end
    end
end
fprintf(sprintf('Corrected RPM %d times\n',correction))

%% Obtaining hover prop drag
% Fx = body drag + Hover_motors drag
% Hover_motors drag = Fx-body drag
%Drag without hover props (only body drag)

%drag_body_coeff = [0                     -3.0061458379662E-2   3.497371379286E-3    1.46865971330522E-1];%all airspeeds without hover props without skew
drag_body_coeff = [-9.013405108486905E-3 -1.988035608425628E-2 9.850048188379294E-4 1.443975207474472E-1]; %all airspeeds without hover props with skew

Fx_body = @(alpha,skew,V) (drag_body_coeff(1)  .*  cos(skew)+...
                              drag_body_coeff(2)+...
                              drag_body_coeff(3)  .*  alpha+...
                              drag_body_coeff(4)  .*  alpha.^2).*V.^2;

test_db.Fx_hover = test_db.Fx-Fx_body(test_db.Turn_Table,test_db.Skew_sp,test_db.Windspeed);

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


% Fx hover props = -6.435825732350349E-3*V^2+-1.180349532783032E-7*V^{1/2}*RPM^2
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