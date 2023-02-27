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
% Removing entries with non-zero hover motor command
test_db = test_db(test_db.Mot_F==0 & test_db.Mot_R==0 & test_db.Mot_B==0 & test_db.Mot_L==0,:);
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
Fx_body =  @(alpha,V) (-0.030061458379662+0.003497371379286.*alpha+0.146865971330522.*alpha.^2).*V.^2;
test_db.Fx_hover = test_db.Fx-Fx_body(test_db.Turn_Table,test_db.Windspeed);

% Calculating std dev of new Fx_pusher
% Mean C = mean A - mean B --> Variance C = Variance A + Variance B - 2*Correlation(A,B)*SD A * SD B
% Assumming uncorrelated
test_db.std_Fx_hover = sqrt(test_db.std_Fx.^2+test_db.std_Fx.^2);

%% Analyze hover prop RPM effect
%Different airspeed and different hover prop values
hover_prop_db = test_db(test_db.Turn_Table==deg2rad(0) & test_db.Skew_sp==deg2rad(0),:);

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
lgd1 = legend(hdls,legend_lbl,'location','southeast');
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