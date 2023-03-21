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

test = 'AE';
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
            correction=correction+1;
        end
    end
end
fprintf(sprintf('Corrected RPM %d times\n',correction))

%% Obtaining hover prop lift
% Fz = body lift + Hover_motors lift
% Hover_motors lift = Fz-body lift
%Drag without hover props (only body drag)

lift_body_coeff = [-1.569286184145456E-3 5.989835400355119E-3 -2.346715949355502E-1 6.611857425073364E-2]; %all airspeeds without hover props with skew

Fz_body = @(alpha,skew,V) (lift_body_coeff(1)  .*  cos(skew)+...
                           lift_body_coeff(2)+...
                           lift_body_coeff(3)  .*  alpha+...
                           lift_body_coeff(4)  .*  alpha.^2).*V.^2;

test_db.Fz_hover = test_db.Fz-Fz_body(test_db.Turn_Table,test_db.Skew_sp,test_db.Windspeed);

% Calculating std dev of new Fz_pusher
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
