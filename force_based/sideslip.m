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
test_codes = unique(full_db.Code);

test = 'DT';
idx = contains(full_db.Code,test);

test_db = full_db(idx,:);

% Transform all angles in rad
deg_columns = {'Turn_Table','Skew','Skew_sp','Pitch','AoA','std_AoA'};
test_db{:,deg_columns} = deg2rad(test_db{:,deg_columns});

% Save
%save(['db_',test,'.mat'],'test_db')

%% Delete unecessary columns
all_columns = test_db.Properties.VariableNames;
columnsToKeep = {'Code','ID','Windspeed','Skew','Skew_sp','Pitch','Turn_Table','Mx','My','Mz','Fx','Fy','Fz','std_Mx','std_My','std_Mz','std_Fx','std_Fy','std_Fz','std_AoA'};
test_db(:,~ismember(all_columns,columnsToKeep)) = [];

%% Select only sideslip smaller than +-90 deg

test_db = test_db(abs(test_db.Turn_Table)<deg2rad(95),:);

%% Plot
windspeed_bins = unique(round(test_db.Windspeed,0));
test_db.Windspeed_bin = round(test_db.Windspeed,0);

figure
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = test_db(test_db.Windspeed_bin==windspeed_bins(i),:);

    subplot(2,2,1)
    hdls(1,i) = plot(-rad2deg(temp_db.Turn_Table),temp_db.Fy,'*','color',col(i,:));
    hold on

    subplot(2,2,2)
    hdls(2,i) = plot(rad2deg(temp_db.Skew_sp),temp_db.Fy,'*','color',col(i,:));
    hold on

    subplot(2,2,3)
    hdls(3,i) = plot(-rad2deg(temp_db.Turn_Table),temp_db.Fy./temp_db.Windspeed.^2,'*','color',col(i,:));
    hold on

    subplot(2,2,4)
    hdls(4,i) = plot(rad2deg(temp_db.Skew_sp),temp_db.Fy./temp_db.Windspeed.^2,'*','color',col(i,:));
    hold on

    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
end

subplot(2,2,1)
xlabel('\beta (beta) [deg]')
ylabel('F_y [N]')
%lgd1 = legend(hdls(1,:),legend_lbl,'location','southeast');
%title(lgd1,'Airspeed') % add legend title
subplot(2,2,2)
xlabel('Skew Setpoint [deg]')
ylabel('F_y [N]')
%lgd1 = legend(hdls(2,:),legend_lbl,'location','southeast');
%title(lgd1,'Airspeed') % add legend title
subplot(2,2,3)
xlabel('\beta) [deg]')
ylabel('F_y/Airspeed^2 [N/((m/s)^2)]')
%lgd1 = legend(hdls(3,:),legend_lbl,'location','southeast');
%title(lgd1,'Airspeed') % add legend title
grid on
subplot(2,2,4)
xlabel('Skew Setpoint [deg]')
ylabel('F_y/Airspeed^2 [N/((m/s)^2)]')
lgd1 = legend(hdls(4,:),legend_lbl,'location','southeast');
title(lgd1,'Airspeed') % add legend title
sgtitle('Sideforce Data from Wind Tunnel tests')

%% Fit on all data
% k  = [k_beta]
% x = [beta,skew,V]

% Beta = -turn table angle

x = [-test_db.Turn_Table,test_db.Skew_sp,test_db.Windspeed];
y = [test_db.Fy];

fit_quad = @(k,x)  (k(1).*x(:,1)).*x(:,3).^2;    % Function to fit
fcn_quad = @(k) sqrt(mean((fit_quad(k,x) - y).^2));           % Least-Squares cost function
[s_quad,RMS_all_quad] = fminsearch(fcn_quad,[1E-1])  

% s_quad =
% 
%   -0.219453124999999
% 
% 
% RMS_all_quad =
% 
%    1.206448910981871

k_beta_quad = s_quad(1); %N/(rad (m/s)^2)

fit_lin = @(k,x)  (k(1).*x(:,1)).*x(:,3);    % Function to fit
fcn_lin = @(k) sqrt(mean((fit_lin(k,x) - y).^2));               % Least-Squares cost function
[s_lin,RMS_all_lin] = fminsearch(fcn_lin,[1E-1])  

% s_lin =
% 
%   -2.552109374999995
% 
% 
% RMS_all_lin =
% 
%    4.774235997447986

k_beta_lin = s_lin(1); %N/(rad (m/s))

fit_zero = @(k,x)  (k(1).*x(:,1));    % Function to fit
fcn_zero = @(k) sqrt(mean((fit_zero(k,x) - y).^2));               % Least-Squares cost function
[s_zero,RMS_all_zero] = fminsearch(fcn_zero,[1E-1])  

% s_zero =
% 
%  -22.261093749999947
% 
% 
% RMS_all_zero =
% 
%   10.667648178124590

k_beta_zero = s_zero(1); %N/(rad (m/s))

% HENCE:
% Fy = (beta [rad] *-2.194341022544540*1E-1)* V [m/s]^2
% or
% Fy = (beta [rad] *-2.552098952269741*1E0)* V [m/s]
% or
% Fy = (beta [rad] *-2.226114526713827*1E1)

%% Visual Verification

figure
subplot(1,2,1)
plot3(-rad2deg(test_db.Turn_Table),rad2deg(test_db.Skew_sp),test_db.Fy./(test_db.Windspeed.^2),'*')
xlabel('\beta [deg]')
ylabel('Skew Setpoint [deg]')
zlabel('F_y/Airspeed^2 [N/((m/s)^2)]')
grid on

hold on
beta = linspace(min(test_db.Turn_Table),max(test_db.Turn_Table),10);
skew = linspace(min(test_db.Skew_sp),max(test_db.Skew_sp),10);
[BETA,SKEW] = meshgrid(beta,skew);
surf(rad2deg(BETA),rad2deg(SKEW),(k_beta_quad*BETA))

subplot(1,2,2)
plot3(-rad2deg(test_db.Turn_Table),test_db.Windspeed,test_db.Fy,'*')
xlabel('\beta [deg]')
ylabel('Wind Speed [m/s]')
zlabel('F_y [N]')
grid on
hold on
beta = linspace(min(test_db.Turn_Table),max(test_db.Turn_Table),10);
wind = linspace(min(test_db.Windspeed),max(test_db.Windspeed),10);
[BETA,WIND] = meshgrid(beta,wind);
surf(rad2deg(BETA),WIND,(k_beta_quad*BETA.*WIND.^2))
sgtitle(sprintf('Quadratic airspeed fit on skew and turn table\n k_{beta}=%2.2e [N/(rad*(m/s)^2)]\n RMS %2.2f [N]',k_beta_quad,RMS_all_quad))


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

windspeed_bins = unique(round(skew_db.Windspeed,0));
skew_db.Windspeed_bin = round(skew_db.Windspeed,0);
figure
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = skew_db(skew_db.Windspeed_bin==windspeed_bins(i),:);
    temp_x = [linspace(min(skew_db.Skew_sp),max(skew_db.Skew_sp),20)',ones(20,1).*windspeed_bins(i)];
    if show_error_bar; hdls(i) = errorbar(rad2deg(temp_db.Skew_sp),temp_db.Fy,temp_db.std_Fy,'*','color',col(i,:));
    else; hdls(i) = plot(rad2deg(temp_db.Skew_sp),temp_db.Fy,'*','color',col(i,:)); end
    
    hold on
    %plot(rad2deg(linspace(min(skew_db.Skew_sp),max(skew_db.Skew_sp),20)),fit_skew(s_skew,temp_x),'--','color',col(i,:))
    
    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
end
xlabel('Skew Setpoint [deg]')
ylabel('F_y [N]')
lgd1 = legend(hdls,legend_lbl,'location','southeast');
title(lgd1,'Airspeed') % add legend title
title('Sideforce due to skew angle for 0 angle of attack, 0 sideslip')
axis([0 inf -inf inf])
grid on

% Sideforce due to wing is minimal, because wing is not creating lift/drag at 0 angle of attack

%% Select test with changing angle of attack

test = 'LP5';
idx = contains(full_db.Code,test);

test_db_2 = full_db(idx,:);

% Transform all angles in rad
deg_columns = {'Turn_Table','Skew','Skew_sp','Pitch','AoA','std_AoA'};
test_db_2{:,deg_columns} = deg2rad(test_db_2{:,deg_columns});

%% Remove entries
% Removing entries with non-zero control surfaces
test_db_2 = test_db_2(test_db_2.Rud==0 & test_db_2.Elev==0 & test_db_2.Ail_R==0 & test_db_2.Ail_L==0 ,:);
% Removing entries with non-zero pusher motor
test_db_2 = test_db_2(test_db_2.Mot_Push==0,:);
% Removing entries with non-zero hover motor command
test_db = test_db(test_db.Mot_F==0 & test_db.Mot_R==0 & test_db.Mot_B==0 & test_db.Mot_L==0,:);
% Removing entries with angle of attack higher than 15 deg
test_db_2(abs(test_db_2.Turn_Table)>deg2rad(15),:) = [];

%% Obtaining wing Fx
% Fx = body drag + wing drag
% Wing drag = Fx-body drag
%Drag without hover props (only body drag)

%drag_body_coeff = [0                     -3.0061458379662E-2   3.497371379286E-3    1.46865971330522E-1];%all airspeeds without hover props without skew
drag_body_coeff = [-9.013405108486905E-3 -1.988035608425628E-2 9.850048188379294E-4 1.443975207474472E-1]; %all airspeeds without hover props with skew

Fx_body = @(alpha,skew,V) (drag_body_coeff(1)  .*  cos(skew)+...
                              drag_body_coeff(2)+...
                              drag_body_coeff(3)  .*  alpha+...
                              drag_body_coeff(4)  .*  alpha.^2).*V.^2;

test_db_2.Fx_wing = test_db_2.Fx-Fx_body(test_db_2.Turn_Table,test_db_2.Skew_sp,test_db_2.Windspeed);

% Calculating std dev of new Fx_pusher
% Mean C = mean A - mean B --> Variance C = Variance A + Variance B - 2*Correlation(A,B)*SD A * SD B
% Assumming uncorrelated
test_db_2.std_Fx_wing = sqrt(test_db_2.std_Fx.^2+test_db_2.std_Fx.^2);

% Get resultant drag force of the whole wing
test_db_2.F_wing = sqrt(test_db_2.Fx_wing.^2+test_db_2.Fy.^2);

%% Select high angle of attack
turn_table_bins = deg2rad(unique(round(rad2deg(test_db_2.Turn_Table),0)));
test_db_2.turn_table_bins = deg2rad(round(rad2deg(test_db_2.Turn_Table),0));

skew_db = test_db_2(test_db_2.Turn_Table>0,:);%skew_db = test_db_2(test_db_2.Turn_Table==turn_table_bins(end-2),:);

x = [skew_db.F_wing,skew_db.Skew_sp,skew_db.Windspeed];
y = [skew_db.Fy];

% Fy = (K1*F_wing + K2*F_wing*skew + K3*F_wing*skew^2)
fit_1 = @(k,x)  x(:,1).*(k(1)+k(2).*x(:,2)+k(3).*x(:,2).^2); % Function to fit
fcn_1 = @(k) sqrt(mean((fit_1(k,x) - y).^2));           % Least-Squares cost function
[s_1,RMS_1] = fminsearch(fcn_1,[0.8;0.6;-0.5],options) %bound first coefficient to negative value

% s =
% 
%    0.831044413346576
%    0.662270181202322
%   -0.659313505154950
% 
% 
% RMS =
% 
%    0.277928796479855

% Fy = (K1 + K2* F_wing * cos(skew))
fit_2 = @(k,x)  k(1)+k(2).*x(:,1).*cos(x(:,2)); % Function to fit
fcn_2 = @(k) sqrt(mean((fit_2(k,x) - y).^2));           % Least-Squares cost function
[s_2,RMS_2] = fminsearch(fcn_2,[0;1],options) %bound first coefficient to negative value

% s_2 =
% 
%    0.407720619975271
%    1.137633677149507
% 
% 
% RMS_2 =
% 
%    1.111483400870425

windspeed_bins = unique(round(skew_db.Windspeed,0));
skew_db.Windspeed_bin = round(skew_db.Windspeed,0);

f1 = figure();
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
Ax(1) = axes(f1); 
for i=1:length(windspeed_bins)
    temp_db = skew_db(skew_db.Windspeed_bin==windspeed_bins(i),:);
    temp_x = [temp_db.F_wing,temp_db.Skew_sp,temp_db.Windspeed];
    if show_error_bar; hdls(i) = errorbar(rad2deg(temp_db.Skew_sp),temp_db.Fy,temp_db.std_Fy,'*','color',col(i,:));
    else; hdls(i) = plot(rad2deg(temp_db.Skew_sp),temp_db.Fy,'*','color',col(i,:)); end
    hold on
    
    plot(rad2deg(temp_db.Skew_sp),fit_1(s_1,temp_x),'o','color',col(i,:))
    
    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
end
set(Ax(1), 'Box','off')
lgd1 = legend(hdls,legend_lbl,'location','southeast');
title(lgd1,'Airspeed') % add legend title

% copy axes 
Ax(2) = copyobj(Ax(1),gcf);
delete(get(Ax(2), 'Children') )

% plot helper data, but invisible
hold on
H1 = plot([NaN NaN],[NaN NaN], '*', 'LineWidth', 1, 'Color', [0 0 0], 'Parent', Ax(2));
H2 = plot([NaN NaN],[NaN NaN], 'o', 'LineWidth', 1, 'Color', [0 0 0], 'Parent', Ax(2));
hold off
% make second axes invisible
set(Ax(2), 'Color', 'none', 'XTick', [], 'YAxisLocation', 'right', 'Box', 'Off', 'Visible', 'off')
% add linestyle legend
lgd2 = legend([H1 H2], 'Data', 'Fit', 'Location', 'southwest');
title(lgd2,'Type') % add legend title
set(lgd2,'color','none')
title('Sideforce due to skew angle for different angles of attack and windspeed, 0 sideslip')
xlabel('Skew Setpoint [deg]')
ylabel('F_y [N]')
axis([-inf inf -inf inf])
grid on

%% Visualization for a single angle of attack
vis_db = test_db_2(test_db_2.Turn_Table==turn_table_bins(end-2),:);

windspeed_bins = unique(round(vis_db.Windspeed,0));
vis_db.Windspeed_bin = round(vis_db.Windspeed,0);

f1 = figure();
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
Ax(1) = axes(f1); 
for i=1:length(windspeed_bins)
    temp_db = vis_db(vis_db.Windspeed_bin==windspeed_bins(i),:);
    temp_x = [temp_db.F_wing,temp_db.Skew_sp,temp_db.Windspeed];
    if show_error_bar; hdls(i) = errorbar(rad2deg(temp_db.Skew_sp),temp_db.Fy,temp_db.std_Fy,'*','color',col(i,:));
    else; hdls(i) = plot(rad2deg(temp_db.Skew_sp),temp_db.Fy,'*','color',col(i,:)); end
    hold on
    
    plot(rad2deg(temp_db.Skew_sp),fit_1(s_1,temp_x),'-','color',col(i,:))
    plot(rad2deg(temp_db.Skew_sp),fit_2(s_2,temp_x),'--','color',col(i,:))
    
    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
end
set(Ax(1), 'Box','off')
lgd1 = legend(hdls,legend_lbl,'location','northeast');
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
lgd2 = legend([H1 H2], 'Quadratic', 'Trigonometric', 'Location', 'northwest');
title(lgd2,'Fit Type') % add legend title
set(lgd2,'color','none')
title(sprintf(['Comparison of fit at AoA = %2.2f deg\nFy = K1*F_{wing} + K2*F_{wing}*skew + K3*F_{wing}*skew^2 |  K1 = %2.2e K2 = %2.2e K3 = %2.2e  |  RMS = %2.2f\n' ...
    'Fy = K1 + K2* F_{wing} * cos(skew) |  K1 = %2.2e K2 = %2.2e  |  RMS = %2.2f'],rad2deg(vis_db.Turn_Table(1)),s_1(1),s_1(2),s_1(3),RMS_1,s_2(1),s_2(2),RMS_2))
xlabel('Skew Setpoint [deg]')
ylabel('F_y [N]')
axis([-inf inf -inf inf])
grid on

%% Other visualization
windspeed_bins = unique(round(skew_db.Windspeed,0));
skew_db.Windspeed_bin = round(skew_db.Windspeed,0);
figure
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = skew_db(skew_db.Windspeed_bin==windspeed_bins(i),:);
    temp_x = [linspace(min(skew_db.Skew_sp),max(skew_db.Skew_sp),20)',ones(20,1).*windspeed_bins(i)];

    subplot(3,1,1)
    plot(temp_db.Fx_wing,temp_db.Fy,'-','color',col(i,:))
    hold on
    xlabel('Fx')
    ylabel('Fy')
    subplot(3,1,2)
    plot(temp_db.F_wing,temp_db.Fx_wing,'-','color',col(i,:))
    hold on
    xlabel('Ftot')
    ylabel('Fx')
    subplot(3,1,3)
    plot(temp_db.F_wing,temp_db.Fy,'-','color',col(i,:))
    hold on
    xlabel('Ftot')
    ylabel('Fy')

end

figure
for i=1:length(windspeed_bins)
    temp_db = skew_db(skew_db.Windspeed_bin==windspeed_bins(i),:);
    temp_x = [linspace(min(skew_db.Skew_sp),max(skew_db.Skew_sp),20)',ones(20,1).*windspeed_bins(i)];

    subplot(3,1,1)
    plot(temp_db.Skew_sp,temp_db.F_wing,'-','color',col(i,:))
    hold on
    xlabel('Skew')
    ylabel('Ftot')
    subplot(3,1,2)
    plot(temp_db.Skew_sp,temp_db.Fx_wing,'-','color',col(i,:))
    hold on
    xlabel('Skew')
    ylabel('Fx')
    subplot(3,1,3)
    plot(temp_db.Skew_sp,temp_db.Fy,'-','color',col(i,:))
    hold on
    xlabel('Skew')
    ylabel('Fy')

end

%% Plot and fit sideforce depending on v (side velocity)

x = [sin(-test_db.Turn_Table).*test_db.Windspeed];
y = [test_db.Fy];

% Fy = (K1*v^2*-sign(v))
fit_v = @(k,x)  x(:,1).^2.*k(1).*sign(x(:,1)); % Function to fit
fcn_v = @(k) sqrt(mean((fit_v(k,x) - y).^2));           % Least-Squares cost function
[s_v,RMS_v] = fminsearch(fcn_v,[0.1],options) %bound first coefficient to negative value

% s_v =
% 
%   -0.320878906249999
% 
% 
% RMS_v =
% 
%    3.359814374020583

skew_bins = unique(deg2rad(round(rad2deg(test_db.Skew_sp),0)));
test_db.skew_bins = deg2rad(round(rad2deg(test_db.Skew_sp),0));

legend_lbl = {};
col=linspecer(length(skew_bins));
hdls = [];

figure
for i=1:length(skew_bins)
    temp_db = test_db(test_db.skew_bins==skew_bins(i),:);
    temp_x = [linspace(min(sin(-test_db.Turn_Table).*test_db.Windspeed),max(sin(-test_db.Turn_Table).*test_db.Windspeed),20)'];
    
    hdls(i) = plot((sin(-temp_db.Turn_Table).*temp_db.Windspeed),temp_db.Fy,'*','color',col(i,:));
    hold on
    
    legend_lbl{i} = [mat2str(rad2deg(skew_bins(i))),' deg'];
end
plot(temp_x,fit_v(s_v,temp_x),'-')
xlabel('v [m/s]')
ylabel('F_y [N]')

lgd1 = legend(hdls(:),legend_lbl,'location','southeast');
title(lgd1,'Skew Angle') % add legend title
sgtitle('Sideforce Data from Wind Tunnel tests')

