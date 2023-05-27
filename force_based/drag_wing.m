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
idx_1 = contains(full_db.Code,test_1);
idx_2 = contains(full_db.Code,test_2);

test_db_1 = full_db(idx_1,:);
test_db_2 = full_db(idx_2,:);

forces_columns = {'Mx','My','Mz','Fx','Fy','Fz',};

% Substract db2 from db1
test_db = table();
for i=1:size(test_db_1,1)
    id = test_db_1.ID{i}(length(test_1)+2:end);
    if any(contains(test_db_2.ID,id))
        test_db(end+1,:) = test_db_1(i,:);
        test_db{end,forces_columns} = test_db_1{i,forces_columns} -test_db_2{contains(test_db_2.ID,id),forces_columns};
        test_db.Code{end} = [test_1 '-' test_2];
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

%% Obtaining wing Fx
test_db.Fx_wing = test_db.Fx;

save('db_Fx_wing.mat','test_db')
%% Plot Fx
windspeed_bins = unique(round(test_db.Windspeed,0));
test_db.Windspeed_bin = round(test_db.Windspeed,0);

figure
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = test_db(test_db.Windspeed_bin==windspeed_bins(i),:);

    subplot(2,2,1)
    hdls(1,i) = plot(rad2deg(temp_db.Turn_Table),temp_db.Fx_wing,'*','color',col(i,:));
    hold on

    subplot(2,2,2)
    hdls(2,i) = plot(rad2deg(temp_db.Skew_sp),temp_db.Fx_wing,'*','color',col(i,:));
    hold on

    subplot(2,2,3)
    hdls(3,i) = plot(rad2deg(temp_db.Turn_Table),temp_db.Fx_wing./temp_db.Windspeed.^2,'*','color',col(i,:));
    hold on

    subplot(2,2,4)
    hdls(4,i) = plot(rad2deg(temp_db.Skew_sp),temp_db.Fx_wing./temp_db.Windspeed.^2,'*','color',col(i,:));
    hold on

    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
end

subplot(2,2,1)
xlabel('Turn table angle (angle of attack) [deg]')
ylabel('F_x Wing [N]')
%lgd1 = legend(hdls(1,:),legend_lbl,'location','southeast');
%title(lgd1,'Airspeed') % add legend title
subplot(2,2,2)
xlabel('Skew Setpoint [deg]')
ylabel('F_x Wing [N]')
%lgd1 = legend(hdls(2,:),legend_lbl,'location','southeast');
%title(lgd1,'Airspeed') % add legend title
subplot(2,2,3)
xlabel('Turn table angle (angle of attack) [deg]')
ylabel('F_x Wing/Airspeed^2 [N/((m/s)^2)]')
%lgd1 = legend(hdls(3,:),legend_lbl,'location','southeast');
%title(lgd1,'Airspeed') % add legend title
grid on
subplot(2,2,4)
xlabel('Skew Setpoint [deg]')
ylabel('F_x Wing/Airspeed^2 [N/((m/s)^2)]')
lgd1 = legend(hdls(4,:),legend_lbl,'location','southeast');
title(lgd1,'Airspeed') % add legend title
sgtitle('Fx Wing Drag Data from Wind Tunnel tests')

%% Wing Drag in quad mode
quad_db = test_db(test_db.Skew_sp==deg2rad(0),:);

% Fx = (k1+k2*alpha+k3*alpha^2)*V^2
% k  = [k1 k2 k3 k4]
% x = [pprz,V]
x = [quad_db.Turn_Table,quad_db.Windspeed];
y = [quad_db.Fx_wing];

fit_quad = @(k,x)  (k(1)+k(2).*x(:,1)+k(3).*x(:,1).^2).*x(:,2).^2; % Function to fit
fcn_quad = @(k) sqrt(mean((fit_quad(k,x) - y).^2));           % Least-Squares cost function
[s_quad,RMS_quad] = fminsearch(fcn_quad,[-4E-2;0;0],options) 

% s_quad =
% 
%   -0.004869627275552
%   -0.000990964612148
%   -0.096495386865907
% 
% 
% RMS_quad =
% 
%    0.248298288225588

figure
windspeed_bins = unique(round(quad_db.Windspeed,0));
quad_db.Windspeed_bin = round(quad_db.Windspeed,0);
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = quad_db(quad_db.Windspeed_bin==windspeed_bins(i),:);
    temp_x = [linspace(min(temp_db.Turn_Table),max(temp_db.Turn_Table),10)',ones(10,1).*windspeed_bins(i)];

    if show_error_bar; hdls(i) = errorbar(rad2deg(temp_db.Turn_Table),temp_db.Fx_wing,temp_db.std_Fx,'*','color',col(i,:));
    else; hdls(i) = plot(rad2deg(temp_db.Turn_Table),temp_db.Fx_wing,'*','color',col(i,:)); end
    
    hold on
    plot(rad2deg(linspace(min(temp_db.Turn_Table),max(temp_db.Turn_Table),10)),fit_quad(s_quad,temp_x),'--','color',col(i,:))
    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
end
lgd1 = legend(hdls,legend_lbl,'location','southeast');
title(lgd1,'Airspeed') % add legend title
title(sprintf('Wing Drag Quad Mode\nFx = (k1+k2*alpha+k3*alpha^2)*V^2\nK1 = %2.2e K2 = %2.2e K3 = %2.2e |  RMS = %2.2f',s_quad(1),s_quad(2),s_quad(3),RMS_quad))
xlabel('Turn table angle (angle of attack) [deg]')
ylabel('Wing Drag F_x [N]')
axis([-inf inf -inf inf])
grid on

%% Wing Drag in fordward flight mode
ff_db = test_db(test_db.Skew_sp==deg2rad(90),:);

% Fx = (k1+k2*alpha+k3*alpha^2)*V^2
% k  = [k1 k2 k3 k4]
% x = [AoA,V]
x = [ff_db.Turn_Table,ff_db.Windspeed];
y = [ff_db.Fx_wing];

fit_ff = @(k,x)  (k(1)+k(2).*x(:,1)+k(3).*x(:,1).^2).*x(:,2).^2; % Function to fit
fcn_ff = @(k) sqrt(mean((fit_ff(k,x) - y).^2));           % Least-Squares cost function
[s_ff,RMS_ff] = fminsearch(fcn_ff,[-3E-2;1E-1;8E-1],options) 

% s_ff =
% 
%   -0.002800986894285
%    0.169228749580798
%    0.576150103424988
% 
% 
% RMS_ff =
% 
%    0.190593006280145

figure
windspeed_bins = unique(round(ff_db.Windspeed,0));
ff_db.Windspeed_bin = round(ff_db.Windspeed,0);
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = ff_db(ff_db.Windspeed_bin==windspeed_bins(i),:);
    temp_x = [linspace(min(temp_db.Turn_Table),max(temp_db.Turn_Table),10)',ones(10,1).*windspeed_bins(i)];

    if show_error_bar; hdls(i) = errorbar(rad2deg(temp_db.Turn_Table),temp_db.Fx_wing,temp_db.std_Fx,'*','color',col(i,:));
    else; hdls(i) = plot(rad2deg(temp_db.Turn_Table),temp_db.Fx_wing,'*','color',col(i,:)); end
    
    hold on
    plot(rad2deg(linspace(min(temp_db.Turn_Table),max(temp_db.Turn_Table),10)),fit_ff(s_ff,temp_x),'--','color',col(i,:))
    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
end
lgd1 = legend(hdls,legend_lbl,'location','southeast');
title(lgd1,'Airspeed') % add legend title
title(sprintf('Forward Flight\nFx = (k1+k2*alpha+k3*alpha^2)*V^2\nK1 = %2.2e K2 = %2.2e K3 = %2.2e |  RMS = %2.2f',s_ff(1),s_ff(2),s_ff(3),RMS_ff))
xlabel('Turn table angle (angle of attack) [deg]')
ylabel('Wing Drag F_x [N]')
axis([-inf inf -inf inf])
grid on

%% Defining fit functions
% Fx0 = (k1+k2*alpha+k3*alpha^2)             *(k4*sin(skew)^2+k5)*V^2
% Fx1 = (k1   +         (k2*alpha+k3*alpha^2))*(sin(skew)^2+k4)*V^2
% Fx2 = (k1*(1+k5*skew)+(k2*alpha+k3*alpha^2))*(sin(skew)^2+k4)*V^2
% Fx3 = ((k1+k2*alpha+k3*alpha^2)             *(sin(skew)^2+k4)*V^2
% Fx4 = ((k1+k2*alpha+k3*alpha^2)             *(sin(skew)^2)+k4)*V^2
% Fx5 = ((k1+k2*alpha+k3*alpha^2)             *(sin(skew)^2+k4)+k5)*V^2
% Fx6 = (k1+k5*sin(skew))+(k2*alpha+k3*alpha^2))*(sin(skew)^2+k4)*V^2

fit_0 = @(k,x)  x(:,2).^2.*(k(1)+ k(2).*x(:,1)+k(3).*x(:,1).^2) .* (k(4)*sin(x(:,3)).^2+k(5)); % Function to fit

fit_1 = @(k,x)  x(:,2).^2.*(k(1)+ (k(2).*x(:,1)+k(3).*x(:,1).^2) .* (sin(x(:,3)).^2+k(4)) ); % Function to fit

fit_2 = @(k,x)  x(:,2).^2.*(k(1).*(1+k(5).*x(:,3))+ (k(2).*x(:,1)+k(3).*x(:,1).^2).*(sin(x(:,3)).^2+k(4))); % Function to fit

fit_3 = @(k,x)  x(:,2).^2.*( (k(1)+k(2).*x(:,1)+k(3).*x(:,1).^2) .* (sin(x(:,3)).^2+k(4)) ); % Function to fit

fit_4 = @(k,x)  x(:,2).^2.*( (k(1)+k(2).*x(:,1)+k(3).*x(:,1).^2) .* (sin(x(:,3)).^2) + k(4) ); % Function to fit

fit_5 = @(k,x)  x(:,2).^2.*( (k(1)+k(2).*x(:,1)+k(3).*x(:,1).^2) .* (sin(x(:,3)).^2+k(4)) + k(5) ); % Function to fit

fit_6 = @(k,x)  x(:,2).^2.*(k(1)+k(5).*sin(x(:,3))+ (k(2).*x(:,1)+k(3).*x(:,1).^2).*(sin(x(:,3)).^2+k(4))); % Function to fit

%% Wing Drag fixed airspeed, different skews
skew_db = test_db(test_db.Windspeed<12 & test_db.Windspeed>9,:);

% k  = [k1 k2 k3 k4]
% x = [AoA,V,skew]
x = [skew_db.Turn_Table,skew_db.Windspeed,skew_db.Skew_sp];
y = [skew_db.Fx_wing];

fcn_0 = @(k) sqrt(mean((fit_0(k,x) - y).^2));           % Least-Squares cost function
[s_skew_0,RMS_skew_0,~,~] = fminsearchbnd(fcn_0,[-7E-3;2E-1;8E-1;1;-2E-1],[-inf -inf -inf 0.25 -inf],[inf inf inf 1.75 inf],options)

fcn_1 = @(k) sqrt(mean((fit_1(k,x) - y).^2));           % Least-Squares cost function
[s_skew_1,RMS_skew_1] = fminsearch(fcn_1,[0;0;0;-4E-2],options) 

fcn_2 = @(k) sqrt(mean((fit_2(k,x) - y).^2));           % Least-Squares cost function
[s_skew_2,RMS_skew_2] = fminsearch(fcn_2,[0;0;0;-4E-2;0],options) 

fcn_3 = @(k) sqrt(mean((fit_3(k,x) - y).^2));           % Least-Squares cost function
[s_skew_3,RMS_skew_3] = fminsearch(fcn_3,[-2E-2;1E-1;8E-1;1E-1],options) 

fcn_4 = @(k) sqrt(mean((fit_4(k,x) - y).^2));           % Least-Squares cost function
[s_skew_4,RMS_skew_4] = fminsearch(fcn_4,[0;0;0;0],options) 

fcn_5 = @(k) sqrt(mean((fit_5(k,x) - y).^2));           % Least-Squares cost function
[s_skew_5,RMS_skew_5] = fminsearch(fcn_5,[0;0;0;0;0],options) 

fcn_6 = @(k) sqrt(mean((fit_6(k,x) - y).^2));           % Least-Squares cost function
[s_skew_6,RMS_skew_6] = fminsearch(fcn_6,[0;0;0;0;0],options) 


% s_skew_0 =
% 
%   -0.006389565225176
%    0.247125961776396
%    0.905125085056246
%    0.740317318018966
%   -0.053642714957628
% 
% 
% RMS_skew_0 =
% 
%    0.415562202050763
% 
% 
% s_skew_1 =
% 
%   -0.004339186576666
%    0.169935631438089
%    0.648357754204796
%   -0.018069443644292
% 
% 
% RMS_skew_1 =
% 
%    0.269460697230777
% 
%  
% Exiting: Maximum number of function evaluations has been exceeded
%          - increase MaxFunEvals option.
%          Current function value: 0.316498 
% 
% 
% s_skew_2 =
% 
%   -0.002723395661958
%    0.187976572380121
%    0.666313616791292
%   -0.060612973516036
%    0.426398426797174
% 
% 
% RMS_skew_2 =
% 
%    0.316497931387635
% 
% 
% s_skew_3 =
% 
%   -0.004730248880130
%    0.182954145876165
%    0.670073870382007
%   -0.072461249451492
% 
% 
% RMS_skew_3 =
% 
%    0.415562202210446
% 
% 
% s_skew_4 =
% 
%    0.002204919479303
%    0.167394836881765
%    0.609610146102001
%   -0.005425504789052
% 
% 
% RMS_skew_4 =
% 
%    0.259808937675339
% 
% 
% s_skew_5 =
% 
%    0.002078611647114
%    0.168196119807633
%    0.612694551151183
%   -0.003827649194794
%   -0.005318926818865
% 
% 
% RMS_skew_5 =
% 
%    0.259651155173138
% 
% 
% s_skew_6 =
% 
%   -0.006428638953316
%    0.167195264490172
%    0.594410370645878
%    0.003983889380919
%    0.003532085496834

figure
windspeed_bins = unique(round(skew_db.Windspeed,0));
skew_bins = deg2rad(unique(round(rad2deg(skew_db.Skew_sp),0)));
skew_db.skew_bin = deg2rad(round(rad2deg(skew_db.Skew_sp),0));
legend_lbl = {};
col=linspecer(length(skew_bins));
hdls = [];
for i=1:length(skew_bins)
    temp_db = skew_db(skew_db.skew_bin==skew_bins(i),:);
    temp_x = [linspace(min(temp_db.Turn_Table),max(temp_db.Turn_Table),20)',ones(20,1).*windspeed_bins,ones(20,1).*skew_bins(i)];

    if show_error_bar; hdls(i) = errorbar(rad2deg(temp_db.Turn_Table),temp_db.Fx_wing,temp_db.std_Fx,'*','color',col(i,:));
    else; hdls(i) = plot(rad2deg(temp_db.Turn_Table),temp_db.Fx_wing,'*','color',col(i,:)); end
    
    hold on
    plot(rad2deg(linspace(min(temp_db.Turn_Table),max(temp_db.Turn_Table),20)),fit_1(s_skew_1,temp_x),'--','color',col(i,:))
    legend_lbl{i} = [mat2str(rad2deg(skew_bins(i))),' deg'];
end
lgd1 = legend(hdls,legend_lbl,'location','southeast');
title(lgd1,'Skew') % add legend title
title(sprintf('Windspeed = 11 m/s\nFx1 = (k1+(k2*alpha+k3*alpha^2))*(sin(skew)^2+k4)*V^2\nK1 = %2.2e K2 = %2.2e K3 = %2.2e K4 = %2.2e  |  RMS = %2.2f',s_skew_1(1),s_skew_1(2),s_skew_1(3),s_skew_1(4),RMS_skew_1))
xlabel('Turn table angle (angle of attack) [deg]')
ylabel('Wing Drag F_x [N]')
axis([-inf inf -inf inf])
grid on

% RMS on all airspeeds when gains are calculated using only one airspeed
x = [test_db.Turn_Table,test_db.Windspeed,test_db.Skew_sp];
y = [test_db.Fx_wing];
RMS_all_0 = sqrt(mean((fit_0(s_skew_0,x) - y).^2))
RMS_all_1 = sqrt(mean((fit_1(s_skew_1,x) - y).^2))
RMS_all_2 = sqrt(mean((fit_2(s_skew_2,x) - y).^2))
RMS_all_3 = sqrt(mean((fit_3(s_skew_3,x) - y).^2))
RMS_all_4 = sqrt(mean((fit_4(s_skew_4,x) - y).^2))
RMS_all_5 = sqrt(mean((fit_5(s_skew_5,x) - y).^2))
RMS_all_6 = sqrt(mean((fit_6(s_skew_6,x) - y).^2))

% RMS_all_0 =
% 
%    0.400895297482130
% 
% 
% RMS_all_1 =
% 
%    0.371082869146817
% 
% 
% RMS_all_2 =
% 
%    0.372128946774269
% 
% 
% RMS_all_3 =
% 
%    0.400893861715477
% 
% 
% RMS_all_4 =
% 
%    0.369822226929969
% 
% 
% RMS_all_5 =
% 
%    0.368046727746292
% 
% 
% RMS_all_6 =
% 
%    0.356955615868584

% RMS on ff flight mode and quad mode
verif_db = test_db(test_db.Skew_sp==deg2rad(0) | test_db.Skew_sp==deg2rad(90),:);

x = [verif_db.Turn_Table,verif_db.Windspeed,verif_db.Skew_sp];
y = [verif_db.Fx_wing];
RMS_quad_ff_0 = sqrt(mean((fit_0(s_skew_0,x) - y).^2))
RMS_quad_ff_1 = sqrt(mean((fit_1(s_skew_1,x) - y).^2))
RMS_quad_ff_2 = sqrt(mean((fit_2(s_skew_2,x) - y).^2))
RMS_quad_ff_3 = sqrt(mean((fit_3(s_skew_3,x) - y).^2))
RMS_quad_ff_4 = sqrt(mean((fit_4(s_skew_4,x) - y).^2))
RMS_quad_ff_5 = sqrt(mean((fit_5(s_skew_5,x) - y).^2))
RMS_quad_ff_6 = sqrt(mean((fit_6(s_skew_6,x) - y).^2))

% RMS_quad_ff_0 =
% 
%    0.509635034804140
% 
% 
% RMS_quad_ff_1 =
% 
%    0.335503252131224
% 
% 
% RMS_quad_ff_2 =
% 
%    0.350840605819728
% 
% 
% RMS_quad_ff_3 =
% 
%    0.509631203112162
% 
% 
% RMS_quad_ff_4 =
% 
%    0.323681712758922
% 
% 
% RMS_quad_ff_5 =
% 
%    0.318247909795392
% 
% 
% RMS_quad_ff_6 =
% 
%    0.306488942975441

%% Analyze result of fit
RMS = sqrt(mean((fit_6(s_skew_6,x) - y).^2))
range = max(y)-min(y)
RMS_percentage_range = RMS./range
max_error = max(abs((fit_6(s_skew_6,x) - y)))
max_error_percentage_range = max_error./range

%% Fit on all Airspeed
% Fx0 = (k1+k2*alpha+k3*alpha^2)             *(k4*sin(skew)^2+k5)*V^2
% Fx1 = (k1   +         (k2*alpha+k3*alpha^2))*(sin(skew)^2+k4)*V^2
% Fx2 = (k1*(1+k5*skew)+(k2*alpha+k3*alpha^2))*(sin(skew)^2+k4)*V^2
% Fx3 = ((k1+k2*alpha+k3*alpha^2)             *(sin(skew)^2+k4)*V^2
% Fx4 = ((k1+k2*alpha+k3*alpha^2)             *(sin(skew)^2)+k4)*V^2
% Fx5 = ((k1+k2*alpha+k3*alpha^2)             *(sin(skew)^2+k4)+k5)*V^2

% k  = [k1 k2 k3 k4]
% x = [AoA,V,skew]
x = [test_db.Turn_Table,test_db.Windspeed,test_db.Skew_sp];
y = [test_db.Fx_wing];

fcn_0 = @(k) sqrt(mean((fit_0(k,x) - y).^2));           % Least-Squares cost function
[s_all_0,RMS_all_0] = fminsearchbnd(fcn_0,[-7E-3;2E-1;8E-1;1;-2E-1],[-inf -inf -inf 0.25 -inf],[inf inf inf 1.75 inf],options)

fcn_1 = @(k) sqrt(mean((fit_1(k,x) - y).^2));           % Least-Squares cost function
[s_all_1,RMS_all_1] = fminsearch(fcn_1,[0;0;0;-4E-2],options)

fcn_2 = @(k) sqrt(mean((fit_2(k,x) - y).^2));           % Least-Squares cost function
[s_all_2,RMS_all_2] = fminsearch(fcn_2,[0;0;0;-4E-2;0],options) 

fcn_3 = @(k) sqrt(mean((fit_3(k,x) - y).^2));           % Least-Squares cost function
[s_all_3,RMS_all_3] = fminsearch(fcn_3,[-2E-2;1E-1;8E-1;1E-1],options) 

fcn_4 = @(k) sqrt(mean((fit_4(k,x) - y).^2));           % Least-Squares cost function
[s_all_4,RMS_all_4] = fminsearch(fcn_4,[0;0;0;0;0],options) 

fcn_5 = @(k) sqrt(mean((fit_5(k,x) - y).^2));           % Least-Squares cost function
[s_all_5,RMS_all_5] = fminsearch(fcn_5,[0;0;0;0;0],options) 

fcn_6 = @(k) sqrt(mean((fit_6(k,x) - y).^2));           % Least-Squares cost function
[s_all_6,RMS_all_6] = fminsearch(fcn_6,[0;0;0;0;0],options) 

% RMS_all_0 =
% 
%    0.386952932108423
% 
% 
% s_all_1 =
% 
%   -0.002601792206941
%    0.179883791169228
%    0.611798113080549
%   -0.041770608930273
% 
% 
% RMS_all_1 =
% 
%    0.335121716419165
% 
% 
% s_all_2 =
% 
%   -0.003737310168884
%    0.178460735801756
%    0.573232139607035
%   -0.028402292516850
%   -0.415777251977168
% 
% 
% RMS_all_2 =
% 
%    0.328575524322293
% 
% 
% s_all_3 =
% 
%   -0.002720300583516
%    0.188406658696613
%    0.622379355158904
%   -0.075912544837951
% 
% 
% RMS_all_3 =
% 
%    0.386952932171058
% 
% 
% s_all_4 =
% 
%    0.002587513570566
%    0.171805402278874
%    0.559272447080590
%   -0.004153349827198
%    0.708283634247233
% 
% 
% RMS_all_4 =
% 
%    0.339930172933094
% 
% 
% s_all_5 =
% 
%    0.001507991229935
%    0.178752719373553
%    0.585319252507548
%   -0.032179212999776
%   -0.003267084435135
% 
% 
% RMS_all_5 =
% 
%    0.331092904450090
% 
% 
% s_all_6 =
% 
%   -0.004456003510673
%    0.177673241987774
%    0.562762931473108
%   -0.023512678648294
%    0.003143161313704
% 
% 
% RMS_all_6 =
% 
%    0.319024005842446

% RMS on ff flight mode and quad mode
verif_db = test_db(test_db.Skew_sp==deg2rad(0) | test_db.Skew_sp==deg2rad(90),:);

x = [verif_db.Turn_Table,verif_db.Windspeed,verif_db.Skew_sp];
y = [verif_db.Fx_wing];
RMS_quad_ff_0 = sqrt(mean((fit_0(s_all_0,x) - y).^2))
RMS_quad_ff_1 = sqrt(mean((fit_1(s_all_1,x) - y).^2))
RMS_quad_ff_2 = sqrt(mean((fit_2(s_all_2,x) - y).^2))
RMS_quad_ff_3 = sqrt(mean((fit_3(s_all_3,x) - y).^2))
RMS_quad_ff_4 = sqrt(mean((fit_4(s_all_4,x) - y).^2))
RMS_quad_ff_5 = sqrt(mean((fit_5(s_all_5,x) - y).^2))
RMS_quad_ff_6 = sqrt(mean((fit_6(s_all_6,x) - y).^2))

% RMS_quad_ff_0 =
% 
%    0.491034248614084
% 
% 
% RMS_quad_ff_1 =
% 
%    0.368377289272403
% 
% 
% RMS_quad_ff_2 =
% 
%    0.349168808695396
% 
% 
% RMS_quad_ff_3 =
% 
%    0.491033047520525
% 
% 
% RMS_quad_ff_4 =
% 
%    0.386936254051868
% 
% 
% RMS_quad_ff_5 =
% 
%    0.353990475060111
% 
% 
% RMS_quad_ff_6 =
% 
%    0.322702982902347

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

nice_db = test_db;

windspeed_bins = unique(round(nice_db.Windspeed,0));
nice_db.Windspeed_bin = round(nice_db.Windspeed,0);
skew_bins = deg2rad(unique(round(rad2deg(nice_db.Skew_sp),0)));
skew_bins = skew_bins(1:2:end);
nice_db.skew_bin = deg2rad(round(rad2deg(nice_db.Skew_sp),0));

legend_lbl = {};
col=linspecer(length(skew_bins));
hdls = [];
for i=1:length(skew_bins)
    temp_db = nice_db(nice_db.skew_bin==skew_bins(i),:);
    temp_x = [linspace(min(temp_db.Turn_Table),max(temp_db.Turn_Table),20)',ones(20,1).*1,ones(20,1).*skew_bins(i)];
    
    temp_db.Fx_scaled = temp_db.Fx_wing./temp_db.Windspeed.^2;

    temp_group = groupsummary(temp_db, ['Turn_Table'], 'mean', 'Fx_scaled');

    hdls(i,1) = plot(rad2deg(temp_group.Turn_Table),temp_group.mean_Fx_scaled,mymarkerstyles{i},'color',col(i,:),'MarkerSize',marker_size);
    hold on
    hdls(i,2) = plot(rad2deg(linspace(min(temp_db.Turn_Table),max(temp_db.Turn_Table),20)),fit_1(s_all_1,temp_x),'-','color',col(i,:));
        
    legend_lbl{i} = [mat2str(rad2deg(skew_bins(i))),' deg'];
end
lgd1 = legend(hdls(:,1),legend_lbl,'Location', 'northoutside', 'Orientation', 'horizontal');
xlabel('Angle of attack [deg]')
ylabel('Wing F_x / V^2 [N s^2 m^{-2}]')
axis([-7 17 -0.05 0.1])
grid on

% Change font size
set(findall(gcf,'-property','FontSize'),'FontSize',font_size) 

fig_name = ['WING_FX_',formattedDateTime,'.eps'];
exportgraphics(fig,fig_name,'BackgroundColor','none','ContentType','vector')
grid on
