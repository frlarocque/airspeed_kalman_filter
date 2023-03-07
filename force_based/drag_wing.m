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

test = 'LP5';
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
% Fx = body drag + wing drag
% Wing drag = Fx-body drag
%Drag without hover props (only body drag)

%drag_body_coeff = [0                     -3.0061458379662E-2   3.497371379286E-3    1.46865971330522E-1];%all airspeeds without hover props without skew
drag_body_coeff = [-9.013405108486905E-3 -1.988035608425628E-2 9.850048188379294E-4 1.443975207474472E-1]; %all airspeeds without hover props with skew

Fx_body = @(alpha,skew,V) (drag_body_coeff(1)  .*  cos(skew)+...
                              drag_body_coeff(2)+...
                              drag_body_coeff(3)  .*  alpha+...
                              drag_body_coeff(4)  .*  alpha.^2).*V.^2;

test_db.Fx_wing = test_db.Fx-Fx_body(test_db.Turn_Table,test_db.Skew_sp,test_db.Windspeed);

% Calculating std dev of new Fx_pusher
% Mean C = mean A - mean B --> Variance C = Variance A + Variance B - 2*Correlation(A,B)*SD A * SD B
% Assumming uncorrelated
test_db.std_Fx_wing = sqrt(test_db.std_Fx.^2+test_db.std_Fx.^2);

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
%   -0.009110099494277
%   -0.009251650698286
%   -0.088837372634962
% 
% 
% RMS_quad =
% 
%    0.236176924284178

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
%   -0.008130675202344
%    0.144941457970910
%    0.688716833250097
% 
% 
% RMS_ff =
% 
%    0.346045664770645

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
% Fx1 = (k1   +         (k2*alpha+k3*alpha^2))*(sin(skew)^2+k4)*V^2
% Fx2 = (k1*(1+k5*skew)+(k2*alpha+k3*alpha^2))*(sin(skew)^2+k4)*V^2
% Fx3 = ((k1+k2*alpha+k3*alpha^2)             *(sin(skew)^2+k4)*V^2
% Fx4 = ((k1+k2*alpha+k3*alpha^2)             *(sin(skew)^2)+k4)*V^2
% Fx5 = ((k1+k2*alpha+k3*alpha^2)             *(sin(skew)^2+k4)+k5)*V^2

fit_1 = @(k,x)  x(:,2).^2.*(k(1)+ (k(2).*x(:,1)+k(3).*x(:,1).^2) .* (sin(x(:,3)).^2+k(4)) ); % Function to fit

fit_2 = @(k,x)  x(:,2).^2.*(k(1).*(1+k(5).*x(:,3))+ (k(2).*x(:,1)+k(3).*x(:,1).^2).*(sin(x(:,3)).^2+k(4))); % Function to fit

fit_3 = @(k,x)  x(:,2).^2.*( (k(1)+k(2).*x(:,1)+k(3).*x(:,1).^2) .* (sin(x(:,3)).^2+k(4)) ); % Function to fit

fit_4 = @(k,x)  x(:,2).^2.*( (k(1)+k(2).*x(:,1)+k(3).*x(:,1).^2) .* (sin(x(:,3)).^2) + k(4) ); % Function to fit

fit_5 = @(k,x)  x(:,2).^2.*( (k(1)+k(2).*x(:,1)+k(3).*x(:,1).^2) .* (sin(x(:,3)).^2+k(4)) + k(5) ); % Function to fit

%% Wing Drag fixed airspeed, different skews
skew_db = test_db(test_db.Windspeed<12 & test_db.Windspeed>9,:);

% k  = [k1 k2 k3 k4]
% x = [AoA,V,skew]
x = [skew_db.Turn_Table,skew_db.Windspeed,skew_db.Skew_sp];
y = [skew_db.Fx_wing];

fcn_1 = @(k) sqrt(mean((fit_1(k,x) - y).^2));           % Least-Squares cost function
[s_skew_1,RMS_skew_1] = fminsearch(fcn_1,[0;0;0;-4E-2],options) 

fcn_2 = @(k) sqrt(mean((fit_2(k,x) - y).^2));           % Least-Squares cost function
[s_skew_2,RMS_skew_2] = fminsearch(fcn_2,[0;0;0;-4E-2;0],options) 

fcn_3 = @(k) sqrt(mean((fit_3(k,x) - y).^2));           % Least-Squares cost function
[s_skew_3,RMS_skew_3] = fminsearch(fcn_3,[-2E-2;1E-1;8E-1;1E-1],options) 

fcn_4 = @(k) sqrt(mean((fit_4(k,x) - y).^2));           % Least-Squares cost function
[s_skew_4,RMS_skew_4] = fminsearch(fcn_4,[0;0;0;0;0],options) 

fcn_5 = @(k) sqrt(mean((fit_5(k,x) - y).^2));           % Least-Squares cost function
[s_skew_5,RMS_skew_5] = fminsearch(fcn_5,[0;0;0;0;0],options) 

% s_skew_1 =
% 
%   -0.007875783089360
%    0.154235238409942
%    0.697527217260195
%   -0.041059621726089
% 
% 
% RMS_skew_1 =
% 
%    0.287411382565726
%  
% 
% s_skew_2 =
% 
%   -0.007742841697518
%    0.154451091730156
%    0.704154476765070
%   -0.043380923100371
%    0.026848472271382
% 
% 
% RMS_skew_2 =
% 
%    0.287243643699243
% 
% 
% s_skew_3 =
% 
%   -0.010766680789344
%    0.171775260391515
%    0.788215220127712
%   -0.129144950138309
% 
% 
% RMS_skew_3 =
% 
%    0.593448868446247
% 
% 
% s_skew_4 =
% 
%    0.000895348298053
%    0.145783480638893
%    0.668221743019729
%   -0.008776655411892
%   -0.056951442846835
% 
% 
% RMS_skew_4 =
% 
%    0.306290533040551
% 
% 
% s_skew_5 =
% 
%   -0.000643702348094
%    0.154656160180710
%    0.709170597030569
%   -0.045203821273234
%   -0.007597224815211
% 
% 
% RMS_skew_5 =
% 
%    0.286558622370498

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
RMS_all_1 = sqrt(mean((fit_1(s_skew_1,x) - y).^2))
RMS_all_2 = sqrt(mean((fit_2(s_skew_2,x) - y).^2))
RMS_all_3 = sqrt(mean((fit_3(s_skew_3,x) - y).^2))
RMS_all_4 = sqrt(mean((fit_4(s_skew_4,x) - y).^2))
RMS_all_5 = sqrt(mean((fit_5(s_skew_5,x) - y).^2))

% RMS_all_1 =
% 
%    0.404281197138023
% 
% 
% RMS_all_2 =
% 
%    0.405021974269689
% 
% 
% RMS_all_3 =
% 
%    0.603419270934680
% 
% 
% RMS_all_4 =
% 
%    0.426429216764415
% 
% 
% RMS_all_5 =
% 
%    0.403999048291677

% RMS on ff flight mode and quad mode
verif_db = test_db(test_db.Skew_sp==deg2rad(0) | test_db.Skew_sp==deg2rad(90),:);

x = [verif_db.Turn_Table,verif_db.Windspeed,verif_db.Skew_sp];
y = [verif_db.Fx_wing];
RMS_quad_ff_1 = sqrt(mean((fit_1(s_skew_1,x) - y).^2))
RMS_quad_ff_2 = sqrt(mean((fit_2(s_skew_2,x) - y).^2))
RMS_quad_ff_3 = sqrt(mean((fit_3(s_skew_3,x) - y).^2))
RMS_quad_ff_4 = sqrt(mean((fit_4(s_skew_4,x) - y).^2))
RMS_quad_ff_5 = sqrt(mean((fit_5(s_skew_5,x) - y).^2))

% RMS_quad_ff_1 =
% 
%    0.400126297257944
% 
% 
% RMS_quad_ff_2 =
% 
%    0.400646150831059
% 
% 
% RMS_quad_ff_3 =
% 
%    0.851182944020033
% 
% 
% RMS_quad_ff_4 =
% 
%    0.461000129894547
% 
% 
% RMS_quad_ff_5 =
% 
%    0.404946329442834

%% Fit on all Airspeed
% Fx1 = (k1   +         (k2*alpha+k3*alpha^2))*(sin(skew)^2+k4)*V^2
% Fx2 = (k1*(1+k5*skew)+(k2*alpha+k3*alpha^2))*(sin(skew)^2+k4)*V^2
% Fx3 = ((k1+k2*alpha+k3*alpha^2)             *(sin(skew)^2+k4)*V^2
% Fx4 = ((k1+k2*alpha+k3*alpha^2)             *(sin(skew)^2)+k4)*V^2
% Fx5 = ((k1+k2*alpha+k3*alpha^2)             *(sin(skew)^2+k4)+k5)*V^2

% k  = [k1 k2 k3 k4]
% x = [AoA,V,skew]
x = [test_db.Turn_Table,test_db.Windspeed,test_db.Skew_sp];
y = [test_db.Fx_wing];

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

% s_all_1 =
% 
%   -0.006849932988326
%    0.166581715295916
%    0.683731535328429
%   -0.053784863542344
% 
% 
% RMS_all_1 =
% 
%    0.380000688060119
% 
% 
% s_all_2 =
% 
%   -0.006917488880644
%    0.166507674359643
%    0.681474926113354
%   -0.053031841802562
%   -0.012978679888761
% 
% 
% RMS_all_2 =
% 
%    0.379976439000814
% 
% 
% s_all_3 =
% 
%   -0.009356750427738
%    0.182035182809969
%    0.768334761461139
%   -0.131578702491019
% 
% 
% RMS_all_3 =
% 
%    0.591305496586931
% 
% 
% s_all_4 =
% 
%    0.001531774160445
%    0.155564274318801
%    0.638424981003179
%   -0.008177110786727
%   -1.082748830072914
% 
% 
% RMS_all_4 =
% 
%    0.404411023401697
% 
% 
% s_all_5 =
% 
%   -0.000462183445575
%    0.166850348800852
%    0.692271415096719
%   -0.056595229598389
%   -0.006655831273171
% 
% 
% RMS_all_5 =
% 
%    0.379671667788112

% RMS on ff flight mode and quad mode
verif_db = test_db(test_db.Skew_sp==deg2rad(0) | test_db.Skew_sp==deg2rad(90),:);

x = [verif_db.Turn_Table,verif_db.Windspeed,verif_db.Skew_sp];
y = [verif_db.Fx_wing];
RMS_quad_ff_1 = sqrt(mean((fit_1(s_all_1,x) - y).^2))
RMS_quad_ff_2 = sqrt(mean((fit_2(s_all_2,x) - y).^2))
RMS_quad_ff_3 = sqrt(mean((fit_3(s_all_3,x) - y).^2))
RMS_quad_ff_4 = sqrt(mean((fit_4(s_all_4,x) - y).^2))
RMS_quad_ff_5 = sqrt(mean((fit_5(s_all_5,x) - y).^2))

% RMS_quad_ff_1 =
% 
%    0.445554691403407
% 
% 
% RMS_quad_ff_2 =
% 
%    0.444651219104435
% 
% 
% RMS_quad_ff_3 =
% 
%    0.835422310736343
% 
% 
% RMS_quad_ff_4 =
% 
%    0.504019176279786
% 
% 
% RMS_quad_ff_5 =
% 
%    0.448555046214412

