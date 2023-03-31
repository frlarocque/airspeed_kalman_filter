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

%% Obtaining wing Fz
test_db.Fz_wing = test_db.Fz;

save('db_Fz_wing.mat','test_db')
%% Plot Fz wing
windspeed_bins = unique(round(test_db.Windspeed,0));
test_db.Windspeed_bin = round(test_db.Windspeed,0);

figure
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = test_db(test_db.Windspeed_bin==windspeed_bins(i),:);

    subplot(2,2,1)
    hdls(1,i) = plot(rad2deg(temp_db.Turn_Table),temp_db.Fz_wing,'*','color',col(i,:));
    hold on

    subplot(2,2,2)
    hdls(2,i) = plot(rad2deg(temp_db.Skew_sp),temp_db.Fz_wing,'*','color',col(i,:));
    hold on

    subplot(2,2,3)
    hdls(3,i) = plot(rad2deg(temp_db.Turn_Table),temp_db.Fz_wing./temp_db.Windspeed.^2,'*','color',col(i,:));
    hold on

    subplot(2,2,4)
    hdls(4,i) = plot(rad2deg(temp_db.Skew_sp),temp_db.Fz_wing./temp_db.Windspeed.^2,'*','color',col(i,:));
    hold on

    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
end

subplot(2,2,1)
xlabel('Turn table angle (angle of attack) [deg]')
ylabel('Wing F_z [N]')
%lgd1 = legend(hdls(1,:),legend_lbl,'location','southeast');
%title(lgd1,'Airspeed') % add legend title
subplot(2,2,2)
xlabel('Skew Setpoint [deg]')
ylabel('Wing F_z [N]')
%lgd1 = legend(hdls(2,:),legend_lbl,'location','southeast');
%title(lgd1,'Airspeed') % add legend title
subplot(2,2,3)
xlabel('Turn table angle (angle of attack) [deg]')
ylabel('Wing F_z/Airspeed^2 [N/((m/s)^2)]')
%lgd1 = legend(hdls(3,:),legend_lbl,'location','southeast');
%title(lgd1,'Airspeed') % add legend title
grid on
subplot(2,2,4)
xlabel('Skew Setpoint [deg]')
ylabel('Wing F_z/Airspeed^2 [N/((m/s)^2)]')
lgd1 = legend(hdls(4,:),legend_lbl,'location','southeast');
title(lgd1,'Airspeed') % add legend title
sgtitle('Wing Fz Drag Data from Wind Tunnel tests')

%% Wing Fz Lift in quad mode
quad_db = test_db(test_db.Skew_sp==deg2rad(0),:);

% Fz = (k1+k2*alpha+k3*alpha^2)*V^2
% k  = [k1 k2 k3 k4]
% x = [AoA,V]
x = [quad_db.Turn_Table,quad_db.Windspeed];
y = [quad_db.Fz_wing];

fit_quad = @(k,x)  (k(1)+k(2).*x(:,1)+k(3).*x(:,1).^2).*x(:,2).^2; % Function to fit
fcn_quad = @(k) sqrt(mean((fit_quad(k,x) - y).^2));           % Least-Squares cost function
[s_quad,RMS_quad] = fminsearch(fcn_quad,[0;0;0],options) 

% s_quad =
% 
%   -0.005351242978305
%   -0.062624686872132
%   -0.760848309199772
% 
% 
% RMS_quad =
% 
%    0.259777761115678

figure
windspeed_bins = unique(round(quad_db.Windspeed,0));
quad_db.Windspeed_bin = round(quad_db.Windspeed,0);
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = quad_db(quad_db.Windspeed_bin==windspeed_bins(i),:);
    temp_x = [linspace(min(temp_db.Turn_Table),max(temp_db.Turn_Table),10)',ones(10,1).*windspeed_bins(i)];

    if show_error_bar; hdls(i) = errorbar(rad2deg(temp_db.Turn_Table),temp_db.Fz_wing,temp_db.std_Fz,'*','color',col(i,:));
    else; hdls(i) = plot(rad2deg(temp_db.Turn_Table),temp_db.Fz_wing,'*','color',col(i,:)); end
    
    hold on
    plot(rad2deg(linspace(min(temp_db.Turn_Table),max(temp_db.Turn_Table),10)),fit_quad(s_quad,temp_x),'--','color',col(i,:))
    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
end
lgd1 = legend(hdls,legend_lbl,'location','southwest');
title(lgd1,'Airspeed') % add legend title
title(sprintf('Wing Fz Lift Quad Mode\nFz = (k1+k2*alpha+k3*alpha^2)*V^2\nK1 = %2.2e K2 = %2.2e K3 = %2.2e |  RMS = %2.2f',s_quad(1),s_quad(2),s_quad(3),RMS_quad))
xlabel('Turn table angle (angle of attack) [deg]')
ylabel('Wing Lift F_z [N]')
axis([-inf inf -inf inf])
grid on

%% Wing Fz Lift in fordward flight mode
ff_db = test_db(test_db.Skew_sp==deg2rad(90),:);

% Fx = (k1+k2*alpha+k3*alpha^2)*V^2
% k  = [k1 k2 k3 k4]
% x = [AoA,V]
x = [ff_db.Turn_Table,ff_db.Windspeed];
y = [ff_db.Fz_wing];

fit_ff = @(k,x)  (k(1)+k(2).*x(:,1)+k(3).*x(:,1).^2).*x(:,2).^2; % Function to fit
fcn_ff = @(k) sqrt(mean((fit_ff(k,x) - y).^2));           % Least-Squares cost function
[s_ff,RMS_ff] = fminsearch(fcn_ff,[-3E-2;1E1;0],options) 

% s_ff =
% 
%   -0.133058953887821
%   -0.974792873867665
%    0.002072362592607
% 
% 
% RMS_ff =
% 
%    0.574136641510675

figure
windspeed_bins = unique(round(ff_db.Windspeed,0));
ff_db.Windspeed_bin = round(ff_db.Windspeed,0);
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = ff_db(ff_db.Windspeed_bin==windspeed_bins(i),:);
    temp_x = [linspace(min(temp_db.Turn_Table),max(temp_db.Turn_Table),10)',ones(10,1).*windspeed_bins(i)];

    if show_error_bar; hdls(i) = errorbar(rad2deg(temp_db.Turn_Table),temp_db.Fz_wing,temp_db.std_Fz,'*','color',col(i,:));
    else; hdls(i) = plot(rad2deg(temp_db.Turn_Table),temp_db.Fz_wing,'*','color',col(i,:)); end
    
    hold on
    plot(rad2deg(linspace(min(temp_db.Turn_Table),max(temp_db.Turn_Table),10)),fit_ff(s_ff,temp_x),'--','color',col(i,:))
    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
end
lgd1 = legend(hdls,legend_lbl,'location','southwest');
title(lgd1,'Airspeed') % add legend title
title(sprintf('Forward Flight\nFz = (k1+k2*alpha+k3*alpha^2)*V^2\nK1 = %2.2e K2 = %2.2e K3 = %2.2e |  RMS = %2.2f',s_ff(1),s_ff(2),s_ff(3),RMS_ff))
xlabel('Turn table angle (angle of attack) [deg]')
ylabel('Wing Lift F_z [N]')
axis([-inf inf -inf inf])
grid on

%% Best Fit different skew angles

% Different skew angles are fitted and the gains are listed below
angles = deg2rad([90 75 60 45 30 15 0]);
k1 = [-1.33E-1 -1.11E-1 -9.38E-2 -7.02E-2 -4.38E-2 -1.83E-2 -5.35E-3]; %constant
k2 = [-9.75 -9.64 -8.87 -7.72 -4.67 -2.06 -0.626]*1E-1; %alpha
k3 = [2.07E-3 5.06E-3 4.4438E-1 3.85E-1 1.69E-3 -6.28E-1 -7.61E-1]; %alpha^2

figure
subplot(1,3,1)
plot(rad2deg(angles),k1,'*')
hold on
plot(rad2deg([0:pi/40:pi/2]),sin([0:pi/40:pi/2]).^2*min(k1))
legend('Best Fit','sin(skew)^2*K')
xlabel('Skew Angle [deg]')
ylabel('K1')

subplot(1,3,2)
plot(rad2deg(angles),k2,'*')
hold on
plot(rad2deg([0:pi/40:pi/2]),sin([0:pi/40:pi/2]).^2.*(min(k2)-max(k2))+max(k2))
legend('Best Fit','K*sin(skew)^2+K')
xlabel('Skew Angle [deg]')
ylabel('K2')

subplot(1,3,3)
plot(rad2deg(angles),k3,'*')
hold on
plot(rad2deg([0:pi/40:pi/2]),min(k3)+sin([0:pi/40:pi/2]).^2.*-min(k3))
legend('Best Fit','K*sin(skew)^2+K')
xlabel('Skew Angle [deg]')
ylabel('K3')

sgtitle('Variation of best gain for different skew angles')

%% Defining fit functions
% Fx0 = ((k1+k2*alpha+k3*alpha^2)*(k4*sin(skew)^2+k5)*V^2
% Fx1 = (k1*sin(skew)+k1*k4 +alpha*(k2*sin(skew)^2+k2*k4) +alpha^2*(k3*sin(skew)^2+k3*k4))*V^2 = ((k1+k2*alpha+k3*alpha^2)*(sin(skew)^2+k4)*V^2
% Fx2 = (k1*sin(skew)^2     +alpha*(k2*sin(skew)^2+k3)    +alpha^2*k4                    )*V^2
% Fx3 = (k1*sin(skew)^2     +alpha*(k2*sin(skew)^2+k3)    +alpha^2*(k4*sin(skew)^2+k5)   )*V^2
% Fx4 = (k1*sin(skew)^2     +alpha*(k2*sin(skew)^2+k3)    +alpha^2*(k4*sin(skew)  +k5)   )*V^2
% Fx5 = (k1*sin(skew)^2     +alpha*(k2*sin(skew)  +k3)    +alpha^2*(k4*sin(skew)  +k5)   )*V^2

fit_0 = @(k,x)  x(:,2).^2.*((k(1)+ k(2).*x(:,1)+k(3).*x(:,1).^2) .* (k(4)*sin(x(:,3)).^2+k(5)) ); % Function to fit

fit_1 = @(k,x)  x(:,2).^2.*((k(1)+ k(2).*x(:,1)+k(3).*x(:,1).^2) .* (sin(x(:,3)).^2+k(4)) ); % Function to fit

fit_2 = @(k,x)  x(:,2).^2.*(k(1).*sin(x(:,3)).^2 + x(:,1).*(k(2).*sin(x(:,3)).^2+k(3))+k(4).*x(:,1).^2); % Function to fit

fit_3 = @(k,x)  x(:,2).^2.*(k(1).*sin(x(:,3)).^2 + x(:,1).*(k(2).*sin(x(:,3)).^2+k(3))+x(:,1).^2.*(k(4).*sin(x(:,3)).^2+k(5))); % Function to fit

fit_4 = @(k,x)  x(:,2).^2.*(k(1).*sin(x(:,3)).^2 + x(:,1).*(k(2).*sin(x(:,3)).^2+k(3))+x(:,1).^2.*(k(4).*sin(x(:,3))+k(5))); % Function to fit

fit_5 = @(k,x)  x(:,2).^2.*(k(1).*sin(x(:,3)).^2+x(:,1).*(k(2).*sin(x(:,3))+k(3))+x(:,1).^2.*(k(4).*sin(x(:,3)).^2+k(5))); % Function to fit

%% Fit on all Airspeed

% k  = [k1 k2 k3 k4]
% x = [AoA,V,skew]
x = [test_db.Turn_Table,test_db.Windspeed,test_db.Skew_sp];
y = [test_db.Fz_wing];

fcn_0 = @(k) sqrt(mean((fit_0(k,x) - y).^2));           % Least-Squares cost function
[s_all_0,RMS_all_0,~,~] = fminsearchbnd(fcn_0,[-6E-2;-1E0;2E-1;1;5E-1],[-inf -inf -inf 0.25 -inf],[inf inf inf 1.75 inf],options)

% s_all_0 =
% 
%   -0.088598603570368
%   -0.769895263990865
%    0.129061287269501
%    1.129567235352637
%    0.246854021808693
% 
% 
% RMS_all_0 =
% 
%    1.227204499661779

fcn_1 = @(k) sqrt(mean((fit_1(k,x) - y).^2));           % Least-Squares cost function
[s_all_1,RMS_all_1] = fminsearch(fcn_1,[-0.1;-0.8;0.8;0],options)

% s_all_1 =
% 
%   -0.100077872757405
%   -0.869647996437125
%    0.145783145637766
%    0.218539487824641
% 
% 
% RMS_all_1 =
% 
%    1.227204499687460

fcn_2 = @(k) sqrt(mean((fit_2(k,x) - y).^2));           % Least-Squares cost function
[s_all_2,RMS_all_2] = fminsearch(fcn_2,[-0.1;-0.8;0.7;-0.6],options) 

% s_all_2 =
% 
%   -0.125708085064488
%   -0.711254893378571
%   -0.254762016195191
%   -0.190315739294444
% 
% 
% RMS_all_2 =
% 
%    1.337347388746325

fcn_3 = @(k) sqrt(mean((fit_3(k,x) - y).^2));           % Least-Squares cost function
[s_all_3,RMS_all_3] = fminsearch(fcn_3,[-0.1;-0.8;0.7;0.6;-0.6],options) 

% s_all_3 =
% 
%   -0.129845590798654
%   -0.918740289699189
%   -0.154156533824267
%    1.184349238090133
%   -0.723954204896201
% 
% 
% RMS_all_3 =
% 
%    1.122757273818692

fcn_4 = @(k) sqrt(mean((fit_4(k,x) - y).^2));           % Least-Squares cost function
[s_all_4,RMS_all_4] = fminsearch(fcn_4,[-0.1;-0.8;0.7;0.6;-0.6],options) 

% s_all_4 =
% 
%   -0.128026468497919
%   -0.827500133975459
%   -0.198396099544368
%    0.739095225415133
%   -0.611312529183582
% 
% 
% RMS_all_4 =
% 
%    1.251055890465767

fcn_5 = @(k) sqrt(mean((fit_5(k,x) - y).^2));           % Least-Squares cost function
[s_all_5,RMS_all_5] = fminsearch(fcn_5,[-0.1;-0.8;0.7;0.6;-0.6],options) 

% s_all_5 =
% 
%   -0.129199158031542
%   -0.859057657321430
%   -0.085466879762789
%    0.559106759298114
%   -0.421036286758122
% 
% 
% RMS_all_5 =
% 
%    1.349486437524914


% RMS without low skew angle
verif_db = test_db(test_db.Skew_sp>=deg2rad(45),:);

x = [verif_db.Turn_Table,verif_db.Windspeed,verif_db.Skew_sp];
y = [verif_db.Fz_wing];
RMS_0 = sqrt(mean((fit_0(s_all_0,x) - y).^2))
RMS_1 = sqrt(mean((fit_1(s_all_1,x) - y).^2))
RMS_2 = sqrt(mean((fit_2(s_all_2,x) - y).^2))
RMS_3 = sqrt(mean((fit_3(s_all_3,x) - y).^2))
RMS_4 = sqrt(mean((fit_4(s_all_4,x) - y).^2))
RMS_5 = sqrt(mean((fit_5(s_all_5,x) - y).^2))

% RMS_0 =
%    1.035394983771238
% RMS_1 =
%    1.035395833187440
% RMS_2 =
%    1.098208295671669
% RMS_3 =
%    1.000523894611924
% RMS_4 =
%    0.988966592638393
% RMS_5 =
%    1.287629604297610

%% Verification of fit

figure
windspeed_bins = unique(round(test_db.Windspeed,0));
test_db.Windspeed_bin = round(test_db.Windspeed,0);
skew_bins = deg2rad(unique(round(rad2deg(test_db.Skew_sp),0)));
test_db.skew_bin = deg2rad(round(rad2deg(test_db.Skew_sp),0));
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    for j=1:2:length(skew_bins)
        subplot(2,2,(j+1)/2)
        temp_db = test_db(test_db.skew_bin==skew_bins(j) & test_db.Windspeed_bin==windspeed_bins(i),:);
        temp_x = [linspace(min(temp_db.Turn_Table),max(temp_db.Turn_Table),20)',ones(20,1).*windspeed_bins(i),ones(20,1).*skew_bins(j)];
    
        if show_error_bar; hdls(i) = errorbar(rad2deg(temp_db.Turn_Table),temp_db.Fz_wing,temp_db.std_Fz,'*','color',col(i,:));
        else; hdls(i) = plot(rad2deg(temp_db.Turn_Table),temp_db.Fz_wing,'*','color',col(i,:)); end
        
        hold on
        plot(rad2deg(linspace(min(temp_db.Turn_Table),max(temp_db.Turn_Table),20)),fit_1(s_all_1,temp_x),'--','color',col(i,:))
        
        xlabel('Turn table angle (angle of attack) [deg]')
        ylabel('Wing Lift F_z [N]')
        axis([-inf inf min(test_db.Fz_wing) max(test_db.Fz_wing)])
        grid on
        title(sprintf('Skew Angle %2.0f deg',rad2deg(skew_bins(j))))

    end
    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
end
    
title(lgd1,'Airspeed') % add legend title
sgtitle(sprintf('Fz = ((k1+k2*alpha+k3*alpha^2)*(sin(skew)^2+k4)*V^2\nK1 = %2.2e K2 = %2.2e K3 = %2.2e K4 = %2.2e  |  RMS = %2.2f',s_all_1(1),s_all_1(2),s_all_1(3),s_all_1(4),RMS_all_1))

%% Analyze result of fit
RMS = sqrt(mean((fit_1(s_all_1,x) - y).^2))
range = max(y)-min(y)
RMS_percentage_range = RMS./range
max_error = max(abs((fit_1(s_all_1,x) - y)))
max_error_percentage_range = max_error./range