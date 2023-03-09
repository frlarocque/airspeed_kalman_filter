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

test = 'ES';
idx = contains(full_db.Code,test);

test_db = full_db(idx,:);

% Transform all angles in rad
deg_columns = {'Turn_Table','Skew','Skew_sp','Pitch','AoA','std_AoA'};
test_db{:,deg_columns} = deg2rad(test_db{:,deg_columns});

% Save
save(['db_',test,'.mat'],'test_db')

%% Remove entries

% Removing entries with non-zero control surfaces (except elev)
test_db = test_db(test_db.Rud==0 & test_db.Ail_R==0 & test_db.Ail_L==0 ,:);
% Removing entries with non-zero pusher motor
test_db = test_db(test_db.Mot_Push==0,:);
% Removing entries with non-zero hover motor command
test_db = test_db(test_db.Mot_F==0 & test_db.Mot_R==0 & test_db.Mot_B==0 & test_db.Mot_L==0,:);
% Removing entries with angle of attack higher than 15 deg
test_db(abs(test_db.Turn_Table)>deg2rad(15),:) = []; 
% Select only skew = 90deg
test_db = test_db(test_db.Skew_sp==deg2rad(90),:);

%% Pprz to elev angle mapping
pprz_mapping = [-96, 1922,2739,3592,4842,5611,6284,7329,8350,9246,9600,48,-961,-1838,-2739,-3605,-4578,-5647,-6752,-7606,-8374,-9167,-9600,0,-100,-500];
elev_mapping = deg2rad([-1.9,-3.67,-4.3,-5.1,-6.2,-6.7,-7.3,-8.3,-9.2,-9.9,-10.3,-2.2,1.9,6.2,10.3,14.7,19.1,23.8,27.8,31,33.1,35.6,36.6,-2.1,-1.7,-0.1]);

% Equivalent to:
pprz2elev_m = polyfit(pprz_mapping(pprz_mapping<0),elev_mapping(pprz_mapping<0),1);
pprz2elev_p = polyfit(pprz_mapping(pprz_mapping>0),elev_mapping(pprz_mapping>0),1);

test_db.elev_angle = interp1(pprz_mapping,elev_mapping,test_db.Elev);

figure
plot(test_db.Elev,rad2deg(test_db.elev_angle),'*')
xlabel('Elevator pprz signal')
ylabel('Elevator Angle [deg]')
title('Relationship between Elevator pprz signal and Elevator angle')
hold on
plot(test_db.Elev(test_db.Elev<0),rad2deg(polyval(pprz2elev_m,test_db.Elev(test_db.Elev<0))))
plot(test_db.Elev(test_db.Elev>0),rad2deg(polyval(pprz2elev_p,test_db.Elev(test_db.Elev>0))))
legend('Data points',sprintf('Angle [rad] = %2.2e pprz + %2.2e',pprz2elev_m),sprintf('Angle [rad] = %2.2e pprz + %2.2e',pprz2elev_p))

%% Add Lift and Drag Entries
% Defining lift as perpendicular to speed vector

% Fz points up instead of down
for i=1:size(test_db,1)
    temp_A = [sin(test_db.elev_angle(i)) -cos(test_db.elev_angle(i));-cos(test_db.elev_angle(i)) -sin(test_db.elev_angle(i))];
    temp_b = [test_db.Fx(i);-test_db.Fz(i)];

    temp_x = inv(temp_A)*temp_b;

    test_db.Lift(i) = temp_x(1);
    test_db.Drag(i) = temp_x(2);
end

%% Obtaining elevator Fx
% Fx = body drag + wing drag + elevator drag
% @ 0 elevator command, elevator creating minimal drag
% offset everything by 0 elevator command drag

windspeed_bins = unique(round(test_db.Windspeed,0));
test_db.Windspeed_bin = round(test_db.Windspeed,0);

for i=1:size(test_db,1)
    offset = interp1(test_db.elev_angle(test_db.Windspeed_bin==test_db.Windspeed_bin(i)),test_db.Fx(test_db.Windspeed_bin==test_db.Windspeed_bin(i)),0);
    test_db.Fx_elev(i) = test_db.Fx(i)-offset;
    test_db.Drag_elev(i) = test_db.Drag(i)-offset;
end

% Calculating std dev of new Fz_pusher
% Mean C = mean A - mean B --> Variance C = Variance A + Variance B - 2*Correlation(A,B)*SD A * SD B
% Assumming uncorrelated
test_db.std_Fx_elev = sqrt(test_db.std_Fx.^2+test_db.std_Fx.^2);

%% Plot Fx

windspeed_bins = unique(round(test_db.Windspeed,0));
test_db.Windspeed_bin = round(test_db.Windspeed,0);

figure
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = test_db(test_db.Windspeed_bin==windspeed_bins(i),:);

    hdls(i) = plot(rad2deg(temp_db.elev_angle),temp_db.Fx_elev,'*','color',col(i,:));
    hold on

    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
end

xlabel('Elevator angle [deg]')
ylabel('F_x [N]')
lgd1 = legend(hdls,legend_lbl,'location','southeast');
title(lgd1,'Airspeed') % add legend title
sgtitle('Fx Drag Data from Wind Tunnel tests')
grid on

%% Select elevator command not stalled
lin_db = test_db(abs(test_db.elev_angle)<deg2rad(25),:);

% Fz = (k1+k2*alpha+k3*alpha^2)*V^2
% k  = [k1 k2]
% x = [AoA,V]
x = [lin_db.elev_angle,lin_db.Windspeed];
y = [lin_db.Fx_elev];

fit_elev = @(k,x)  (k(1)+k(2).*x(:,1)+k(3).*x(:,1).^2).*x(:,2).^2; % Function to fit
fcn_elev = @(k) sqrt(mean((fit_elev(k,x) - y).^2));           % Least-Squares cost function
[s_elev,RMS_elev] = fminsearch(fcn_elev,[0;1E-3;-1E-5],options) %bound first coefficient to negative value

% s_elev =
% 
%   -0.002608468213419
%   -0.030215146812742
%   -0.000271549151544
% 
% 
% RMS_elev =
% 
%    0.512552710574194

figure
windspeed_bins = unique(round(test_db.Windspeed,0));
test_db.Windspeed_bin = round(test_db.Windspeed,0);
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = test_db(test_db.Windspeed_bin==windspeed_bins(i),:);
    temp_x = [linspace(min(lin_db.elev_angle),max(lin_db.elev_angle),10)',ones(10,1).*windspeed_bins(i)];

    if show_error_bar; hdls(i) = errorbar(rad2deg(temp_db.elev_angle),temp_db.Fx_elev,temp_db.std_Fx,'*','color',col(i,:));
    else; hdls(i) = plot(rad2deg(temp_db.elev_angle),temp_db.Fx_elev,'*','color',col(i,:)); end
    
    hold on
    plot(rad2deg(linspace(min(lin_db.elev_angle),max(lin_db.elev_angle),10)),fit_elev(s_elev,temp_x),'--','color',col(i,:))
    legend_lbl{i} = [mat2str(windspeed_bins(i)),' m/s'];
end
lgd1 = legend(hdls,legend_lbl,'location','southwest');
title(lgd1,'Airspeed') % add legend title
title(sprintf('Elevator Fit\nFx = (k1+k2*alpha+k3*alpha^2)*V^2\nK1 = %2.2e K2 = %2.2e K3 = %2.2e  |  RMS = %2.2f',s_elev(1),s_elev(2),s_elev(3),RMS_elev))
xlabel('Elevator angle [deg]')
ylabel('Elevator F_x [N]')
axis([-inf inf -inf inf])
grid on

%Fx = (-2.608468213419211E-3  + -3.021514681274200E-2*alpha+ -2.715491515441531E4*alpha^2)*V^2
