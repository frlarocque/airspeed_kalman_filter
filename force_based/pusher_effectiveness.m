clear all

options = optimset('TolFun',1E-5,'TolX',1E-5);

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

test = 'AE';
idx = contains(full_db.Code,test);

test_db = full_db(idx,:);

% Transform all angles in rad
deg_columns = {'Turn_Table','Skew','Skew_sp','Pitch','AoA','std_AoA'};
test_db{:,deg_columns} = deg2rad(test_db{:,deg_columns});

% Save
save(['db_',test,'.mat'],'test_db')

%% Remove entries

test_db = test_db(test_db.Rud==0 & test_db.Elev==0 & test_db.Ail_R==0 & test_db.Ail_L==0 & test_db.Mot_F<1000 &test_db.Mot_R<1000 &test_db.Mot_B<1000 & test_db.Mot_L<1000,:);

%% Plot
figure
subplot(2,1,1)
plot(test_db.Mot_Push,test_db.Fx,'*')
xlabel('Pusher Command [pprz]')
ylabel('F_x [N]')
grid on

subplot(2,1,2)
plot(test_db.Windspeed,test_db.Fx,'*')
xlabel('Wind speed [m/s]')
ylabel('F_x [N]')
grid on

%% Fit on all
% Delete 0 command signals, as propeller is windmilling at higher airspeed
test_db = test_db(test_db.Mot_Push~=0 & test_db.Mot_Push~=9000 & test_db.Windspeed<16,:);


% F = k1*V*pprz+k_2*pprz^2
% k  = [k1 k_2]
% x = [pprz,V]
x = [test_db.Mot_Push,test_db.Windspeed];
y = [test_db.Fx];

fit = @(k,x)  k(1).*x(:,2).*x(:,1)+k(2).*x(:,1).^2;    % Function to fit
fcn = @(k) sqrt(mean((fit(k,x) - y).^2));           % Least-Squares cost function
[s_all,RMS_all] = fminsearch(fcn,[1E-3;1E-3])

windspeed_bins = unique(round(test_db.Windspeed,0));
test_db.Windspeed_bin = round(test_db.Windspeed,0);

figure
legend_lbl = {};
col=linspecer(length(windspeed_bins));
hdls = [];
for i=1:length(windspeed_bins)
    temp_db = test_db(test_db.Windspeed_bin==windspeed_bins(i),:);
    hdls(i) = plot(temp_db.Mot_Push,temp_db.Fx,'*','color',col(i,:));
    hold on
    plot(linspace(min(temp_db.Mot_Push),max(temp_db.Mot_Push),10),s_all(1).*windspeed_bins(i).*linspace(min(temp_db.Mot_Push),max(temp_db.Mot_Push),10)+s_all(2).*linspace(min(temp_db.Mot_Push),max(temp_db.Mot_Push),10).^2,'--','color',col(i,:))
    legend_lbl{i} = [bin_column,' = ',mat2str(windspeed_bins(i))];
end
legend(hdls,legend_lbl,'location','best')

figure
plot3(test_db.Mot_Push,test_db.Windspeed,test_db.Fx,'*')
hold on
pprz = linspace(min(test_db.Mot_Push),max(test_db.Mot_Push),10);
wind = linspace(min(test_db.Windspeed),max(test_db.Windspeed),10);
[PPRZ,WIND] = meshgrid(pprz,wind);
surf(PPRZ,WIND,PPRZ.*WIND.*s_all(1)+PPRZ.^2*s_all(2))
xlabel('Pusher Command [pprz]')
ylabel('Wind speed [m/s]')
zlabel('F_x [N]')
grid on

% Hence:
%
% Fx = -1.8820e-04*pprz [pprz] * v [m/s]+ 5.5833e-07 * pprz^2 [pprz]

%% Fit on zero airspeed
static_db = test_db(test_db.Windspeed<2 & test_db.Windspeed>0,:);

figure
plot(static_db.Mot_Push,static_db.Fx,'*')
xlabel('Pusher Command [pprz]')
ylabel('F_x [N]')
grid on
hold on

% F = k_2*pprz^2
% k  = [k_2]
% x = [pprz,V]
x = [static_db.Mot_Push,static_db.Windspeed];
y = [static_db.Fx];

fit = @(k,x)  k(1).*x(:,1).^2+k(2);    % Function to fit
fcn = @(k) sqrt(mean((fit(k,x) - y).^2));           % Least-Squares cost function
[s_static,RMS_static] = fminsearch(fcn,[1E-3;1E-3])

plot(linspace(min(static_db.Mot_Push),max(static_db.Mot_Push),10),linspace(min(static_db.Mot_Push),max(static_db.Mot_Push),10).^2.*s_static(1))


%% Fit on fixed command
command_db = test_db(test_db.Mot_Push<3500 & test_db.Mot_Push>2500,:);

figure
plot(command_db.Windspeed,command_db.Fx,'*')
xlabel('Wind speed [m/s]')
ylabel('F_x [N]')
grid on
hold on

% F = k1*V*pprz+k_2*pprz^2+k3
% k  = [k1 k_2 k_3]
% x = [pprz,V]
x = [test_db.Mot_Push,test_db.Windspeed];
y = [test_db.Fx];

fit = @(k,x)  k(1).*x(:,2).*x(:,1)+k(2).*x(:,1).^2;    % Function to fit
fcn = @(k) sqrt(mean((fit(k,x) - y).^2));           % Least-Squares cost function
[s_command,RMS_command] = fminsearch(fcn,[1E-3;1E-3])

plot(linspace(min(command_db.Windspeed),max(command_db.Windspeed),10),linspace(min(command_db.Windspeed),max(command_db.Windspeed),10).*mean(command_db.Mot_Push).*s_command(1)+mean(command_db.Mot_Push).^2.*s_command(2))





