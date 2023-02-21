%% Init
clear all
clc

%% Load files
% Without wing, without hover props
load('db_LP4.mat')
test_db = test_db(test_db.Mot_F<1000,:); %Remove tests with propellers
no_props_db = test_db;
% Without wing
load('db_LP3.mat')
test_db = test_db(test_db.Mot_F<1000,:); %Remove tests with propellers
props_db = test_db;


%% Plot difference

% Possible pitch angles
pitch_choices = unique(no_props_db.Turn_Table)
skew_choices = unique(no_props_db.Skew_sp)

selected_pitch = pitch_choices(6);
selected_skew = skew_choices(end);

figure
plot(no_props_db(no_props_db.Turn_Table==selected_pitch & no_props_db.Skew_sp==selected_skew,:).Windspeed,no_props_db(no_props_db.Turn_Table==selected_pitch & no_props_db.Skew_sp==selected_skew,:).Fz,'*')
hold on
plot(props_db(props_db.Turn_Table==selected_pitch & props_db.Skew_sp==selected_skew,:).Windspeed,props_db(props_db.Turn_Table==selected_pitch & props_db.Skew_sp==selected_skew,:).Fz,'*')
xlabel('Airspeed (m/s)')
ylabel('Lift Force (N)')
legend('Without props','With props')
grid on


