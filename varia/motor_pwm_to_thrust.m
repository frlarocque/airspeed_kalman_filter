clc
close all
clearvars
%%
% Folder in which all test data folders are located
[file,path] = uigetfile('Select a file');

rctb = RCTestbenchData(fullfile(path,file));

%% Plot values
pwm_values = unique(rctb.pwm);
pwm_index = zeros(1,length(pwm_values));
for i=1:length(pwm_values)
    pwm_index(i) = find(rctb.pwm==pwm_values(i), 1, 'last');
end

Xname = 'pwm';
Yname = 'T';
rctb.plt(Xname, Yname)
hold on
plot(rctb.pwm(pwm_index),rctb.T(pwm_index),'*')

%% Polyfit
P = polyfit(rctb.pwm(pwm_index(1:end)),rctb.T(pwm_index(1:end)),1)
plot(rctb.pwm(pwm_index),polyval(P,rctb.pwm(pwm_index)))
legend('Test Data','Selected Data Points','Fit')
