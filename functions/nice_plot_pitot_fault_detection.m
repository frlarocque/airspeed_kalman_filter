function nice_plot_pitot_fault_detection(airspeed_pitot,kalman_res,res_fault_detector,AR)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

set(gcf, 'Renderer', 'Painters');

size = 1000;

fig_height = size;
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

%% General Figure Setup
fig = figure('position',[0 0 fig_width fig_height]);

% Store the default line width value
origLineWidth = get(groot, 'DefaultLineLineWidth');

% Set a new default line width value
set(groot, 'DefaultLineLineWidth', 2);

% Set colors and line styles
mycolors = linspecer(3,'qualitative');
mylinestyles = {'-', '--', ':'};
set(gcf,'DefaultAxesColorOrder',mycolors, ...
        'DefaultAxesLineStyleOrder',mylinestyles)

% First figure: Airspeed
ax1 = subplot(2,1,1);

hold on
s1 = plot(kalman_res.t,kalman_res.x(1,:));
ax1.LineStyleOrderIndex = ax1.ColorOrderIndex;

filter_freq = 0.4;
[b,a] = butter(2,2*filter_freq*mean(diff(kalman_res.t)),'low');
s2 = plot(airspeed_pitot.time,filtfilt(b,a,airspeed_pitot.data));
%title('Airspeed')
xlabel('Time [s]')
ylabel('Speed [m/s]')
grid on
grid minor
legend([s1,s2],'Estimation','Pitot Tube','Orientation','horizontal')
axis([-inf inf 0 1.2.*max(airspeed_pitot.data)])

% Export figure
if strcmp(subplot_config,'one')
   fig_name = [file_ident,'_airspeed_',formattedDateTime,'.eps'];
   exportgraphics(fig,fig_name,'BackgroundColor','none','ContentType','vector')
end

end