%% 
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
mycolors = linspecer(5,'qualitative');
mylinestyles = {'-', '--', ':'};
mymarkerstyles = {'o','+','*','x','square','diamond','^'};
set(gcf,'DefaultAxesColorOrder',mycolors, ...
        'DefaultAxesLineStyleOrder',mylinestyles)

%%
fault_time = 311.915;
zoom_range = [311.5,313.5];

ax1 = subplot(1,1,1);
s1 = plot(kalman_res{1}.t,kalman_res{1}.z(7,:));
hold on
ax1.LineStyleOrderIndex = ax1.ColorOrderIndex;
s2 = plot(kalman_res{1}.t,kalman_res{1}.x(1,:));
xlim([kalman_res{1}.t(1)-3 kalman_res{1}.t(1)+75])
ylim([0 25])
xlabel('Time [s]','fontSize',font_size)
ylabel('Airspeed [m/s]','fontSize',font_size)
grid on
ax2=gca;
ax2.FontSize = font_size; 

% tail
[xt yt] = ds2nfu(zoom_range(1)-2, 10);
% head
[xh yh] = ds2nfu(zoom_range(1), kalman_res{1}.z(7,find(kalman_res{1}.t>=fault_time,1,'first')));
annotation('textarrow',[xt xh],[yt yh],'String','Fault','fontsize',font_size)

[p, z, d] = zoomPlot(kalman_res{1}.t,kalman_res{1}.z(7,:), zoom_range, [0.368 0.565 0.53 0.342],[1 3]);
xticks(linspace(zoom_range(1),zoom_range(2),9))
ylim([-3 23])
hold on
grid on
%grid minor
ax3=gca;
ax3.FontSize = font_size-4;

f = xline(fault_time,'-',{'Fault'},'linewidth',1.5*line_width,'color',mycolors(3,:),'LabelHorizontalAlignment','left','LabelVerticalAlignment','top','fontsize',font_size-4);

% % tail
% [xt yt] = ds2nfu(fault_time-0.08, 5);
% % head
% [xh yh] = ds2nfu(fault_time, kalman_res{1}.z(7,find(kalman_res{1}.t>=fault_time,1,'first')));
% annotation('textarrow',[xt xh],[yt yh],'String','Fault Start','fontsize',font_size-8)

detection_diff = kalman_res{1}.t(find(kalman_res{1}.pitot_fault_detector.flag(3,:)==1,1,'first'));
d2 = xline(detection_diff,':',{'Criteria Deriv.'},'linewidth',1.5*line_width,'color',mycolors(5,:),'LabelHorizontalAlignment','right','LabelVerticalAlignment','top','fontsize',font_size-4);

% % tail
% [xt yt] = ds2nfu(detection_diff+0.2, 20);
% % head
% [xh yh] = ds2nfu(detection_diff, kalman_res{1}.z(7,find(kalman_res{1}.t>=detection_diff,1,'first')));
% annotation('textarrow',[xt xh],[yt yh],'String','Detection by derivative','fontsize',font_size-8)

detection_high = kalman_res{1}.t(find(kalman_res{1}.pitot_fault_detector.flag(2,:)==1,1,'first'));
d1 = xline(detection_high,'--',{'Criteria Norm.'},'linewidth',1.5*line_width,'color',mycolors(4,:),'LabelHorizontalAlignment','right','LabelVerticalAlignment','top','fontsize',font_size-4);

% % tail
% [xt yt] = ds2nfu(detection_high+0.5, 2.5);
% % head
% [xh yh] = ds2nfu(detection_high, kalman_res{1}.z(7,find(kalman_res{1}.t>=detection_high,1,'first')));
% annotation('textarrow',[xt xh],[yt yh],'String','Detection by high','fontsize',font_size-8)

%ax4=axes('position',get(ax2,'position'),'visible','off');
%legend(ax4,[f,d1,d2],{'Fault','Detection Norm.','Detection Deriv.'},'fontSize',font_size,'Location', 'northwest', 'Orientation', 'vertical');    
ax5=axes('position',get(ax2,'position'),'visible','off');
legend(ax5,[s1,s2],{'Pitot Tube','Estimation'},'fontSize',font_size,'Location', 'southeast', 'Orientation', 'vertical')

%legend(ax4,[s1,s2,f,d1,d2],{'Pitot Tube','Estimation','Fault','Detection High','Detection Diff'},'fontSize',font_size-4,'Location', 'northwest', 'Orientation', 'vertical');    

% Change font size
%set(findall(gcf,'-property','FontSize'),'FontSize',font_size)

% Export
fig_name = ['abrupt_pitot_failure_',formattedDateTime,'.eps'];
exportgraphics(fig,fig_name,'BackgroundColor','none','ContentType','vector')
