function nice_airspeed_estimation_plot(kalman_res,airspeed_pitot,subplot_config,AR,WT,font_size)
set(gcf, 'Renderer', 'Painters');
% Default to four subplot
if nargin<3
    subplot_config = 'four';
end
if nargin<4
    AR = 1;
end
if nargin<5
    WT = 0;
end
if nargin<6
    font_size = 16;
end

if WT
    file_ident = 'WT';
else
    file_ident = 'OD';
end

% Set figure size and aspect ratio
if strcmp(subplot_config,'two')
    %AR = 1;
    heigth_size = 1000;
elseif strcmp(subplot_config,'one')
    %AR = 8;
    heigth_size = 500;
else
    %AR = 1;
    heigth_size = 1000;
end
fig_height = heigth_size;
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

%% Find hover,transition and forward flight times
[hover_logical transition_logical ff_logical] = identify_hover_transition_ff(kalman_res.u(12,:));
hover_start = kalman_res.t(diff(hover_logical)==1);
hover_end = kalman_res.t(diff(hover_logical)==-1);
hover_start = [kalman_res.t(1) ; hover_start];
hover_end = [hover_end  ; kalman_res.t(end)];

transition_start = kalman_res.t(diff(transition_logical)==1);
transition_end = kalman_res.t(diff(transition_logical)==-1);
ff_start = kalman_res.t(diff(ff_logical)==1);
ff_end = kalman_res.t(diff(ff_logical)==-1);

%% First figure: Airspeed
if strcmp(subplot_config,'two')
    ax1=subplot(2,1,1);
elseif strcmp(subplot_config,'one')
    ax1 = subplot(1,1,1);
else
    ax1 = subplot(2,2,1);
end

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
legend([s1,s2],'Estimation','Pitot Tube','Orientation','horizontal')
axis([-inf inf 0 1.2.*max(airspeed_pitot.data)])

% Export figure
if strcmp(subplot_config,'one')
   % Change font size
   set(findall(gcf,'-property','FontSize'),'FontSize',font_size) 

   fig_name = [file_ident,'_airspeed_',formattedDateTime,'.eps'];
   exportgraphics(fig,fig_name,'BackgroundColor','none','ContentType','vector')
end

%% Second Figure: Wind

if strcmp(subplot_config,'two')
    % Nothing!
else
    if strcmp(subplot_config,'one')
        fig = figure('position',[0 0 fig_width fig_height]);
        set(gcf,'DefaultAxesColorOrder',mycolors, ...
        'DefaultAxesLineStyleOrder',mylinestyles)
        ax2 = subplot(1,1,1);
    else
        ax2 = subplot(2,2,2);
    end

    hold on
    p1=plot(kalman_res.t,kalman_res.x(4,:));
    yline([mean(kalman_res.x(4,:))],mylinestyles{ax2.ColorOrderIndex-1})
    text(510, mean(kalman_res.x(4,:)), 'Average N', 'VerticalAlignment', 'Bottom');
    ax2.LineStyleOrderIndex = ax2.ColorOrderIndex;
     
    p2=plot(kalman_res.t,kalman_res.x(5,:));
    yline([mean(kalman_res.x(5,:))],mylinestyles{ax2.ColorOrderIndex-1})
    text(510, mean(kalman_res.x(5,:)),'Average E', 'VerticalAlignment', 'Bottom');
    ax2.LineStyleOrderIndex = ax2.ColorOrderIndex;

    p3=plot(kalman_res.t,kalman_res.x(6,:));
    %yline([mean(kalman_res.x(6,:))],mylinestyles{ax2.ColorOrderIndex-1},'Average D')
    
    %title('Wind')
    xlabel('Time [s]')
    ylabel('Speed [m/s]')
    legend([p1,p2,p3],{'North','East','Down'},'Orientation','horizontal')
    grid on
    axis([-inf inf -inf 1.6*max(max(kalman_res.x(4:6,:)))])

    % Export figure
    if strcmp(subplot_config,'one')
       % Change font size
       set(findall(gcf,'-property','FontSize'),'FontSize',font_size) 

       fig_name = [file_ident,'_wind_',formattedDateTime,'.eps'];
       exportgraphics(fig,fig_name,'BackgroundColor','none','ContentType','vector')
    end
end

%% Third Figure: Skew
% Saturate skew angle
kalman_res.u(12,:) = min(deg2rad(90),max(kalman_res.u(12,:),0));

if strcmp(subplot_config,'two')
    ax3=subplot(2,1,2);
elseif strcmp(subplot_config,'one')
    fig = figure('position',[0 0 fig_width fig_height]);
    set(gcf,'DefaultAxesColorOrder',mycolors, ...
        'DefaultAxesLineStyleOrder',mylinestyles)
    ax3 = subplot(1,1,1);
else
    ax3 = subplot(2,2,3);
end

plot(kalman_res.t,rad2deg(kalman_res.u(12,:)));
hold on
%title('Skew')
xlabel('Time [s]','fontSize',font_size)
ylabel('Skew Angle [deg]','fontSize',font_size)
grid on
yline([0],'--','Quad-Mode','LabelHorizontalAlignment','Right','LabelVerticalAlignment','Top')
yline([90],'--','FWD Flight','LabelHorizontalAlignment','Right','LabelVerticalAlignment','Bottom')
axis([min(kalman_res.t) max(kalman_res.t) -5 125])
ax3.XAxis.FontSize = font_size;
ax3.YAxis.FontSize = font_size;

% Hatch definition
Alpha           = {0.5,0.5,0.5};
HatchColor		= {mycolors(1,:),mycolors(2,:),mycolors(3,:)};
HatchAngle		= {30,-45,70};
HatchType		= {'single','single','single'};

% Create patches and apply hatches
for i=1:length(hover_start)
    x = kalman_res.t(kalman_res.t>=hover_start(i) & kalman_res.t<=hover_end(i));
    y = rad2deg(kalman_res.u(12,kalman_res.t>=hover_start(i) & kalman_res.t<=hover_end(i))');
    
    % Remove unecessary points
    x = x(diff(y)~=0);
    y = y(diff(y)~=0);
    if length(x)>2 && length(y)>2
        x = [x; x(end); x(1)];
        y = [y;0;0];
        hover_patch(i) = patch(x,y,HatchColor{1},'FaceAlpha',Alpha{1},'EdgeColor','none');
        hatchfill2(hover_patch(i),'HatchStyle',HatchType{1},'HatchColor',HatchColor{1},'HatchAngle',HatchAngle{1});
    end
end
for i=1:length(transition_start)
    x = kalman_res.t(kalman_res.t>=transition_start(i) & kalman_res.t<=transition_end(i));
    y = rad2deg(kalman_res.u(12,kalman_res.t>=transition_start(i) & kalman_res.t<=transition_end(i))');
    
    % Remove unecessary points
    x = x(diff(y)~=0);
    y = y(diff(y)~=0);
    if length(x)>2 && length(y)>2
        x = [x; x(end); x(1)];
        y = [y;0;0];
        transition_patch(i) = patch(x,y,HatchColor{2},'FaceAlpha',Alpha{2},'EdgeColor','none');
        hatchfill2(transition_patch(i),'HatchStyle',HatchType{2},'HatchColor',HatchColor{2},'HatchAngle',HatchAngle{2});
    end
end
for i=1:length(ff_start)
    x = kalman_res.t(kalman_res.t>=ff_start(i) & kalman_res.t<=ff_end(i));
    y = rad2deg(kalman_res.u(12,kalman_res.t>=ff_start(i) & kalman_res.t<=ff_end(i))');
    
    % Remove unecessary points
    x = x(diff(y)~=0);
    y = y(diff(y)~=0);
    if length(x)>2 && length(y)>2
        x = [x; x(end); x(1)];
        y = [y;0;0];
        ff_patch(i) = patch(x,y,HatchColor{3},'FaceAlpha',Alpha{3},'EdgeColor','none');
        hatchfill2(ff_patch(i),'HatchStyle',HatchType{3},'HatchColor',HatchColor{3},'HatchAngle',HatchAngle{3});
    end
end

%Make the legend
Legend  			= {'Hover','Transition','Forward Flight'};
[legend_h,object_h,plot_h,text_str] = legendflex([hover_patch(1) transition_patch(1) ff_patch(1)],Legend,'nrow',1,'FontSize',font_size);
%object_h is the handle for the lines, patches and text objects
%hatch the legend patches to match the patches 
for i=1:3
    object_h(i+3).FaceAlpha = 0.5;
	hatchfill2(object_h(i+3),HatchType{i}, ...
					'HatchAngle',HatchAngle{i},...
					'HatchColor',HatchColor{i}, ...
                   'HatchDensity',10);
end

% Export figure
if strcmp(subplot_config,'one')
   % Change font size
   set(findall(gcf,'-property','FontSize'),'FontSize',font_size) 

   fig_name = [file_ident,'_skew_',formattedDateTime,'.eps'];
   exportgraphics(fig,fig_name,'BackgroundColor','none','ContentType','vector')
end

%% Fourth figure: Motor Command
if strcmp(subplot_config,'two')
    % Nothing
else
    if strcmp(subplot_config,'one')
        fig = figure('position',[0 0 fig_width fig_height]);
        set(gcf,'DefaultAxesColorOrder',mycolors, ...
        'DefaultAxesLineStyleOrder',mylinestyles)
        ax4 = subplot(1,1,1);
    else
    ax4 = subplot(2,2,4);
    end
    plot(kalman_res.t,kalman_res.u(10,:));
    ax4.LineStyleOrderIndex = ax4.ColorOrderIndex;
    hold on
    plot(kalman_res.t,kalman_res.u(11,:));
    %title('Motor Commands')
    if WT
        xlabel('Time [s]')
        ylabel('PWM Command')
        axis([-inf inf 900 2100])
    else
        xlabel('Time [s]')
        ylabel('Motor RPM')
        axis([-inf inf 0 10000])
        set(ax4, 'YTick', 0:3000:10000)
    end
    legend('Pusher Motor','Hover Motor','Orientation','horizontal')
    grid on

    % Export figure
    if strcmp(subplot_config,'one')
       % Change font size
       set(findall(gcf,'-property','FontSize'),'FontSize',font_size) 

       fig_name = [file_ident,'_motor_',formattedDateTime,'.eps'];
       exportgraphics(fig,fig_name,'BackgroundColor','none','ContentType','vector')
    end
end

if strcmp(subplot_config,'two')
    % Change font size
    set(findall(gcf,'-property','FontSize'),'FontSize',font_size) 

    linkaxes([ax1,ax3],'x')
    fig_name = [file_ident,'_two_',formattedDateTime,'.eps'];
    exportgraphics(fig,fig_name,'BackgroundColor','none','ContentType','vector')
elseif strcmp(subplot_config,'one')
    %Nothing
else
    % Change font size
    set(findall(gcf,'-property','FontSize'),'FontSize',font_size) 

    fig_name = [file_ident,'_four_',formattedDateTime,'.eps'];
    exportgraphics(fig,fig_name,'BackgroundColor','none','ContentType','vector')
    linkaxes([ax1,ax2,ax3,ax4],'x')
end

% Restore the original default line width value
set(groot, 'DefaultLineLineWidth', origLineWidth);
end