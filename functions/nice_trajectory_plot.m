function nice_trajectory_plot(position_NED,velocity_NED,IMU_angle,dt,wind_vect,skew,show_time)
%TRAJECTORY Plots vehicle trajectory and velocity vector in NED Frame
%
% Inputs:
%           -position_NED: struct with position in NED frame [m]
%           -velocity_NED: struct with velocity in NED frame [m/s]
%           -IMU_angle: struct with euleur angles [rad]
%
% Ouputs: none

%% General Figure Setup
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
set(gcf,'DefaultAxesColorOrder',mycolors)

%%

psi.data = IMU_angle.data(:,3);psi.time = IMU_angle.time;

% Define start and end time
start_time = max([position_NED.time(1),velocity_NED.time(1),psi.time(1)]);
end_time = min([position_NED.time(end),velocity_NED.time(end),psi.time(end)]);

% Resample to dt
resample_time = [start_time:dt:end_time];

if nargin<7
    show_time = false;
end

if nargin<6
    skew = zeros(1,length(resample_time));
else
    skew = resample(timeseries(skew.data,skew.time),position_NED.time);
    skew = skew.data;
end

% Position
XY = resample(timeseries(position_NED.data(:,[1:2]),position_NED.time), resample_time);
X = XY.data(:,1);
Y = XY.data(:,2);

% Velocity
UV = resample(timeseries(velocity_NED.data(:,[1:2]),velocity_NED.time), resample_time);
U = UV.data(:,1);
V = UV.data(:,2);

% Heading Angle
PSI = resample(timeseries(psi.data,psi.time), resample_time);
PSI = PSI.data;

psi_u = cos(PSI).*sqrt(U.^2+V.^2);
psi_v = sin(PSI).*sqrt(U.^2+V.^2);

%% Plot figure
scale = 1.0;
start_point = plot(position_NED.data(1,2),position_NED.data(1,1),"^",'MarkerSize',marker_size,...
            'MarkerFaceColor',mycolors(1,:), ... 
            'MarkerEdgeColor',mycolors(1,:));
hold on
end_point = plot(position_NED.data(end,2),position_NED.data(end,1),"v",'MarkerSize',marker_size,...
            'MarkerFaceColor',mycolors(2,:), ... 
            'MarkerEdgeColor',mycolors(2,:));

if all(skew==0)
    p1 = plot(position_NED.data(:,2),position_NED.data(:,1),'--r');
else
    interval = 50;
    x = position_NED.data(1:interval:end,2)';
    y = position_NED.data(1:interval:end,1)';
    z = zeros(size(x));
    lineColor = rad2deg(skew(1:interval:end))';
    p1 = surface([x;x], [y;y], [z;z], [lineColor;lineColor],...
	    'FaceColor', 'no',...
	    'EdgeColor', 'interp',...
	    'LineWidth', line_width*3);
    grid on;colorbar('northoutside',...
        'Ticks',[0 45 90],...
        'TickLabels',{'Quad Mode','Transition','FWD Flight'});
end    
hold on
xlabel('East position [m]')
ylabel('North position [m]')
q1 = quiver(Y,X,V,U,scale,'Color',mycolors(3,:),...
    'linewidth',line_width);
q2 = quiver(Y,X,psi_v,psi_u,scale,'Color',mycolors(4,:),...
    'linewidth',line_width);
if show_time
    for i=1:length(X)
        text(Y(i),X(i),sprintf("%2.0f",resample_time(i)))
    end
end
axis('equal')
%axis('equal','padded')
legend([start_point end_point q1 q2],{'Takeoff','Landing','Ground Speed','Orientation'},'Location','Northwest')

% Change font size
set(findall(gcf,'-property','FontSize'),'FontSize',font_size) 

if nargin>4
    text(200,120,'Wind [m/s]','FontSize',font_size-4)
    % Plot wind if available
    axes('Position',[.62 .14 0.3 0.3])
    c = compass(wind_vect(1),wind_vect(2));
    text_hdl = polarticks(8,c);
    view(90,-90)
    c(1).Color = mycolors(5,:);
    c(1).LineWidth = line_width;
    
    set(findall(text_hdl,'-property','FontSize'),'FontSize',font_size-6)
    ax2=gca;
    ax2.FontSize = font_size-4; 
end

fig_name = ['trajectory_',formattedDateTime,'.eps'];
exportgraphics(fig,fig_name,'BackgroundColor','none','ContentType','vector')