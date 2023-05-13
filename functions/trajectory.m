function trajectory(position_NED,velocity_NED,IMU_angle,dt,wind_vect,skew,show_time)
%TRAJECTORY Plots vehicle trajectory and velocity vector in NED Frame
%
% Inputs:
%           -position_NED: struct with position in NED frame [m]
%           -velocity_NED: struct with velocity in NED frame [m/s]
%           -IMU_angle: struct with euleur angles [rad]
%
% Ouputs: none

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

% Plot figure
scale = 0.5;
figure('name','Trajectory')
s1 = plot(position_NED.data(1,2),position_NED.data(1,1),'pentagram','MarkerSize',20,'MarkerFaceColor','r')
if all(skew==0)
    p1 = plot(position_NED.data(:,2),position_NED.data(:,1),'--r');
else
    x = position_NED.data(:,2)';
    y = position_NED.data(:,1)';
    z = zeros(size(x));
    lineColor = rad2deg(skew)';
    p1 = surface([x;x], [y;y], [z;z], [lineColor;lineColor],...
	    'FaceColor', 'no',...
	    'EdgeColor', 'interp',...
	    'LineWidth', 2);
    grid on;colorbar;
end    
hold on
xlabel('E position [m]')
ylabel('N position [m]')
q1 = quiver(Y,X,V,U,0.5,'b','linewidth',2);
q2 = quiver(Y,X,psi_v,psi_u,0.5,'c','linewidth',2);
if show_time
    for i=1:length(X)
        text(Y(i),X(i),sprintf("%2.0f",resample_time(i)))
    end
end
axis('equal')
legend([s1 p1,q1,q2],{'Start','Trajectory','Ground Speed','Orientation'})

if nargin>4
    % Plot wind if available
    axes('Position',[.75 .15 0.1 0.1])
    q3 = quiver(0,0,wind_vect(2),wind_vect(1),'color',[0.8500 0.3250 0.0980],'linewidth',2,'MaxHeadSize',0.7);
    title('Wind')
end