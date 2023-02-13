function trajectory(position_NED,velocity_NED,IMU_angle,dt)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

%position_NED = position_NED.flight;
%velocity_NED = V_XYZ.flight;
psi.data = IMU_angle.data(:,3);psi.time = IMU_angle.time;

%dt = 10;

start_time = max([position_NED.time(1),velocity_NED.time(1),psi.time(1)]);
end_time = min([position_NED.time(end),velocity_NED.time(end),psi.time(end)]);

resample_time = [start_time:dt:end_time];

% Resample to dt
XY = resample(timeseries(position_NED.data(:,[1:2]),position_NED.time), resample_time);
X = XY.data(:,1);
Y = XY.data(:,2);

UV = resample(timeseries(velocity_NED.data(:,[1:2]),velocity_NED.time), resample_time);
U = UV.data(:,1);
V = UV.data(:,2);

PSI = resample(timeseries(psi.data,psi.time), resample_time);
PSI = PSI.data;

psi_u = cos(PSI).*sqrt(U.^2+V.^2);
psi_v = sin(PSI).*sqrt(U.^2+V.^2);

scale = 0.5;

figure('name','Trajectory')
p1 = plot(position_NED.data(:,1),position_NED.data(:,2),'--r');
hold on
xlabel('x position [m]')
ylabel('y position [m]')
q1 = quiver(X,Y,U,V,0.5,'b','linewidth',2);
q2 = quiver(X,Y,psi_u,psi_v,0.5,'c','linewidth',2);
axis('equal')
legend([p1,q1,q2],{'Trajectory','Ground Speed','Orientation'})

end