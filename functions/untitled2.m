
cond = t>194 & t<230;

figure

subplot(1,3,1)
plot(t(cond),a_y_filt(cond));
hold on
xlabel('Time [s]')
ylabel('A_y [m/s^2]')

subplot(1,3,2)
plot(t(cond),rad2deg(beta.flight.data(cond)))
hold on
plot(t(cond),rad2deg(a_y_filt(cond)./(airspeed_pitot.flight.data(cond).^2.*-2E-1)))
xlabel('Time [s]')
ylabel('Beta [deg]')
axis([-inf inf -180 180])

v = airspeed_pitot.flight.data(cond).*sin(beta.flight.data(cond));

subplot(1,3,3)
plot(t(cond),v)
hold on
plot(t(cond),sqrt(abs(a_y_filt(cond)./(10.*-0.32./5.75))).*-sign(a_y_filt(cond)))

xlabel('Time [s]')
ylabel('v [m/s]')

subplot(1,3,1)
plot(t(cond),v.*v.*sign(v).*-3.2E-1/5.75)

%%

cond = t>194 & t<230;
cond = t>194;

figure
subplot(3,1,1)
plot(t(cond),airspeed_pitot.flight.data(cond))
subplot(3,1,2)
plot(t(cond),pusher_prop_rpm_filt(cond))
subplot(3,1,3)
plot(t(cond),a_x_filt(cond))
hold on
plot(t(cond),(Fx_pusher(pusher_prop_rpm_filt(cond),airspeed_pitot.flight.data(cond))+1.6.*Fx_fuselage(0,0,airspeed_pitot.flight.data(cond))+0.5.*Fx_hover_prop(hover_prop_rpm_filt(cond),airspeed_pitot.flight.data(cond)))./5.75)
plot(t(cond),Fx_pusher(pusher_prop_rpm_filt(cond),airspeed_pitot.flight.data(cond))./5.75)
plot(t(cond),1.6*Fx_fuselage(0,0,airspeed_pitot.flight.data(cond))./5.75)
plot(t(cond),Fx_hover_prop(hover_prop_rpm_filt(cond),airspeed_pitot.flight.data(cond))./5.75)
legend('ax','est','pusher','fuse','hp')

x = [Fx_pusher(pusher_prop_rpm_filt(cond),airspeed_pitot.flight.data(cond)) Fx_fuselage(0,0,airspeed_pitot.flight.data(cond)) airspeed_pitot.flight.data(cond)];
y = a_x_filt(cond);

fit = @(k,x)  (k(1).*x(:,1)+k(2).*x(:,3)+k(3).*x(:,3).^2)./5.75;          % Function to fit
fcn = @(k) sqrt(mean((fit(k,x) - y).^2));  % Least-Squares cost function
[s,RMS] = fminsearchbnd(fcn,[1 1 1],[1 -2.5 -0.5],[1.1 2.5 1.5])  

%  s =
% 
%    1.000000000038122  -0.176146941826598  -0.120953514419305


figure
plot(t(cond),y)
hold on
plot(t(cond),fit(s,x))