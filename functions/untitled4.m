u = kalman_res{1}.x(1,:)';
RPM_pusher = kalman_res{1}.u(10,:)';
RPM_hover = kalman_res{1}.u(11,:)';
skew = kalman_res{1}.u(12,:)';
elevator_pprz = kalman_res{1}.u(13,:)';
alpha = atan2(kalman_res{1}.x(1,:),kalman_res{1}.x(3,:))';
alpha = max(min(alpha,deg2rad(15)),deg2rad(-15));
V_a = vecnorm(kalman_res{1}.x(1:3,:))';

% A_x
m = 5;

Fx_push = Fx_pusher(RPM_pusher,u);
Fx_push(RPM_pusher<100) = 0;

Fx_fus = Fx_fuselage(skew,alpha,V_a);
Fx_hprop = Fx_hover_prop(RPM_hover,u);
%Fx_w = Fx_wing(skew,alpha,V_a);
Fx_elev = Fx_elevator(elevator_pprz,V_a);
%a_x = (Fx_push+Fx_fus+Fx_hprop+Fx_elev)./m;
%a_x = (Fx_push+Fx_fus+Fx_hprop+Fx_elev+Fx_w+)./m;

k(4) = 0;
k(5) = -0.083946669313987*0.8; %b2;
F_drag = k(4).*u+k(5).*u.^2;


%a_x = (Fx_push+F_drag)./m;

%%

figure
subplot(3,1,1)
plot(t,Fx_push)
hold on
plot(t,F_drag)
legend('Pusher','Drag')

subplot(3,1,2)
plot(t,Fx_push)
hold on
plot(t,Fx_fus)
plot(t,Fx_hprop)
plot(t,Fx_elev)
plot(t,sum([Fx_fus,Fx_hprop,Fx_elev],2))
legend('Pusher','fuselage','hprop','elev')

subplot(3,1,3)
plot(t,F_drag)
hold on
plot(t,sum([Fx_fus,Fx_hprop,Fx_elev],2))

%%
figure
plot(t,F_drag./(V_a.^2))
hold on
plot(t,Fx_fus./V_a.^2)
plot(t,Fx_hprop./V_a.^2)
plot(t,Fx_elev./V_a.^2)
legend('Drag','Fus','Hprop','Elev')