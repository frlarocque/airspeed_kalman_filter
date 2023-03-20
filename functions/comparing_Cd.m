
RPM_hover = 3800;
u = [0:0.1:20];

F_total = zeros(1,length(u));

for j=1:length(u)

    Fx_fus = Fx_fuselage(0,0,u(j));
    Fx_hprop = Fx_hover_prop(RPM_hover,u(j));
    Fx_w = 0 ; %wing is not installed! Fx_w = Fx_wing(skew,alpha,V_a);
    Fx_elev = Fx_elevator(0,u(j));

    F_total(j) = Fx_fus+Fx_hprop+Fx_elev+Fx_w;

end

u_crit = 6;
k_1 = -0.095;
k_2 = -0.053;
k_sections = k_2.*u.^2+u_crit.^2.*(k_1-k_2);
k_sections(u<u_crit) = k_1.*u(u<u_crit).^2;

figure
plot(u,-0.095.*u.^2)
hold on
plot(u,-0.053.*u.^2)
plot(u,k_sections,'--')
plot(u,F_total)
legend('0.095','0.053','sections','Current Model')