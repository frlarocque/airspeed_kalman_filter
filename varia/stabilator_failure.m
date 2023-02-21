%% Init
clear all
clc
close all

%% Parameters
g = 9.81;
rho = 1.225; %[kg/m^3]

W = 6.5*g; %weight of vehicule [N]
TWR = 2;
F_motor = W*TWR/4; % [N]
d_motor_fixed = 0.47; %[m]
d_motor_rotating = 0.37; %[m]
eta_reverse = 0.7; %efficiency of propeller when running in reverse

c = 0.1878; %chord of H-Stab [m]
b = 0.8; %span of H-Stab [m]
AR = b/c;
alpha = deg2rad([-20:1:20]);
V = [0:1:30]; %[m/s]
d_vstab = 0.696+c/4; % Distance from CG to CP H-Stab [m]

CL_0 = 0; %Symmetric airfoil
CL_alpha = 3.7586; %using XFLR5 %2*pi*(AR/(2+AR)); %Assumming elleptic lift distribution and thin airfoil theory (Anderson, 2017)

[ALPHA,VELOCITY] = meshgrid(alpha,V);

M = d_vstab.*0.5.*rho.*VELOCITY.^2.*c.*b.*(CL_0+CL_alpha.*ALPHA); %[N*m]

figure
surf(rad2deg(ALPHA),VELOCITY,M)
xlabel('Angle of attack [deg]')
ylabel('Velocity [m/s]')
zlabel('Pitching Moment [N/m]')
title('Pitching moment created by V-Stab as a function of velocity and angle of attack')

M_one_fixed = F_motor*d_motor_fixed;
M_two_fixed = F_motor*d_motor_fixed*(1+eta_reverse);
M_two_forward = F_motor*d_motor_fixed+F_motor*d_motor_rotating;
M_all_motors = F_motor*(d_motor_fixed+d_motor_rotating)*(1+eta_reverse);
excess_manoeuvering = 0.15;


curve_one_fixed_p = ((1-excess_manoeuvering)*M_one_fixed./(V.^2.*0.5.*rho.*c.*b.*d_vstab)-CL_0)./CL_alpha;
curve_one_fixed_m = ((1-excess_manoeuvering)*-M_one_fixed./(V.^2.*0.5.*rho.*c.*b.*d_vstab)-CL_0)./CL_alpha;
curve_two_fixed_p = ((1-excess_manoeuvering)*M_two_fixed./(V.^2.*0.5.*rho.*c.*b.*d_vstab)-CL_0)./CL_alpha;
curve_two_fixed_m = ((1-excess_manoeuvering)*-M_two_fixed./(V.^2.*0.5.*rho.*c.*b.*d_vstab)-CL_0)./CL_alpha;
curve_two_forward_p = ((1-excess_manoeuvering)*M_two_forward./(V.^2.*0.5.*rho.*c.*b.*d_vstab)-CL_0)./CL_alpha;
curve_two_forward_m = ((1-excess_manoeuvering)*-M_two_forward./(V.^2.*0.5.*rho.*c.*b.*d_vstab)-CL_0)./CL_alpha;
curve_all_p = ((1-excess_manoeuvering)*M_all_motors./(V.^2.*0.5.*rho.*c.*b.*d_vstab)-CL_0)./CL_alpha;
curve_all_m = ((1-excess_manoeuvering)*-M_all_motors./(V.^2.*0.5.*rho.*c.*b.*d_vstab)-CL_0)./CL_alpha;


figure
plot(V,rad2deg(curve_one_fixed_p))
hold on
plot(V,rad2deg(curve_two_fixed_p))
plot(V,rad2deg(curve_two_forward_p))
plot(V,rad2deg(curve_all_p))
axis([-inf inf 0 20])
legend({'One fixed motor','Two fixed motor (forward+reverse thrust)','Two forward thrust motor','All motors (forward+reverse thrust)'})
ylabel('Angle of attack [deg]')
xlabel('Velocity [m/s]')
title('Allowable V-Stab Angle of attack for different fixed motor configuration')
grid on
