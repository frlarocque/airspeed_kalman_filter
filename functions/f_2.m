function f = f_2(state,input,process_noise)
%F State function for Kalman Filter without Euler Angle Approximation
%
% x = [u v w mu_x mu_y mu_z];
% u = [a_x a_y a_z p q r phi theta psi];

u=state(1);v=state(2);w=state(3);mu_x=state(4);mu_y=state(5);mu_z=state(6);
a_x=input(1);a_y=input(2);a_z=input(3);p=input(4);q=input(5);r=input(6);phi=input(7);theta=input(8);psi=input(9);

if nargin==3
    w_a = process_noise(1:3);
    w_w = process_noise(4:6);
    w_mu = process_noise(7:9);
else
    w_a = zeros(3,1);
    w_w = zeros(3,1);
    w_mu = zeros(3,1);
end

g= 9.81;

% [u_dot v_dot w_dot]'
velocity_body = [0 -w  v;
                 w  0 -u;
                 -v u  0;]*([p;q;r]+w_w)+ ...
                 DCM(phi,theta,psi)'*[0;0;g]+...
                 [a_x;a_y;a_z]+w_a;

%[mu_x_dot mu_y_dot mu_z_dot]'
wind = w_mu;

%f = x_dot = [u_dot v_dot w_dot mu_x_dot mu_y_dot mu_z_dot];
f = [velocity_body;wind];

end

