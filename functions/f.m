function f = f(state,input,process_noise)
%F State function of Kalman filter with Euler Angles estimation 
%
% x = [u v w phi theta psi mu_x mu_y mu_z];
% u = [a_x a_y a_z p q r];

u=state(1);v=state(2);w=state(3);phi=state(4);theta=state(5);psi=state(6);mu_x=state(7);mu_y=state(8);mu_z=state(9);
a_x=input(1);a_y=input(2);a_z=input(3);p=input(4);q=input(5);r=input(6);

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

% [phi_dot theta_dot psi_dot]'
attitude = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
            0 cos(phi)            -sin(phi);
            0 sin(phi)*sec(theta) cos(phi)*sec(theta)]*...
            ([p;q;r]+w_w);

%[mu_x_dot mu_y_dot mu_z_dot]'
wind = w_mu;

%f = x_dot = [u_dot v_dot w_dot phi_dot psi_dot mu_x_dot mu_y_dot mu_z_dot];
f = [velocity_body;attitude;wind];

end

