function g = g(state,input)
%x = [u v w phi theta psi mu_x mu_y mu_z];
%u = [a_x a_y a_z p q r];

u=state(1);v=state(2);w=state(3);phi=state(4);theta=state(5);psi=state(6);mu_x=state(7);mu_y=state(8);mu_z=state(9);
a_x=input(1);a_y=input(2);a_z=input(3);p=input(4);q=input(5);r=input(6);
% V_x V_y V_z
speed = DCM(phi,theta,psi)*[u;v;w]+[mu_x;mu_y;mu_z];

%alpha
alpha = atan2(w,u);
beta = asin(v/sqrt(u.^2+v.^2+w.^2));

g = [speed; alpha;beta];

end