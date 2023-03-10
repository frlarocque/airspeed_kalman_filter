function G = G(g_fh,x,u,epsi)
%G Calculates output jacobian
%
% Inputs:
%           -g_fh: function handle to point to output function
%           -x: states
%           -u: inputs
%           -epsi: epsilon to perform numerical derivative
%
% G = [] nxm where n=number of outputs, m=number of states
%
% G = [ dg_1/dx_1 dg_1/dx_2 ... dg_1/dx_m ;
%       dg_2/dx_1 dg_2/dx_2 ....dg_2/dx_m ;
%       ...                               ;
%       dg_n/dx_1 dg_n/dx_2 ....dg_n/dx_m ];


y = g_fh(x,u);
G = zeros(length(y),length(x));
for i=1:length(x)
    
   x_p = x;x_p(i)=x_p(i)+epsi;
   x_m = x;x_m(i)=x_m(i)-epsi;

   dg = (g_fh(x_p,u)-g_fh(x_m,u))/(2*epsi);
    
   G(:,i) = dg;
end
end