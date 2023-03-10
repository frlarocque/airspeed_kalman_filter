function F = F(f_fh,x,u,epsi)
%F Calculates states jacobian
%
% Inputs:
%           -f_fh: function handle to point to output function
%           -x: states
%           -u: inputs
%           -epsi: epsilon to perform numerical derivative
%
% F = [] nxn where n=number of states, n=number of states
%
% F = [ dx_1/dx_1 dx_1/dx_2 ... dx_1/dx_n ;
%       dx_2/dx_1 dx_2/dx_2 ....dx_2/dx_n ;
%       ...                               ;
%       dx_n/dx_1 dx_n/dx_2 ....dx_n/dx_n ];

F = zeros(length(x),length(x));
for i=1:length(x)
    
   x_p = x;x_p(i)=x_p(i)+epsi;
   x_m = x;x_m(i)=x_m(i)-epsi;

   df = (f_fh(x_p,u)-f_fh(x_m,u))/(2*epsi);
    
   F(:,i) = df;
end
end