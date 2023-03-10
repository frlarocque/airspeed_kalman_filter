function L = L(f_fh,x,u,epsi)
%F Calculates states noise jacobian
%
% Inputs:
%           -f_fh: function handle to point to output function
%           -x: states
%           -u: inputs
%           -epsi: epsilon to perform numerical derivative
%
% L = [] nxm where n=number of states, m=number of noise
%
% L = [ dx_1/dw_1 dx_1/dw_2 ... dx_1/dw_m ;
%       dx_2/dw_1 dx_2/dw_2 ....dx_2/dw_m ;
%       ...                               ;
%       dx_n/dw_1 dx_n/dw_2 ....dx_n/dw_m ];

noise_length = 9;

L = zeros(length(x),noise_length);
for i=1:noise_length
    
   w_p = zeros(noise_length,1);w_p(i)=w_p(i)+epsi;
   w_m = zeros(noise_length,1);w_m(i)=w_m(i)-epsi;

   dl = (f_fh(x,u,w_p)-f_fh(x,u,w_m))/(2*epsi);
    
   L(:,i) = dl;
end
end