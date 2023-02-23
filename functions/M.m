function M = M(g_fh,x,u,epsi)
%G Calculates output noise jacobian
%
% Inputs:
%           -g_fh: function handle to point to output function
%           -x: states
%           -u: inputs
%           -epsi: epsilon to perform numerical derivative

y = g_fh(x,u);
M = zeros(length(y),length(y));
for i=1:length(y)
    
   w_p = zeros(size(y));w_p(i)=w_p(i)+epsi;
   w_m = zeros(size(y));w_m(i)=w_m(i)-epsi;

   dm = (g_fh(x,u,w_p)-g_fh(x,u,w_m))/(2*epsi);
    
   M(:,i) = dm;
end
end