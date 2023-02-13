function F = F(x,u,epsi)
F = zeros(length(x),length(x));
for i=1:length(x)
    
   x_p = x;x_p(i)=x_p(i)+epsi;
   x_m = x;x_m(i)=x_m(i)-epsi;

   df = (f(x_p,u)-f(x_m,u))/(2*epsi);
    
   F(:,i) = df;
end
end