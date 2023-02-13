function F = F_2(x,u,epsi)
F = zeros(length(x),length(x));
for i=1:length(x)
    
   x_p = x;x_p(i)=x_p(i)+epsi;
   x_m = x;x_m(i)=x_m(i)-epsi;

   df = (f_2(x_p,u)-f_2(x_m,u))/(2*epsi);
    
   F(:,i) = df;
end
end