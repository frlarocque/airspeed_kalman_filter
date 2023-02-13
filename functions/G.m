function G = G(x,u,epsi)
y = g(x,u);
G = zeros(length(y),length(x));
for i=1:length(x)
    
   x_p = x;x_p(i)=x_p(i)+epsi;
   x_m = x;x_m(i)=x_m(i)-epsi;

   dg = (g(x_p,u)-g(x_m,u))/(2*epsi);
    
   G(:,i) = dg;
end
end