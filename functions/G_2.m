function G = G_2(x,u,epsi)
y = g_2(x,u);
G = zeros(length(y),length(x));
for i=1:length(x)
    
   x_p = x;x_p(i)=x_p(i)+epsi;
   x_m = x;x_m(i)=x_m(i)-epsi;

   dg = (g_2(x_p,u)-g_2(x_m,u))./(2*epsi);
    
   G(:,i) = dg;
end
end