k_lift = 0.5 * 1.225 * 0.4586;
k_tail = 0.5 * 1.225 * 0.138;
k_fuse = 0.5 * 1.225;

Fx_wing_TO = @ (u,sk,the) (-227.085647.*k_lift.*u.^2  +  5186.70122.*k_lift.*the.*u.^2.*sin(sk).^2  +  25531.1699.*k_lift.*the.^2.*u.^2.*sin(sk).^2)./10000;
Fy_wing_TO = @ (u,sk,the) (744.464339.*k_lift.*sk.*u.^2  +  1514.24153.*k_lift.*the.*u.^2  +  -491.620021.*k_lift.*sk.^2.*u.^2  +  13256.1841.*k_lift.*the.^2.*u.^2  +  14654.8858.*k_lift.*sk.*the.*u.^2  +  -9995.34011.*k_lift.*sk.^2.*the.*u.^2)./10000;

Fz_wing_TO = @ (u,sk,the) (-14473.5556.*k_lift.*the.*u.^2  +  -4300.0198.*k_lift.*u.^2.*sin(sk).^2  +  -26532.9954.*k_lift.*the.*u.^2.*sin(sk).^2)./10000;

Fx_ele_TO = 0;
Fy_ele_TO = 0;
Fz_ele_TO = @ (u,the) (501.156273.*k_tail.*u.^2  +  -17187.0962.*k_tail.*the.*u.^2)./10000;

Fx_fuse_TO = @ (u,sk,the) (-480.00018.*k_fuse.*u.^2  +  62.418984.*k_fuse.*sk.^2.*u.^2  +  1809.023.*k_fuse.*the.^2.*u.^2)./10000;
Fy_fuse_TO = 0;
Fz_fuse_TO = @ (u,the) (-1181.4327.*k_fuse.*the.*u.^2./10000);


%% Compare fuselage

u = 15;
skew = deg2rad(90);
theta = deg2rad([0:0.1:10]);

figure
for i=1:length(theta)
    hold on
    plot(rad2deg(theta(i)),Fz_wing_TO(u,skew,theta(i)),'o')
    plot(rad2deg(theta(i)),Fz_wing(skew,theta(i),u),'*')

end

