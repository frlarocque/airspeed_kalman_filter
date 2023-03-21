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

%% Compare parameters
u_list = [5,8,11,14];
select = 3;
skew_list = deg2rad([0:30:90]);
theta_list = deg2rad([-5:0.5:15]);

[SKEW_LIST,THETA_LIST] = meshgrid(skew_list,theta_list);
%% Compare Wing 

%Fx
figure;
legend_lbl = {};
col=linspecer(length(u_list));
hdls = [];
Ax = []; 
for j=1:length(skew_list)
    Ax(end+1) = subplot(ceil(sqrt(length(skew_list))),ceil(sqrt(length(skew_list))),j);
    title(sprintf('Skew %2.0f deg',rad2deg(skew_list(j))))

    for i=1:length(u_list)
        hold on
        hdls(i) = plot(rad2deg(theta_list),Fx_wing_TO(u_list(i),skew_list(j),theta_list),'-','color',col(i,:));
        plot(rad2deg(theta_list),Fx_wing(skew_list(j),theta_list,u_list(i)),'--','color',col(i,:))
        legend_lbl{i} = [mat2str(u_list(i)),' m/s'];
    end

    if j==length(skew_list)
        set(Ax(j), 'Box','off')
        lgd1 = legend(hdls,legend_lbl,'location','southeast');
        title(lgd1,'Airspeed') % add legend title
        
        % copy axes 
        Ax(j+1) = copyobj(Ax(j),gcf);
        delete(get(Ax(j+1), 'Children') )
        
        % plot helper data, but invisible
        hold on
        H1 = plot([NaN NaN],[NaN NaN], '-', 'LineWidth', 1, 'Color', [0 0 0], 'Parent', Ax(j+1));
        H2 = plot([NaN NaN],[NaN NaN], '--', 'LineWidth', 1, 'Color', [0 0 0], 'Parent', Ax(j+1));
        hold off
        % make second axes invisible
        set(Ax(j+1), 'Color', 'none', 'XTick', [], 'YAxisLocation', 'right', 'Box', 'Off', 'Visible', 'off')
        % add linestyle legend
        lgd2 = legend([H1 H2], 'Tomaso', 'Frédéric', 'Location', 'northwest');
        title(lgd2,'Fit Source') % add legend title
        set(lgd2,'color','none')
    end
    xlabel('Angle of Attack [deg]')
    ylabel('Force [N]')
    grid on
end
sgtitle('Wing x-axis')
%linkaxes(Ax(1:end-1),'y')

% Difference between models
figure
subplot(1,2,1)
surf(rad2deg(SKEW_LIST),rad2deg(THETA_LIST),(Fx_wing_TO(u_list(select),SKEW_LIST,THETA_LIST)-Fx_wing(SKEW_LIST,THETA_LIST,u_list(select))))
xlabel('Skew [deg]')
ylabel('Angle of attack [deg]')
zlabel('Error (T-F) [N]')
subplot(1,2,2)
surf(rad2deg(SKEW_LIST),rad2deg(THETA_LIST),(Fx_wing_TO(u_list(select),SKEW_LIST,THETA_LIST)-Fx_wing(SKEW_LIST,THETA_LIST,u_list(select)))./Fx_wing_TO(u_list(select),SKEW_LIST,THETA_LIST))
xlabel('Skew [deg]')
ylabel('Angle of attack [deg]')
zlabel('Error (T-F) [%]')
zlim([-1 1])
sgtitle('Difference between models: Fx Wing')

%Fz
figure;
legend_lbl = {};
col=linspecer(length(u_list));
hdls = [];
Ax = []; 
for j=1:length(skew_list)
    Ax(end+1) = subplot(ceil(sqrt(length(skew_list))),ceil(sqrt(length(skew_list))),j);
    title(sprintf('Skew %2.0f deg',rad2deg(skew_list(j))))

    for i=1:length(u_list)
        hold on
        hdls(i) = plot(rad2deg(theta_list),Fz_wing_TO(u_list(i),skew_list(j),theta_list),'-','color',col(i,:));
        plot(rad2deg(theta_list),Fz_wing(skew_list(j),theta_list,u_list(i)),'--','color',col(i,:))
        legend_lbl{i} = [mat2str(u_list(i)),' m/s'];
    end

    if j==length(skew_list)
        set(Ax(j), 'Box','off')
        lgd1 = legend(hdls,legend_lbl,'location','southeast');
        title(lgd1,'Airspeed') % add legend title
        
        % copy axes 
        Ax(j+1) = copyobj(Ax(j),gcf);
        delete(get(Ax(j+1), 'Children') )
        
        % plot helper data, but invisible
        hold on
        H1 = plot([NaN NaN],[NaN NaN], '-', 'LineWidth', 1, 'Color', [0 0 0], 'Parent', Ax(j+1));
        H2 = plot([NaN NaN],[NaN NaN], '--', 'LineWidth', 1, 'Color', [0 0 0], 'Parent', Ax(j+1));
        hold off
        % make second axes invisible
        set(Ax(j+1), 'Color', 'none', 'XTick', [], 'YAxisLocation', 'right', 'Box', 'Off', 'Visible', 'off')
        % add linestyle legend
        lgd2 = legend([H1 H2], 'Tomaso', 'Frédéric', 'Location', 'northwest');
        title(lgd2,'Fit Source') % add legend title
        set(lgd2,'color','none')
    end
    xlabel('Angle of Attack [deg]')
    ylabel('Force [N]')
    grid on
end
sgtitle('Wing z-axis')
%linkaxes(Ax(1:end-1),'y')

% Difference between models
figure
subplot(1,2,1)
surf(rad2deg(SKEW_LIST),rad2deg(THETA_LIST),(Fz_wing_TO(u_list(select),SKEW_LIST,THETA_LIST)-Fz_wing(SKEW_LIST,THETA_LIST,u_list(select))))
xlabel('Skew [deg]')
ylabel('Angle of attack [deg]')
zlabel('Error (T-F) [%]')
zlim([-1 1])
subplot(1,2,2)
surf(rad2deg(SKEW_LIST),rad2deg(THETA_LIST),(Fz_wing_TO(u_list(select),SKEW_LIST,THETA_LIST)-Fz_wing(SKEW_LIST,THETA_LIST,u_list(select)))./Fz_wing_TO(u_list(select),SKEW_LIST,THETA_LIST))
xlabel('Skew [deg]')
ylabel('Angle of attack [deg]')
zlabel('Error (T-F) [N]')
sgtitle('Difference between models: Fz Wing')

%% Compare Fuselage

%Fx
figure;
legend_lbl = {};
col=linspecer(length(u_list));
hdls = [];
Ax = []; 
for j=1:length(skew_list)
    Ax(end+1) = subplot(ceil(sqrt(length(skew_list))),ceil(sqrt(length(skew_list))),j);
    title(sprintf('Skew %2.0f deg',rad2deg(skew_list(j))))

    for i=1:length(u_list)
        hold on
        hdls(i) = plot(rad2deg(theta_list),Fx_fuse_TO(u_list(i),skew_list(j),theta_list),'-','color',col(i,:));
        plot(rad2deg(theta_list),Fx_fuselage(skew_list(j),theta_list,u_list(i)),'--','color',col(i,:))
        legend_lbl{i} = [mat2str(u_list(i)),' m/s'];
    end

    if j==length(skew_list)
        set(Ax(j), 'Box','off')
        lgd1 = legend(hdls,legend_lbl,'location','southeast');
        title(lgd1,'Airspeed') % add legend title
        
        % copy axes 
        Ax(j+1) = copyobj(Ax(j),gcf);
        delete(get(Ax(j+1), 'Children') )
        
        % plot helper data, but invisible
        hold on
        H1 = plot([NaN NaN],[NaN NaN], '-', 'LineWidth', 1, 'Color', [0 0 0], 'Parent', Ax(j+1));
        H2 = plot([NaN NaN],[NaN NaN], '--', 'LineWidth', 1, 'Color', [0 0 0], 'Parent', Ax(j+1));
        hold off
        % make second axes invisible
        set(Ax(j+1), 'Color', 'none', 'XTick', [], 'YAxisLocation', 'right', 'Box', 'Off', 'Visible', 'off')
        % add linestyle legend
        lgd2 = legend([H1 H2], 'Tomaso', 'Frédéric', 'Location', 'northwest');
        title(lgd2,'Fit Source') % add legend title
        set(lgd2,'color','none')
    end
    xlabel('Angle of Attack [deg]')
    ylabel('Force [N]')
    grid on
end
sgtitle('Fuselage x-axis')
%linkaxes(Ax(1:end-1),'y')

% Difference between models
figure
subplot(1,2,1)
surf(rad2deg(SKEW_LIST),rad2deg(THETA_LIST),(Fx_fuse_TO(u_list(select),SKEW_LIST,THETA_LIST)-Fx_fuselage(SKEW_LIST,THETA_LIST,u_list(select))))
xlabel('Skew [deg]')
ylabel('Angle of attack [deg]')
zlabel('Error (T-F) [N]')
subplot(1,2,2)
surf(rad2deg(SKEW_LIST),rad2deg(THETA_LIST),(Fx_wing_TO(u_list(select),SKEW_LIST,THETA_LIST)-Fx_wing(SKEW_LIST,THETA_LIST,u_list(select)))./Fx_wing_TO(u_list(select),SKEW_LIST,THETA_LIST))
xlabel('Skew [deg]')
ylabel('Angle of attack [deg]')
zlabel('Error (T-F) [%]')
zlim([-1 1])
sgtitle('Difference between models: Fx Fuselage')

%Fz
figure;
legend_lbl = {};
col=linspecer(length(u_list));
hdls = [];
Ax = []; 
for j=1:length(skew_list)
    Ax(end+1) = subplot(ceil(sqrt(length(skew_list))),ceil(sqrt(length(skew_list))),j);
    title(sprintf('Skew %2.0f deg',rad2deg(skew_list(j))))

    for i=1:length(u_list)
        hold on
        hdls(i) = plot(rad2deg(theta_list),Fz_fuse_TO(u_list(i),theta_list),'-','color',col(i,:));
        plot(rad2deg(theta_list),Fz_fuselage(skew_list(j),theta_list,u_list(i)),'--','color',col(i,:))
        legend_lbl{i} = [mat2str(u_list(i)),' m/s'];
    end

    if j==length(skew_list)
        set(Ax(j), 'Box','off')
        lgd1 = legend(hdls,legend_lbl,'location','southeast');
        title(lgd1,'Airspeed') % add legend title
        
        % copy axes 
        Ax(j+1) = copyobj(Ax(j),gcf);
        delete(get(Ax(j+1), 'Children') )
        
        % plot helper data, but invisible
        hold on
        H1 = plot([NaN NaN],[NaN NaN], '-', 'LineWidth', 1, 'Color', [0 0 0], 'Parent', Ax(j+1));
        H2 = plot([NaN NaN],[NaN NaN], '--', 'LineWidth', 1, 'Color', [0 0 0], 'Parent', Ax(j+1));
        hold off
        % make second axes invisible
        set(Ax(j+1), 'Color', 'none', 'XTick', [], 'YAxisLocation', 'right', 'Box', 'Off', 'Visible', 'off')
        % add linestyle legend
        lgd2 = legend([H1 H2], 'Tomaso', 'Frédéric', 'Location', 'northwest');
        title(lgd2,'Fit Source') % add legend title
        set(lgd2,'color','none')
    end
    xlabel('Angle of Attack [deg]')
    ylabel('Force [N]')
    grid on
end
sgtitle('Fuselage z-axis')
%linkaxes(Ax(1:end-1),'y')

% Difference between models
figure
subplot(1,2,1)
surf(rad2deg(SKEW_LIST),rad2deg(THETA_LIST),(Fz_fuse_TO(u_list(select),THETA_LIST)-Fx_fuselage(SKEW_LIST,THETA_LIST,u_list(select))))
xlabel('Skew [deg]')
ylabel('Angle of attack [deg]')
zlabel('Error (T-F) [N]')
subplot(1,2,2)
surf(rad2deg(SKEW_LIST),rad2deg(THETA_LIST),(Fz_fuse_TO(u_list(select),THETA_LIST)-Fx_fuselage(SKEW_LIST,THETA_LIST,u_list(select)))./Fx_wing_TO(u_list(select),SKEW_LIST,THETA_LIST))
xlabel('Skew [deg]')
ylabel('Angle of attack [deg]')
zlabel('Error (T-F) [%]')
zlim([-1 1])
sgtitle('Difference between models: Fz Fuselage')

%% Compare elevator
%Fz
f1 = figure;
legend_lbl = {};
col=linspecer(length(u_list));
hdls = [];
Ax = axes(f1); 

for i=1:length(u_list)
    hold on
    hdls(i) = plot(rad2deg(theta_list),Fz_ele_TO(u_list(i),theta_list),'-','color',col(i,:));
    plot(rad2deg(theta_list),Fz_elevator(theta_list,u_list(i),0),'--','color',col(i,:))
    legend_lbl{i} = [mat2str(u_list(i)),' m/s'];
end

    set(Ax(1), 'Box','off')
    lgd1 = legend(hdls,legend_lbl,'location','southeast');
    title(lgd1,'Airspeed') % add legend title
    
    % copy axes 
    Ax(2) = copyobj(Ax(1),gcf);
    delete(get(Ax(2), 'Children') )
    
    % plot helper data, but invisible
    hold on
    H1 = plot([NaN NaN],[NaN NaN], '-', 'LineWidth', 1, 'Color', [0 0 0], 'Parent', Ax(2));
    H2 = plot([NaN NaN],[NaN NaN], '--', 'LineWidth', 1, 'Color', [0 0 0], 'Parent', Ax(2));
    hold off
    % make second axes invisible
    set(Ax(2), 'Color', 'none', 'XTick', [], 'YAxisLocation', 'right', 'Box', 'Off', 'Visible', 'off')
    % add linestyle legend
    lgd2 = legend([H1 H2], 'Tomaso', 'Frédéric', 'Location', 'northwest');
    title(lgd2,'Fit Source') % add legend title
    set(lgd2,'color','none')

xlabel('Angle of Attack [deg]')
ylabel('Force [N]')
grid on

title('Elevator z-axis')

% Difference between models
figure
subplot(1,2,1)
plot(rad2deg(theta_list),Fz_ele_TO(u_list(select),theta_list)-Fz_elevator(theta_list,u_list(select),0))
xlabel('Angle of attack [deg]')
ylabel('Error (T-F) [N]')
subplot(1,2,2)
plot(rad2deg(theta_list),Fz_ele_TO(u_list(select),theta_list)-Fz_elevator(theta_list,u_list(select),0))
xlabel('Angle of attack [deg]')
ylabel('Error (T-F) [%]')
ylim([-1 1])
sgtitle('Difference between models: Fz Elevator')
