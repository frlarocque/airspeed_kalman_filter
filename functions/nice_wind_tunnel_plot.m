function nice_wind_tunnel_plot(kalman_res,airspeed_pitot)
[hover_logical transition_logical ff_logical] = identify_hover_transition_ff(kalman_res.u(12,:));
hover_start = kalman_res.t(diff(hover_logical)==1);
hover_end = kalman_res.t(diff(hover_logical)==-1);
if hover_start<hover_end
    hover_start = [kalman_res.t(1) hover_start];
end
transition_start = kalman_res.t(diff(transition_logical)==1);
transition_end = kalman_res.t(diff(transition_logical)==-1);
ff_start = kalman_res.t(diff(ff_logical)==1);
ff_end = kalman_res.t(diff(ff_logical)==-1);

figure
ax1=subplot(2,2,1);
hold on
s1 = plot(kalman_res.t,kalman_res.x(1,:));
s2 = plot(airspeed_pitot.time,airspeed_pitot.data,'--');
title('Airspeed')
xlabel('Time [s]')
ylabel('Speed [m/s]')
grid on
legend([s1,s2],'Kalman','Pitot')

ax2 = subplot(2,2,2);
plot(kalman_res.t,kalman_res.x([4:6],:));
title('Wind')
xlabel('Time [s]')
ylabel('Speed [m/s]')
legend('Kalman N','Kalman E','Kalman D')
grid on

ax3 = subplot(2,2,3);
plot(kalman_res.t,rad2deg(kalman_res.u(12,:)));
hold on
title('Skew')
xlabel('Time [s]')
ylabel('Skew Angle [deg]')
grid on
axis([-inf inf -5 100])

lineStyles = linspecer(3);

for i=1:length(kalman_res.t)-1
    if hover_logical(i)
        color = lineStyles(1,:);
        s1 = fill([kalman_res.t(i) kalman_res.t(i) kalman_res.t(i+1) kalman_res.t(i+1)],rad2deg([0 kalman_res.u(12,i),kalman_res.u(12,i+1) 0]),color,'EdgeColor','none','FaceAlpha',0.5);
    elseif transition_logical(i)
        color = lineStyles(2,:);
        s2 = fill([kalman_res.t(i) kalman_res.t(i) kalman_res.t(i+1) kalman_res.t(i+1)],rad2deg([0 kalman_res.u(12,i),kalman_res.u(12,i+1) 0]),color,'EdgeColor','none','FaceAlpha',0.5);
    elseif ff_logical(i)
        color = lineStyles(3,:);
        s3 = fill([kalman_res.t(i) kalman_res.t(i) kalman_res.t(i+1) kalman_res.t(i+1)],rad2deg([0 kalman_res.u(12,i),kalman_res.u(12,i+1) 0]),color,'EdgeColor','none','FaceAlpha',0.5);
    else
        
    end
    
end
legend([s1 s2 s3],'Hover','Transition','Forward Flight')

ax4 = subplot(2,2,4);
plot(kalman_res.t,kalman_res.u(10,:));
hold on
plot(kalman_res.t,kalman_res.u(11,:));
title('Motor Commands')
xlabel('Time [s]')
ylabel('PWM')
legend('Pusher','Hover Mean')
grid on
axis([-inf inf 900 2100])

linkaxes([ax1,ax2,ax3,ax4],'x')

if ~isfield(kalman_res, 'error')
    warning('Required Field Not Present')
else  
    sgtitle(sprintf('Error RMS\nOverall %2.2f | Hover %2.2f | Transition %2.2f | FF %2.2f\n',kalman_res.error.error_RMS,kalman_res.error.hover.error_RMS,kalman_res.error.transition.error_RMS,kalman_res.error.ff.error_RMS))

end

end