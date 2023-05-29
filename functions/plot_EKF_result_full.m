function plot_EKF_result_full(kalman_res,airspeed_pitot,beta,alpha,wind,cutoff_freq)
%PLOT_EKF_RESULTS Plot results of EKF 



if nargin==6
    dt = mean(diff(kalman_res.t));
    [b,a] = butter(2,2*cutoff_freq*dt,'low');
    kalman_res.x = filtfilt(b,a,kalman_res.x')';
    kalman_res.u = filtfilt(b,a,kalman_res.u')';
    kalman_res.y = filtfilt(b,a,kalman_res.y')';
    kalman_res.z = filtfilt(b,a,kalman_res.z')';
end

figure
ax1=subplot(3,2,1);
hold on
s1 = plot(kalman_res.t,kalman_res.x(1,:));
s2 = plot(airspeed_pitot.time,airspeed_pitot.data,'--');
title('Airspeed Estimation Kalman')
xlabel('Time [s]')
ylabel('Speed [m/s]')
legend('Kalman','Pitot')
grid on
if isfield(airspeed_pitot,'valid_times')
    subplot(3,2,1)
    for i=1:length(airspeed_pitot.valid_times)
        P = patch([airspeed_pitot.valid_times{i}(1) airspeed_pitot.valid_times{i}(1) airspeed_pitot.valid_times{i}(2) airspeed_pitot.valid_times{i}(2)],...
              [min(airspeed_pitot.data),max(airspeed_pitot.data),max(airspeed_pitot.data),min(airspeed_pitot.data)],'green','LineStyle',"none",'FaceAlpha',.1);
        uistack(P, 'bottom');
    end
    
    legend([s1 s2 P],'Kalman Estimation','Measured Airspeed','Valid Pitot Tube')
end

ax2 = subplot(3,2,2);
plot(kalman_res.t,kalman_res.x([4:6],:));
hold on
plot(wind.raw.time,ones(length(wind.raw.time),1)*wind.vect(:,[1:2]),'--')
title('Wind mu')
xlabel('Time [s]')
ylabel('Speed [m/s]')
legend('Kalman N','Kalman E','Kalman D','Mean Real N','Mean Real E')
grid on

ax3 = subplot(3,2,3);
plot(kalman_res.t,kalman_res.x([1:3],:));
title('Velocity Body Axis')
xlabel('Time [s]')
ylabel('Speed [m/s]')
legend('u_b','v_b','w_b')
grid on

ax4 = subplot(3,2,4);
plot(kalman_res.t,kalman_res.z(1:3,:)-kalman_res.y(1:3,:));
hold on
plot(kalman_res.t,kalman_res.z(1:3,:),'--')
plot(kalman_res.t,vecnorm(kalman_res.z(1:3,:)-kalman_res.y(1:3,:)))
title('Ground Velocity')
xlabel('Time [s]')
ylabel('Speed [m/s]')
legend('Kalman V_N','Kalman V_E','Kalman V_D','Real V_N','Real V_E','Real V_D','V_{gnd}')
grid on

ax5 = subplot(3,2,5);
plot(kalman_res.t,rad2deg(asin(kalman_res.x(2,:)./vecnorm(kalman_res.x(1:3,:),2,1))));
hold on
plot(beta.time,rad2deg(beta.data),'--')
title('Sideslip Angle')
xlabel('Time [s]')
ylabel('Sideslip [deg]')
legend('Kalman','Best estimation')
grid on
axis([-inf inf -50 50])

ax6 = subplot(3,2,6);
plot(kalman_res.t,rad2deg(atan2(kalman_res.x(3,:),kalman_res.x(1,:))));
hold on
plot(alpha.time,rad2deg(alpha.data),'--')
title('Angle of attack')
xlabel('Time [s]')
ylabel('Angle of Attack [deg]')
legend('Kalman','Best estimation')
grid on
axis([-inf inf -50 50])

linkaxes([ax1,ax2,ax3,ax4,ax5,ax6],'x')

if ~isfield(kalman_res, 'error')
    warning('Required Field Not Present')
else  
    sgtitle(sprintf('Wind Covariance %.1d | RMS error %.2f',kalman_res.Q{end}(end,end),kalman_res.error.valid_pitot.error_RMS))
end

figure
ax1 = subplot(2,3,1);
ax2 = subplot(2,3,2);
ax3 = subplot(2,3,3);
if size(kalman_res.u,1)>=9
    ax1 = subplot(2,3,1);
    plot(kalman_res.t,kalman_res.u(10,:));
    hold on
    plot(kalman_res.t,kalman_res.u(11,:));
    title('RPM')
    xlabel('Time [s]')
    ylabel('RPM')
    legend('Pusher','Hover Mean')
    grid on

    ax2 = subplot(2,3,2);
    plot(kalman_res.t,rad2deg(kalman_res.u(12,:)));
    hold on
    title('Skew')
    xlabel('Time [s]')
    ylabel('Skew Angle [deg]')
    grid on
    axis([-inf inf 0 90])

    ax3 = subplot(2,3,3);
    plot(kalman_res.t,rad2deg(elevator_pprz2angle(kalman_res.u(13,:))));
    hold on
    title('Elevator')
    xlabel('Time [s]')
    ylabel('Angle [deg]')
    grid on

end

ax4 = subplot(2,3,4);
ax5 = subplot(2,3,5);
ax6 = subplot(2,3,6);

if size(kalman_res.z,1)>3
    ax4 = subplot(2,3,4);
    plot(kalman_res.t,kalman_res.z(4,:)-kalman_res.y(4,:));
    hold on
    plot(kalman_res.t,kalman_res.z(4,:),'--')
    title('Measurement 4')
    xlabel('Time [s]')
    ylabel('Measurement 4')
    legend('Kalman','Measurement')
    grid on
    
    if size(kalman_res.z,1)>4
        ax5 = subplot(2,3,5);
        plot(kalman_res.t,kalman_res.z(5,:)-kalman_res.y(5,:));
        hold on
        plot(kalman_res.t,kalman_res.z(5,:),'--')
        title('Measurement 5')
        xlabel('Time [s]')
        ylabel('Measurement 5')
        legend('Kalman','Measurement')
        grid on

        if size(kalman_res.z,1)>5
            
            ax6 = subplot(2,3,6);
            plot(kalman_res.t,kalman_res.z(6,:)-kalman_res.y(6,:));
            hold on
            plot(kalman_res.t,kalman_res.z(6,:),'--')
            title('Measurement 6')
            xlabel('Time [s]')
            ylabel('Measurement 6')
            legend('Kalman','Measurement')
            grid on
        end
    end
end
linkaxes([ax1,ax2,ax3,ax4,ax5,ax6],'x')


if ~isfield(kalman_res, 'error')
    warning('Required Field Not Present')
else  
    sgtitle(sprintf('Wind Covariance %.1d | RMS error %.2f',kalman_res.Q{end}(end,end),kalman_res.error.valid_pitot.error_RMS))
end

end