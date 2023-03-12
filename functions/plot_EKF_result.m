function plot_EKF_result(kalman_res,airspeed_estimation,airspeed_pitot,wind)
%PLOT_EKF_RESULTS Plot results of EKF 

figure
ax1=subplot(3,1,1);
s1 = plot(airspeed_estimation.time,airspeed_estimation.data);
hold on
s2 = plot(kalman_res.t,kalman_res.x(1,:));
s3 = plot(airspeed_pitot.time,airspeed_pitot.data);
title('Airspeed Estimation Kalman')
xlabel('Time [s]')
ylabel('Speed [m/s]')
legend('Estimation Knowing wind','Kalman Estimation','Measured Airspeed')
grid on

ax2 = subplot(3,1,2);
plot(kalman_res.t,kalman_res.x([4:6],:));
hold on
plot(wind.raw.time,ones(length(wind.raw.time),1)*wind.vect(:,[1:2]),'--')
%plot(wind.raw.time,wind.raw.data(:,[1:2]),'--')
title('Wind mu')
xlabel('Time [s]')
ylabel('Speed [m/s]')
legend('Kalman N','Kalman E','Kalman D','Mean Real N','Mean Real E')
%legend('Kalman N','Kalman E','Kalman D','Mean Real N','Mean Real E','Real N','Real E')
grid on

ax3 = subplot(3,1,3);
plot(kalman_res.t,kalman_res.x([1:3],:));
title('Velocity Body Axis')
xlabel('Time [s]')
ylabel('Speed [m/s]')
legend('u_b','v_b','w_b')
grid on
linkaxes([ax1,ax2,ax3],'x')

if isfield(airspeed_pitot,'valid_times')
    subplot(3,1,1)
    for i=1:length(airspeed_pitot.valid_times)
        P = patch([airspeed_pitot.valid_times{i}(1) airspeed_pitot.valid_times{i}(1) airspeed_pitot.valid_times{i}(2) airspeed_pitot.valid_times{i}(2)],...
              [min(airspeed_pitot.data),max(airspeed_pitot.data),max(airspeed_pitot.data),min(airspeed_pitot.data)],'green','LineStyle',"none",'FaceAlpha',.1);
        uistack(P, 'bottom');
    end
    
    legend([s1 s2 s3 P],'Estimation Knowing wind','Kalman Estimation','Measured Airspeed','Valid Pitot Tube')
end



sgtitle(sprintf('Wind Covariance %.1d | RMS error %.2f',kalman_res.Q(end,end),kalman_res.error.error_RMS))
end