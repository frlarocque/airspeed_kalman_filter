function plot_residual_fault_detector(t,res_fault_detector)

res = res_fault_detector.res;
count = res_fault_detector.count;
flag = res_fault_detector.flag;
dt = mean(diff(t));


figure;
ax1 = subplot(2,2,1);
s1 = plot(t,res(1,:));
hold on
%s2 = plot(t,res(2,:));
s3 = yline(res_fault_detector.criterias.crit_low,'--','color',"#D95319");yline(-res_fault_detector.criterias.crit_low,'o--','color',"#D95319")
s4 = yline(res_fault_detector.criterias.crit_high,'r--');yline(-res_fault_detector.criterias.crit_high,'r--')
%legend([s1,s2,s3,s4],'Raw Innovation','Low Passed Innovation','Low Threshold','High Threshold')
legend([s1,s3,s4],'Raw Innovation','Low Threshold','High Threshold')
xlabel('Time [s]')
ylabel('Innovation [m/s]')
grid on
axis([-inf inf -1.05*res_fault_detector.criterias.crit_high 1.05*res_fault_detector.criterias.crit_high])

ax2 = subplot(2,2,2);
s1 = plot(t,res(3,:));
s2 = yline(res_fault_detector.criterias.crit_diff,'r--');yline(-res_fault_detector.criterias.crit_diff,'r--');
legend([s1,s2],'Innovation Derivative','Threshold')
xlabel('Time [s]')
ylabel('Innovation Derivative [m/s /s]')
grid on
axis([-inf inf -1.05*res_fault_detector.criterias.crit_diff 1.05*res_fault_detector.criterias.crit_diff])

ax3 = subplot(2,2,3);
s1 = plot(t,count(1,:)*dt);
hold on
s2 = plot(t,count(2,:)*dt);
s3 = yline(res_fault_detector.criterias.time_low,'--','color',"#D95319");
s4 = yline(res_fault_detector.criterias.time_high,'r--');
legend([s1 s2 s3 s4],'Low Count','High Count','Limit Low','Limit High')
xlabel('Time [s]')
ylabel('Time for fault')
grid on
axis([-inf inf 0 1.1*res_fault_detector.criterias.time_low])

ax4 = subplot(2,2,4);
s1 = plot(t,count(3,:)*dt);
hold on
s2 = yline(res_fault_detector.criterias.time_diff,'r--');
legend([s1 s2],'Derivative Count','Limit')
xlabel('Time [s]')
ylabel('Time for fault')
axis([-inf inf 0 1.1*res_fault_detector.criterias.time_diff])

linkaxes([ax1,ax2,ax3,ax4],'x');

end