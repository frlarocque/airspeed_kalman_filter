function nice_plot_residual_hist(y_res,bin_n,dt,fit_norm,filt_freq)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
if nargin<5
    filt_freq = 0;
end

% Filter signal
if filt_freq~=0
    [b,a] = butter(2,2*filt_freq*dt,'low');
    for i=1:size(y_res,2)
        y_res(:,i) = filtfilt(b,a,y_res(:,i));
    end
end


% Discard everything bigger than X var
outlier_thresh = 7;
v_x_res = remove_outlier(y_res(:,1),outlier_thresh);
v_y_res = remove_outlier(y_res(:,2),outlier_thresh);
v_z_res = remove_outlier(y_res(:,3),outlier_thresh);
accel_x_res = remove_outlier(y_res(:,4),outlier_thresh);
accel_y_res = remove_outlier(y_res(:,5),outlier_thresh);
accel_z_res = remove_outlier(y_res(:,6),outlier_thresh);
pitot_res = remove_outlier(y_res(:,7),outlier_thresh);
pitot_res_diff = remove_outlier(diff(y_res(:,7))./dt,outlier_thresh);
%pitot_res_diff(abs(pitot_res_diff)>40) = [];

if nargin==2
    fit_norm = false;
end

%% Figure setup
set(gcf, 'Renderer', 'Painters');
AR = 2;
fig_size = 500;
font_size = 20;

fig_height = fig_size;
fig_width = fig_height*AR;

screen = get(0, 'ScreenSize');

if fig_width>screen(3)
    fig_width = screen(3);
    fig_height = fig_width/AR;
end
fprintf('Exporting as %.0fx%.0f \n',fig_width,fig_height);

% Get the current date and time
nowDateTime = datetime('now');

% Format the date and time in the "MM_DD_HH_MM" format
formattedDateTime = datestr(nowDateTime,'mm_dd_HH_MM');

fig = figure('position',[0 0 fig_width fig_height]);

% Store the default line width value
origLineWidth = get(groot, 'DefaultLineLineWidth');

% Set a new default line width value
set(groot, 'DefaultLineLineWidth', 2);

% Set colors and line styles
mycolors = linspecer(3,'qualitative');
mylinestyles = {'-', '--', ':'};
set(gcf,'DefaultAxesColorOrder',mycolors, ...
        'DefaultAxesLineStyleOrder',mylinestyles)

%%
% figure;
% subplot(2,1,1)
% hold on
% plot_hist(v_x_res,bin_n)
% plot_hist(v_y_res,bin_n)
% plot_hist(v_z_res,bin_n)
% var_v = max([var(v_x_res),var(v_y_res),var(v_z_res)]);
% %axis([-5.*var_v 5.*var_v -inf inf])
% max_v = max([max(abs(v_x_res)),max(abs(v_y_res)),max(abs(v_z_res))]);
% xlim([-max_v, max_v]);
% legend('V_x','V_y','V_z')
% ylabel('Probability Density Function')
% xlabel('Innovation Value')
% 
% subplot(2,1,2)
% hold on
% plot_hist(accel_x_res,bin_n)
% plot_hist(accel_y_res,bin_n)
% plot_hist(accel_z_res,bin_n)
% var_accel = max([var(accel_x_res),var(accel_y_res),var(accel_z_res)]);
% %axis([-3.*var_accel 3.*var_accel -inf inf])
% max_a = max([max(abs(accel_x_res)),max(abs(accel_y_res)),max(abs(accel_z_res))]);
% xlim([-max_a, max_a]);
% legend('A_x','A_y','A_z')
% ylabel('Probability Density Function')
% xlabel('Innovation Value')
% 
% line_width = 2;
% if fit_norm
%    subplot(2,1,1)
%    data = v_x_res;
%    x = linspace(min(data),max(data),500); 
% 
%    % Gaussian Fit
%    [params_fit, gaussian_fn] = fit_gaussian(data);
%    y = gaussian_fn(x,params_fit(1),params_fit(2));
%    plot(x,y,'LineWidth',line_width);
%    str1 = sprintf('Speed Gaussian fit mu = %2.2e sigma = %2.2e',params_fit(1),params_fit(2));
%     
%    % Cauchy fit
%    [params_fit, cauchy_fn] = fit_cauchy(data);
%     y = cauchy_fn(x, params_fit(1), params_fit(2));
%     plot(x,y,'linewidth',line_width);
%     str2 = sprintf('Speed Cauchy fit mu = %2.2e gamma = %2.2e',params_fit(1), params_fit(2));
%     
%    legend('V_x','V_y','V_z','Gaussian Fit','Cauchy Fit')
%    title([str1,' ',str2])
% 
%    subplot(2,1,2)
%    data = accel_x_res;
%    x = linspace(min(data),max(data),500); 
% 
%    % Gaussian Fit
%    [params_fit, gaussian_fn] = fit_gaussian(data);
%    y = gaussian_fn(x,params_fit(1),params_fit(2));
%    plot(x,y,'linewidth',line_width);
%    str1 = sprintf('Acceleration Gaussian fit mu = %2.2e sigma = %2.2e',params_fit(1),params_fit(2));
%     
%    % Cauchy fit
%    [params_fit, cauchy_fn] = fit_cauchy(data);
%     y = cauchy_fn(x, params_fit(1), params_fit(2));
%     plot(x,y,'linewidth',line_width);
%     str2 = sprintf('Acceleration Cauchy fit mu = %2.2e gamma = %2.2e',params_fit(1), params_fit(2));
%     
%    legend('A_x','A_y','A_z','Gaussian Fit','Cauchy Fit')
%    title([str1,' ',str2])
% end
% %linkaxes([subplot(3,1,1) subplot(3,1,2) subplot(3,1,3)],'x')
% %max_var = max(var(y));
% %axis([-2.*max_var 2.*max_var -inf inf])

%%

ax1=subplot(1,1,1);
data = pitot_res;

hold on
plot_hist(data,bin_n)
ax1.ColorOrderIndex = 1;
var_pitot = var(data);
xlim([-7, 7]);
%legend('Pitot')
ylabel('Probability Density Function')
xlabel('Airspeed Residual [m/s]')

x = linspace(min(data),max(data),500); 

% Gaussian Fit
[params_fit, gaussian_fn] = fit_gaussian(data);
y = gaussian_fn(x,params_fit(1),params_fit(2));
p1 = plot(x,y);
ax1.LineStyleOrderIndex = ax1.ColorOrderIndex;
str1 = sprintf('Pitot Gaussian fit mu = %2.2e sigma = %2.2e',params_fit(1),params_fit(2));

% Cauchy fit
[params_fit, cauchy_fn] = fit_cauchy(data);
y = cauchy_fn(x, params_fit(1), params_fit(2));
p2 = plot(x,y);
str2 = sprintf('Pitot Cauchy fit mu = %2.2e gamma = %2.2e',params_fit(1), params_fit(2));

grid on
legend([p1,p2],{'Gaussian Fit','Cauchy Fit'})
fprintf([str1,' ',str2])

% Change font size
set(findall(gcf,'-property','FontSize'),'FontSize',font_size)

% Export
fig_name = ['residual_pitot_',formattedDateTime,'.eps'];
exportgraphics(fig,fig_name,'BackgroundColor','none','ContentType','vector')

clf
ax1 = subplot(1,1,1);
data = pitot_res_diff;

hold on
plot_hist(data,bin_n)
ax1.ColorOrderIndex = 1;
var_pitot = var(data);
xlim([-20, 20]);
%legend('Pitot')
ylabel('Probability Density Function')
xlabel('Airspeed Residual Derivative [m/s²]')

x = linspace(min(data),max(data),500); 

% Gaussian Fit
[params_fit, gaussian_fn] = fit_gaussian(data);
y = gaussian_fn(x,params_fit(1),params_fit(2));
p1 = plot(x,y);
ax1.LineStyleOrderIndex = ax1.ColorOrderIndex;
str1 = sprintf('Pitot Gaussian fit mu = %2.2e sigma = %2.2e',params_fit(1),params_fit(2));

% Cauchy fit
[params_fit, cauchy_fn] = fit_cauchy(data);
y = cauchy_fn(x, params_fit(1), params_fit(2));
p2 = plot(x,y);
str2 = sprintf('Pitot Cauchy fit mu = %2.2e gamma = %2.2e',params_fit(1), params_fit(2));

grid on
legend([p1,p2],{'Gaussian Fit','Cauchy Fit'})
fprintf([str1,' ',str2])

% Change font size
set(findall(gcf,'-property','FontSize'),'FontSize',font_size)

% Export
fig_name = ['residual_pitot_diff_',formattedDateTime,'.eps'];
exportgraphics(fig,fig_name,'BackgroundColor','none','ContentType','vector')

    function plot_hist(residuals,bin_n)
        bin_edges = linspace(min(residuals),max(residuals),bin_n);
        counts = histcounts(residuals,bin_edges);
        pdf_hist = counts./(sum(counts).*mean(diff(bin_edges)));
    
        histogram('BinEdges',bin_edges,'BinCounts',pdf_hist,'FaceAlpha',0.2)
        hold on

    end

    function residual_clean = remove_outlier(residuals,factor)
        var_res = var(residuals);
        residual_clean = residuals(abs(residuals)<var_res*factor);
    end

    function [params_fit, cauchy_pdf] = fit_cauchy(data)
        % Define the Cauchy probability density function
        cauchy_pdf = @(x, mu, gamma) (1/pi) .* (gamma ./ ((x - mu).^2 + gamma^2));
        
        % Define the negative log-likelihood function
        negloglik = @(params) -sum(log(cauchy_pdf(data, params(1), params(2))));
        
        % Perform the optimization
        params0 = [0, 1];  % Starting values for the parameters mu and gamma
        params_fit = fminsearch(negloglik, params0);
    end
    

    function [params_fit, gaussian_pdf] = fit_gaussian(data)
        gaussian_pdf = @(x, mu, sigma) 1/(sigma*sqrt(2*pi)) * exp(-(x-mu).^2/(2*sigma^2));
        
        bin_centers = linspace(min(data), max(data), 100);
        bin_width = bin_centers(2) - bin_centers(1);
        bin_edges = [bin_centers - bin_width/2, bin_centers(end) + bin_width/2*1e-6]; % Add an extra bin interval
        sse = @ (params) sum((gaussian_pdf(bin_centers,params(1),params(2)).* numel(data) .* bin_width - histcounts(data, bin_edges)).^2);

        params0 = [0 1];
        params_fit = fminsearch(sse, params0);
        
    end

end

