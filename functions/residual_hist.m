function [outputArg1,outputArg2] = residual_hist(y_res,bin_n,fit_norm)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

% Discard everything bigger than 2 var
v_x_res = remove_outlier(y_res(:,1),5);
v_y_res = remove_outlier(y_res(:,2),5);
v_z_res = remove_outlier(y_res(:,3),5);
accel_x_res = remove_outlier(y_res(:,4),3);
accel_y_res = remove_outlier(y_res(:,5),3);
accel_z_res = remove_outlier(y_res(:,6),3);
pitot_res = remove_outlier(y_res(:,7),3);

if nargin==2
    fit_norm = false;
end

figure;
subplot(3,1,1)
hold on
plot_hist(v_x_res)
plot_hist(v_y_res)
plot_hist(v_z_res)
var_v = max([var(v_x_res),var(v_y_res),var(v_z_res)]);
axis([-5.*var_v 5.*var_v -inf inf])
legend('V_x','V_y','V_z')
ylabel('Probability Density Function')
xlabel('Innovation Value')


subplot(3,1,2)
hold on
plot_hist(accel_x_res)
plot_hist(accel_y_res)
plot_hist(accel_z_res)
var_accel = max([var(accel_x_res),var(accel_y_res),var(accel_z_res)]);
axis([-3.*var_accel 3.*var_accel -inf inf])
legend('A_x','A_y','A_z')
ylabel('Probability Density Function')
xlabel('Innovation Value')

subplot(3,1,3)
hold on
plot_hist(pitot_res)
var_pitot = var(pitot_res);
axis([-3.*var_pitot 3.*var_pitot -inf inf])
legend('Pitot')
ylabel('Probability Density Function')
xlabel('Innovation Value')

%linkaxes([subplot(3,1,1) subplot(3,1,2) subplot(3,1,3)],'x')
%max_var = max(var(y));
%axis([-2.*max_var 2.*max_var -inf inf])

if fit_norm
   subplot(3,1,1)
   data = y_res(:,1);
   pd =fitdist(data,'normal');
   x = linspace(min(data),max(data),100);
   y = pdf(pd,x);

   plot(x,y,'LineWidth',1);
   legend('V_x','V_y','V_z','Normal distribution fit')

   subplot(3,1,2)
   data = y_res(:,4);
   pd =fitdist(data,'normal');
   x = linspace(min(data),max(data),100);
   y = pdf(pd,x);

   plot(x,y,'LineWidth',1);
   legend('A_x','A_y','A_z','Normal distribution fit')

   subplot(3,1,3)
   data = y_res(:,7);

    % Define the Cauchy probability density function
    cauchy_pdf = @(x, mu, gamma) (1/pi) .* (gamma ./ ((x - mu).^2 + gamma^2));
    
    % Define the negative log-likelihood function
    negloglik = @(params) -sum(log(cauchy_pdf(data, params(1), params(2))));
    
    % Perform the optimization
    params0 = [0, 1];  % Starting values for the parameters mu and gamma
    params_fit = fminsearch(negloglik, params0);

   x = linspace(min(data),max(data),500);
   y = cauchy_pdf(x, params_fit(1), params_fit(2));

   plot(x,y,'LineWidth',1);
   legend('Pitot','Cauchy distribution fit')

end

    function plot_hist(residuals)
        bin_edges = linspace(min(residuals),max(residuals),bin_n);
        counts = histcounts(residuals,bin_edges);
        pdf_hist = counts./(sum(counts).*mean(diff(bin_edges)));
    
        histogram('BinEdges',bin_edges,'BinCounts',pdf_hist)
        hold on

    end

    function residual_clean = remove_outlier(residuals,factor)
        var_res = var(residuals);
        residual_clean = residuals(abs(residuals)<var_res*factor);
    end

end

