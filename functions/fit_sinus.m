function [amplitude,period,phase,offset] = fit_sinus(x,y,graph)
%FIT_SINUS Fits the best sinus to a signal
%
% Fits using the following model:
% f(x) = b(1).*(sin(2*pi*x./b(2) + 2*pi/b(3))) + b(4)
%
% Inputs: 
%           -x: time or x component of signal
%           -y signal
%           -graph: 1 to show graph, 0 to hide
%
% Outputs:
%           -amplitude: amplitude of fitted sinus in units of y
%           -period: period of fitted sinus in units of x
%           -phase: phase of fitted sinus in units of x
%           -offset: offset of fitter sinus in units of y

% Estimate parameters of best sinus
yu = max(y);
yl = min(y);
yr = (yu-yl);                               % Range of ‘y’
yz = y-yu+(yr/2);
zci = @(v) find(diff(sign(v)));
zx = x(zci(yz));
per = 2*mean(rmoutliers(diff(zx)));                     % Estimate period
if isnan(per) || isinf(per)
    per = 2*pi;
end
ym = mean(y);                               % Estimate offset
fit = @(b,x)  b(1).*(sin(2*pi*x./b(2) + 2*pi/b(3))) + b(4);    % Function to fit
fcn = @(b) sum((fit(b,x) - y).^2);                              % Least-Squares cost function
s = fminsearch(fcn, [yr;  per;  -1;  ym]);                       % Minimise Least-Squares


% f(x) = b(1).*(sin(2*pi*x./b(2) + 2*pi/b(3))) + b(4)
amplitude = s(1); % in units of y
period = s(2); % in units of x
phase = s(2)/(2*s(3)); % in units of x
offset = s(4); %in units of y

if graph
    xp = linspace(min(x),max(x));
    figure
    plot(x,y,'b',  xp,fit(s,xp), 'r--')
    legend('Signal','Fit')
    grid
end

end