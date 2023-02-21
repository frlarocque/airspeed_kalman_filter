function [error] = error_quantification(obtained,desired)
%ERROR_QUANTIFICATION Calculates different types of error between two
%signals
%
% Calculates relative error, absolute error, RMS error

error.rel_error = (obtained-desired)./desired;
error.rel_error(isinf(error.rel_error))=0;
error.rel_max = max(error.rel_error);
error.rel_min = min(error.rel_error);
error.rel_mean= mean(error.rel_error);

error.error = obtained-desired;
error.error_RMS = sqrt(mean(error.error.^2));
error.error_mean = mean(error.error);
error.error_max = max(error.error);
error.error_min = min(error.error);
error.std_dev = std(error.error);

end