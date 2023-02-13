function [error] = error_quantification(obtained,desired)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

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
error.error_var = var(error.error);

end