function [ particles, weights ] = resample( old_p, old_w )
%RESAMPLE particles based on weights to determine a new set of samples
%based on weights

n = numel(old_w);

particles = old_p(:,randsample(1:n, n, true, old_w));
weights = 1/n * ones(n,1);


end

