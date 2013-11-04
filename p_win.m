function [ x,dist ] = p_win( data_pts, weights, h1, MIN, MAX )
%P_WIN PARZEN WINDOW density estimation - Gaussian Kernel
%Expect data_pts to be 1D
%h1 is my free parameter

hn = h1/sqrt(numel(data_pts));

RANGE = [max(min(data_pts)-7*hn, MIN) min(max(data_pts)+7*hn,MAX)];
% RANGE = [-3, 3];

x = [RANGE(1):.1:RANGE(2)]';
dist = zeros(size(x));

inv_const = 1/(sqrt(2*pi)*hn);

for i=1:numel(x)

    idx = find(data_pts>x(i)-7*hn);
    consider_pts = data_pts(idx);
    temp_weights = weights(idx);
    idx = find(consider_pts<x(i)+7*hn);
    temp_weights = temp_weights(idx);
    consider_pts = consider_pts(idx);
%     
%      consider_pts = data_pts;
%      temp_weights = weights;
     diffs = temp_weights.*exp((((x(i) - consider_pts)./hn).^2)./-2);
%     diffs = exp((((x(i) - consider_pts)./hn).^2)./-2);
    dist(i) = sum(inv_const .*diffs);
end

%dist = 1/sum(weights) .* dist;

% plot(x,dist)

end

