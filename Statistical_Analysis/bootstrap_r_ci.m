function [r_obs, ci] = bootstrap_r_ci(x, y, B, alpha)
% Calculates effect size r and bootstrap CI from paired data (Wilcoxon)
% Inputs:
%   x, y    - paired sample data
%   B       - number of bootstrap samples (e.g., 1000)
%   alpha   - significance level (e.g., 0.05)
% Outputs:
%   r_obs   - observed effect size r
%   ci      - [lower, upper] bounds of CI

    if nargin < 4
        alpha = 0.05;
    end
    if nargin < 3
        B = 1000;
    end

    % Remove NaNs
    valid = ~isnan(x) & ~isnan(y);
    x = x(valid);
    y = y(valid);
    n = length(x);

    % Original Z & r
    [~, ~, stats] = signrank(x, y);
    W = stats.signedrank;
    mu = n*(n+1)/4;
    sigma = sqrt(n*(n+1)*(2*n+1)/24);
    Z_obs = (W - mu) / sigma;
    r_obs = Z_obs / sqrt(n);

    % Bootstrap
    r_dist = zeros(B, 1);
    for i = 1:B
        idx = randsample(n, n, true); % resample with replacement
        x_b = x(idx);
        y_b = y(idx);

        [~, ~, s] = signrank(x_b, y_b);
        if ~isfield(s, 'signedrank')
            continue  % skip if rank couldn't be computed
        end

        W_b = s.signedrank;
        mu_b = n*(n+1)/4;
        sigma_b = sqrt(n*(n+1)*(2*n+1)/24);
        Z_b = (W_b - mu_b) / sigma_b;
        r_dist(i) = Z_b / sqrt(n);
    end

    % Remove NaNs from failed resamples
    r_dist = r_dist(~isnan(r_dist));

    % Compute CI from bootstrap distribution
    lower = quantile(r_dist, alpha/2);
    upper = quantile(r_dist, 1 - alpha/2);
    ci = [lower, upper];
end