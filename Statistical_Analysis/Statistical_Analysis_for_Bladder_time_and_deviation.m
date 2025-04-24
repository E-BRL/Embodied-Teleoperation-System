%%
clear all;
clc;
f_time_raw = [240.0025497, 253.3675607, 246.5560273, 173.072789, 146.180753, ...
              130.5578457, 164.4246867, 295.3131277, 103.3893553, 213.2092253, 237.062337]';

s_time_raw = [244.1279713, 311.6115067, 239.4211997, 233.9615933, 213.494347, ...
              154.1010107, 147.1086857, 284.2720867, 218.978681, 242.9610723, 185.8444817]';

f_time = f_time_raw;
% subject 11 제외
f_time([11], :) = [];
s_time = s_time_raw;
% subject 11 제외
s_time([11], :) = [];

%%
clear all;
clc;
f_deviation_raw = [32.7717802, 48.31096797, 56.39411448, 37.55550248, 14.42525484, ...
               27.81211829, 31.15049588, 38.85709469, 16.90646719, 40.30861972, 41.29744096]';

s_deviation_raw = [31.95482928, 37.36736692, 43.66749731, 42.04091359, 27.67167812, ...
               30.1874117, 40.06083696, 44.40596746, 33.81878847, 45.05566435, 25.3207088]';


f_deviation = f_deviation_raw;
% subject 11 제외
f_deviation([11], :) = [];
s_deviation = s_deviation_raw;
% subject 11 제외
s_deviation([11], :) = [];

%%
if exist('f_deviation', 'var') && exist('s_deviation', 'var')
    f_data = f_deviation;
    s_data = s_deviation;
    data_type = 'deviation';
elseif exist('f_time', 'var') && exist('s_time', 'var')
    f_data = f_time;
    s_data = s_time;
    data_type = 'time';
else
    error('f_time/s_time 또는 f_deviation/s_deviation 중 하나가 정의되어야 합니다.');
end

combined_data = [f_data, s_data]; 
all_data = [f_data; s_data];

% y축 및 제목 설정 자동화
switch data_type
    case 'deviation'
        y_label_str = 'Total tip deviation [m]';
        title_str = 'Comparison of Tip Deviation between Finger and Stylus';
        y_lim = [0 65];
        y_ticks = 0:10:60;
        colorbar_label = 'Deviation difference [m]';
        file_suffix = 'deviation';
    case 'time'
        y_label_str = 'Trial time [sec]';
        title_str = 'Comparison of Trial Time between Finger and Stylus';
        y_lim = [25 360];
        y_ticks = 50:25:350;
        colorbar_label = 'Trial time difference [sec]';
        file_suffix = 'time';
end
%% ===== 통계 분석 자동 처리 =====

% 공통: slope 기반 차이 계산
differences = f_data - s_data;

% 1️⃣ 기본 통계량
cv1 = std(s_data) / mean(s_data) * 100;
cv2 = std(f_data) / mean(f_data) * 100;
fprintf('Stylus CV: %.2f%%\n', cv1);
fprintf('Finger  CV: %.2f%%\n', cv2);

iqr1 = iqr(s_data);
iqr2 = iqr(f_data);
fprintf('Stylus IQR: %.2f\n', iqr1);
fprintf('Finger  IQR: %.2f\n', iqr2);

% 2️⃣ Shapiro-Wilk 정규성 검정
[H_sw, p_sw] = swtest(differences);
fprintf('Shapiro-Wilk Test for Differences: H = %d, p = %.4f\n', H_sw, p_sw);

% 3️⃣ QQ plot
figure;
h = qqplot(differences);
set(h(1), 'Marker', 'd', 'MarkerSize', 9, 'MarkerEdgeColor', 'k','MarkerFaceColor', 'b');
set(h(2), 'LineWidth', 2, 'Color', 'r');
set(h(3), 'LineWidth', 1.5, 'LineStyle', '--', 'Color', [0.5 0.5 0.5]);
xlabel('Theoretical Quantiles', 'FontSize', 12);
ylabel('Sample Quantiles', 'FontSize', 12);
title(['QQ Plot of ' upper(data_type(1)) data_type(2:end) ' Differences'], 'FontSize', 14);
grid on;
saveas(gcf, ['qqplot_' file_suffix '.png']);

% 4️⃣ 대응표본 t-test
[H_t, p_t, ci_t, stats_t] = ttest(f_data, s_data);
fprintf('\nPaired t-test:\n');
fprintf('  t-stat: %.4f | df: %d | p = %.4f\n', stats_t.tstat, stats_t.df, p_t);
fprintf('  95%% CI: [%.4f, %.4f]\n', ci_t(1), ci_t(2));

if p_t < 0.05
    disp('  → 유의한 차이 있음 (p < 0.05)');
else
    disp('  → 유의한 차이 없음 (p ≥ 0.05)');
end

% 5️⃣ Cohen''s d 및 CI
[d, ci_d] = cohens_d_ci(f_data, s_data);
fprintf('Cohen''s d = %.2f, 95%% CI [%.2f, %.2f]\n', d, ci_d(1), ci_d(2));

% 6️⃣ Wilcoxon 부호 순위 검정
[p_w, h_w, stats_w] = signrank(f_data, s_data);
[Z, r_w] = wilcoxon_r(f_data, s_data);
fprintf('\nWilcoxon Signed-Rank Test:\n');
fprintf('  p = %.4f | Z = %.2f | effect size r = %.2f\n', p_w, Z, abs(r_w));

% 7️⃣ Bootstrap 기반 r CI
[r_boot, ci_r] = bootstrap_r_ci(f_data, s_data, 1000, 0.05);
fprintf('Bootstrap r = %.2f, 95%% CI [%.2f, %.2f]\n', r_boot, ci_r(1), ci_r(2));

%%
%%
function [d, ci] = cohens_d_ci(x, y, alpha)
    if nargin < 3
        alpha = 0.05; % 95% CI
    end

    diffs = x - y;
    n = length(diffs);
    mean_diff = mean(diffs);
    std_diff = std(diffs); % N-1

    % Cohen's d
    d = mean_diff / std_diff;

    % 표준오차 (SE)
    se_d = sqrt((1/n) + (d^2 / (2*n)));

    % 신뢰구간 계산 (정규근사 기반)
    z = norminv(1 - alpha/2);
    ci = [d - z * se_d, d + z * se_d];
end

%%
function [Z, r] = wilcoxon_r(x, y)
% WILCOXON_R calculates the Z-score and effect size r for Wilcoxon signed-rank test
% Inputs:
%   x, y - paired samples
% Outputs:
%   Z - Z-score based on normal approximation
%   r - effect size

    if length(x) ~= length(y)
        error('Input vectors x and y must have the same length.');
    end

    % Remove NaN pairs
    valid = ~isnan(x) & ~isnan(y);
    x = x(valid);
    y = y(valid);
    
    n = length(x);  % number of valid pairs

    if n < 1
        error('No valid data pairs remain after NaN removal.');
    end

    % Run Wilcoxon signed-rank test
    [~, ~, stats] = signrank(x, y);

    % Get W (signed rank sum)
    W = stats.signedrank;

    % Compute expected value and standard deviation
    mu = n * (n + 1) / 4;
    sigma = sqrt(n * (n + 1) * (2 * n + 1) / 24);

    % Z-score (normal approximation)
    Z = (W - mu) / sigma;

    % Effect size r
    r = Z / sqrt(n);
end

