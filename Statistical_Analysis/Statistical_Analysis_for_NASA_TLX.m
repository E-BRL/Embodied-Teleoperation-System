%% ==============================
% 1. 데이터 생성 및 정리 (A 인터페이스만 사용)
% ==============================
%clc; clear; close all;
clear all;
clc;

NASA = readtable('NASA_TLX.csv');
%%
T = NASA(1:10,5:16);
%%
T.Properties.VariableNames = {'MD_f', 'PD_f', 'TD_f','E_f','P_f','F_f','MD_s','PD_s','TD_s','E_s','P_s','F_s'};

%%

%%
f_data = T.F_f;
s_data = T.F_s;


%%
%%%% ===== 통계 분석 자동 처리 =====
clc;
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
% % title(['QQ Plot of ' upper(data_type(1)) data_type(2:end) ' Differences'], 'FontSize', 14);
% grid on;
% saveas(gcf, ['qqplot_' file_suffix '.png']);

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
