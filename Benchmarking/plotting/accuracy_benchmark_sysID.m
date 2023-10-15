%% Trajectory Tracking
close all; clear; clc;

% Define RGB colors
purple = [0.5, 0.0, 1.0];
dark_green = [0.0, 0.5, 0.0];
gold = [0.8, 0.6, 0.0];
teal = [0.0, 0.5, 0.5];
orange = [0.9, 0.5, 0.0];

% Define paths for data and figures
path_to_data = '../Benchmarking/data/AccuracySysID_HumanoidLeg';
path_to_figures = '../Benchmarking/figures/AccuracySysID_HumanoidLeg';

% Load the sys id observations
obs = readmatrix('../Benchmarking/data/HumanoidLeg_SysID_data.csv');
t_obs = obs(:, 1);
q_obs = obs(:, 2);
qd_obs = obs(:, 3);
qdd_obs = obs(:, 4);
tau_obs = obs(:, 5);

figure
subplot(4, 1, 1)
plot(t_obs, q_obs, 'Color', 'k', 'LineWidth', 2)
subplot(4, 1, 2)
plot(t_obs, qd_obs, 'Color', purple, 'LineWidth', 2)
subplot(4, 1, 3)
plot(t_obs, qdd_obs, 'Color', dark_green, 'LineWidth', 2)   
subplot(4, 1, 4)
plot(t_obs, tau_obs, 'Color', gold, 'LineWidth', 2)
saveas(gcf, '../Benchmarking/figures/humanoid_leg_sysid.png')


% Load data
data = readmatrix([path_to_data, '.csv']);
t = data(:, 1);
tau_ground_truth = data(:, 2);
tau_pred_unc = data(:, 3);
tau_pred_diag = data(:, 4);
tau_pred_exact = data(:, 5);
q = data(:, 6);

% Plot the actual vs predicted torques
figure
plot(t, tau_ground_truth, 'Color', 'k', 'LineWidth', 3)
hold on
plot(t, tau_pred_exact, '--', 'Color', gold, 'LineWidth', 2)
plot(t, tau_pred_diag, '-.', 'Color', dark_green, 'LineWidth', 2)
plot(t, tau_pred_unc, ':', 'Color', purple, 'LineWidth', 2)
plot(t, q, 'r--', 'LineWidth', 1)
hold off

xlabel('Time (s)', 'Interpreter', 'latex')
ylabel('End-Effector Tracking Error (m)', 'Interpreter', 'latex')
legend({'Ground Truth','C-RNEA', 'Approximate RNEA', 'Unconstrained RNEA'}, 'Location', 'northwest', 'Interpreter', 'latex')
grid on
set(gca, 'FontSize', 14)

saveas(gcf, [path_to_figures, '.png'])
