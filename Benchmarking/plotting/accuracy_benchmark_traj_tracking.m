%% Trajectory Tracking
close all; clear; clc;


% Define RGB colors
purple = [0.5, 0.0, 1.0];
dark_green = [0.0, 0.5, 0.0];
gold = [0.8, 0.6, 0.0];

path_to_data = '../Benchmarking/data/AccuracyTT_';
path_to_figures = '../Benchmarking/figures/AccuracyTT_';

data = readmatrix([path_to_data, 'Robots.csv']);

t = unique(data(:, 1));
rows_per_traj = length(t);

ideal_traj = data(1:rows_per_traj, 2:4)';
exact_traj = data(rows_per_traj + 1:2 * rows_per_traj, 2:4)';
diag_traj = data(2 * rows_per_traj + 1:3 * rows_per_traj, 2:4)';
none_traj = data(3 * rows_per_traj + 1:4 * rows_per_traj, 2:4)';

figure
subplot(3,1,1)
plot(t, ideal_traj(1, :), '-', 'Linewidth', 2.0, 'MarkerSize', 8.0, 'Color', 'k')
hold on
plot(t, exact_traj(1, :), '--', 'Linewidth', 2.0, 'MarkerSize', 8.0, 'Color', gold)
plot(t, diag_traj(1, :), '-.', 'Linewidth', 2.0, 'MarkerSize', 8.0, 'Color', dark_green)
plot(t, none_traj(1, :), ':', 'Linewidth', 2.0, 'MarkerSize', 8.0, 'Color', purple)
grid on
xlabel('Time (s)', 'Interpreter', 'latex')
ylabel('$q$ (rad)', 'Interpreter', 'latex')
legend('Ideal', 'Realized', 'Location', 'Best', 'Interpreter', 'latex')

subplot(3,1,2)
plot(t, ideal_traj(2, :), '-', 'Linewidth', 2.0, 'MarkerSize', 8.0, 'Color', 'k')
hold on
plot(t, exact_traj(2, :), '--', 'Linewidth', 2.0, 'MarkerSize', 8.0, 'Color', gold)
plot(t, diag_traj(2, :), '-.', 'Linewidth', 2.0, 'MarkerSize', 8.0, 'Color', dark_green)
plot(t, none_traj(2, :), ':', 'Linewidth', 2.0, 'MarkerSize', 8.0, 'Color', purple)
grid on
xlabel('Time (s)', 'Interpreter', 'latex')
ylabel('$\dot{q}$ (rad/$s$)', 'Interpreter', 'latex')

subplot(3,1,3)
plot(t, ideal_traj(3, :), '-', 'Linewidth', 2.0, 'MarkerSize', 8.0, 'Color', 'k')
hold on
plot(t, exact_traj(3, :), '--', 'Linewidth', 2.0, 'MarkerSize', 8.0, 'Color', gold)
plot(t, diag_traj(3, :), '-.', 'Linewidth', 2.0, 'MarkerSize', 8.0, 'Color', dark_green)
plot(t, none_traj(3, :), ':', 'Linewidth', 2.0, 'MarkerSize', 8.0, 'Color', purple)

saveas(gcf, [path_to_figures, 'Robots.png'])
