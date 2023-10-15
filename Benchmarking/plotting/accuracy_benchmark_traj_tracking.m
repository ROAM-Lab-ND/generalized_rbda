%% Trajectory Tracking
close all; clear; clc;

% Define RGB colors
purple = [0.5, 0.0, 1.0];
dark_green = [0.0, 0.5, 0.0];
gold = [0.8, 0.6, 0.0];
teal = [0.0, 0.5, 0.5];
orange = [0.9, 0.5, 0.0];

% Define paths for data and figures
path_to_data = '../Benchmarking/data/AccuracyTT_';
path_to_figures = '../Benchmarking/figures/AccuracyTT_';

% Load data
data = readmatrix([path_to_data, 'Robots.csv']);

% Separate data by wave frequency
omega = unique(data(:, 1));
num_freq_sets = length(omega);
freq_set = cell(num_freq_sets, 1);
for i = 1:length(omega)
    freq_set{i} = data(data(:, 1) == omega(i), :);
end

% Extract the length of the trajectory
t = unique(data(:, 2));
traj_length = length(t);

% Collect the tracking error for each frequency
exact_error = zeros(num_freq_sets, traj_length);
diag_error = zeros(num_freq_sets, traj_length);
none_error = zeros(num_freq_sets, traj_length);

for i = 1:num_freq_sets
    data_subset = data(data(:, 1) == omega(i), :);

    % Separate the data by model type
    ideal_traj = data_subset(1:traj_length, 3:5)';
    exact_traj = data_subset(traj_length + 1:2 * traj_length, 3:5)';
    diag_traj = data_subset(2 * traj_length + 1:3 * traj_length, 3:5)';
    none_traj = data_subset(3 * traj_length + 1:4 * traj_length, 3:5)';

    % Compute the norm of the error at every timestep
    for j = 1:traj_length
        exact_error(i, j) = norm(exact_traj(:, j) - ideal_traj(:, j));
        diag_error(i, j) = norm(diag_traj(:, j) - ideal_traj(:, j));
        none_error(i, j) = norm(none_traj(:, j) - ideal_traj(:, j));
    end

end

exact_error_mean = mean(exact_error, 1);
diag_error_mean = mean(diag_error, 1);
none_error_mean = mean(none_error, 1);

exact_error_std = std(exact_error, 1);
diag_error_std = std(diag_error, 1);
none_error_std = std(none_error, 1);

% Plot average tracking error over the sampled frequencies
figure
plotMean(t, exact_error_mean, gold)
hold on
plotMean(t, diag_error_mean, dark_green)
plotMean(t, none_error_mean, purple)

plotStd(t, none_error_mean, none_error_std, purple)
plotStd(t, diag_error_mean, diag_error_std, dark_green)
plotStd(t, exact_error_mean, exact_error_std, gold)

xlabel('Time (s)', 'Interpreter', 'latex')
ylabel('End-Effector Tracking Error (m)', 'Interpreter', 'latex')
legend({'C-RNEA', 'Approximate RNEA', 'Unconstrained RNEA'}, 'Location', 'northwest','Interpreter', 'latex')
grid on
set(gca, 'FontSize', 14)

saveas(gcf, [path_to_figures, 'Robots.png'])

%% Helper Functions
function plotMean(t, mean, color)
    plot(t, mean, '-', 'Linewidth', 2.0, 'MarkerSize', 8.0, 'Color', color)
end

function plotStd(t, mean, std, color)
    fill([t', fliplr(t')], [mean - std, fliplr(mean + std)], color, 'EdgeColor', 'none','FaceAlpha',0.3);
end
