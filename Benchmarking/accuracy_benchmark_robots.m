%% Inverse Dynamics
close all; clear; clc;

% Define RGB colors
purple = [0.5, 0.0, 1.0];
dark_green = [0.0, 0.5, 0.0];
gold = [0.8, 0.6, 0.0];

path_to_data = '../Benchmarking/data/AccuracyID_';
path_to_figures = '../Benchmarking/figures/AccuracyID_';

data = readmatrix([path_to_data, 'Robots.csv']);

alpha = unique(data(:, 1));
rows_per_dataset = length(alpha);

datasets = cell(length(data(:, 1)) / rows_per_dataset, 1);
robot_names = {'Tello Humanoid', 'MIT Humanoid', 'Mini Cheetah'};

for i = 1:length(datasets)
    datasets{i} = data((i - 1) * rows_per_dataset + 1:i * rows_per_dataset, :);
end

figure

colors = {purple, dark_green, gold};
lgd = {};

for i = 1:length(datasets)
    robot_name = robot_names{i};
    lgd = compareInvDynAccuracy(datasets{i}, robot_name, colors{i}, lgd);
end

saveas(gcf, [path_to_figures, 'Robots.png'])

%% Forward Dynamics
clear;

% Define RGB colors
purple = [0.5, 0.0, 1.0];
dark_green = [0.0, 0.5, 0.0];
gold = [0.8, 0.6, 0.0];

path_to_data = '../Benchmarking/data/AccuracyFD_';
path_to_figures = '../Benchmarking/figures/AccuracyFD_';

data = readmatrix([path_to_data, 'Robots.csv']);

alpha = unique(data(:, 1));
rows_per_dataset = length(alpha);

datasets = cell(length(data(:, 1)) / rows_per_dataset, 1);
robot_names = {'Tello Humanoid', 'MIT Humanoid', 'Mini Cheetah'};

for i = 1:length(datasets)
    datasets{i} = data((i - 1) * rows_per_dataset + 1:i * rows_per_dataset, :);
end

figure
colors = {purple, dark_green, gold};
lgd = {};

for i = 1:length(datasets)
    robot_name = robot_names{i};
    lgd = compareFwdDynAccuracy(datasets{i}, robot_name, colors{i}, lgd);
end

saveas(gcf, [path_to_figures, 'Robots.png'])

%% Apply Test Force
clear;

% Define RGB colors
purple = [0.5, 0.0, 1.0];
dark_green = [0.0, 0.5, 0.0];
gold = [0.8, 0.6, 0.0];

path_to_data = '../Benchmarking/data/AccuracyATF_';
path_to_figures = '../Benchmarking/figures/AccuracyATF_';

data = readmatrix([path_to_data, 'Robots.csv']);

alpha = unique(data(:, 1));
rows_per_dataset = length(alpha);

datasets = cell(length(data(:, 1)) / rows_per_dataset, 1);
robot_names = {'Tello Humanoid', 'MIT Humanoid', 'Mini Cheetah'};

for i = 1:length(datasets)
    datasets{i} = data((i - 1) * rows_per_dataset + 1:i * rows_per_dataset, :);
end

figure
colors = {purple, dark_green, gold};
lgd = {};

for i = 1:length(datasets)
    robot_name = robot_names{i};
    lgd = compareApplyTestForceAccuracy(datasets{i}, robot_name, colors{i}, lgd);
end

saveas(gcf, [path_to_figures, 'Robots.png'])

%% Helper Functions
function lgd = compareInvDynAccuracy(data, robot_name, color, lgd)
    lw = 2.0;
    ms = 8.0;

    semilogy(data(:, 1), data(:, 2), 'o-', 'Linewidth', lw, 'MarkerSize', ms, 'Color', color)
    hold on

    % add to the legend
    lgd = [lgd, [robot_name, ' - Unconstrained']];

    tol = 5;

    if ~isApproxEqual(data(:, 2), data(:, 3), tol)

        if ~isApproxEqual(data(:, 3), data(:, 4), tol)
            semilogy(data(:, 1), data(:, 4), '^--', 'Linewidth', lw, 'MarkerSize', ms, 'Color', color)

        else
            semilogy(data(:, 1), data(:, 3), '^--', 'Linewidth', lw, 'MarkerSize', ms, 'Color', color)

        end

        lgd = [lgd, [robot_name, ' - Approximate']];

    end

    grid on
    axis([-inf inf 1e-2 5e3])
    xlabel('Percent of Max Acceleration', 'Interpreter', 'latex')
    ylabel('$\tau$ Error ($Nm$)', 'Interpreter', 'latex')
    legend(lgd, 'Location', 'Best', 'Interpreter', 'latex')
    set(gca, 'Fontsize', 20)
    set(gca, 'TickLabelInterpreter', 'latex')
end

function lgd = compareFwdDynAccuracy(data, robot_name, color, lgd)
    lw = 2.0;
    ms = 8.0;

    semilogy(data(:, 1), data(:, 2), 'o-', 'Linewidth', lw, 'MarkerSize', ms, 'Color', color)
    hold on
    lgd = [lgd, [robot_name, ' - Unconstrained']];

    tol = 5000;

    if ~isApproxEqual(data(:, 2), data(:, 3), tol)
        semilogy(data(:, 1), data(:, 3), '^--', 'Linewidth', lw, 'MarkerSize', ms, 'Color', color)
        lgd = [lgd, [robot_name, ' - Approximate']];
    end

    grid on
    axis([-inf inf 1e0 3e5])
    xlabel('Percent of Max Torque', 'Interpreter', 'latex')
    ylabel('$\ddot{q}$ Errror (rad/$s^2$)', 'Interpreter', 'latex')
    legend(lgd, 'Location', 'Best', 'Interpreter', 'latex')
    set(gca, 'Fontsize', 20)
    set(gca, 'TickLabelInterpreter', 'latex')
end

function lgd = compareApplyTestForceAccuracy(data, robot_name, color, lgd)
    lw = 2.0;
    ms = 8.0;

    semilogy(data(:, 1), data(:, 2), 'o-', 'Linewidth', lw, 'MarkerSize', ms, 'Color', color)
    hold on
    lgd = [lgd, [robot_name, ' - Unconstrained']];

    tol = 50;

    if ~isApproxEqual(data(:, 2), data(:, 3), tol)
        semilogy(data(:, 1), data(:, 3), '^--', 'Linewidth', lw, 'MarkerSize', ms, 'Color', color)
        lgd = [lgd, [robot_name, ' - Approximate']];
    end

    grid on
    xlabel('Percent of Max Force', 'Interpreter', 'latex')
    ylabel('$\Delta \dot{q}$ Error (rad/$s$)', 'Interpreter', 'latex')
    legend(lgd, 'Location', 'Best', 'Interpreter', 'latex')
    set(gca, 'Fontsize', 20)
    set(gca, 'TickLabelInterpreter', 'latex')
    axis([-inf inf 1e-2 2e5])
end

function result = isApproxEqual(a, b, tolerance)
    % Check if the arrays have the same size
    if ~isequal(size(a), size(b))
        result = false;
        return;
    end

    % Check if all elements are within the specified tolerance
    result = all(abs(a - b) <= tolerance);
end
