%% Inverse Dynamics
close all; clear; clc;

path_to_data = '../Benchmarking/data/AccuracyID_';
path_to_figures = '../Benchmarking/figures/AccuracyID_';

data = readmatrix([path_to_data, 'Robots.csv']);

alpha = unique(data(:,1));
rows_per_dataset = length(alpha);

datasets = cell(length(data(:,1)) / rows_per_dataset, 1);
robot_names = {'Tello','Humanoid','Mini Cheetah'};

for i = 1:length(datasets)
    datasets{i} = data((i-1)*rows_per_dataset + 1:i*rows_per_dataset, :);
end

figure
colors = {'r', 'g', 'b'};
lgd = {};
for i = 1:length(datasets)
    robot_name = robot_names{i};
    lgd = compareInvDynAccuracy(datasets{i}, robot_name, colors{i}, lgd);
end
saveas(gcf, [path_to_figures, 'Robots.png'])

%% Forward Dynamics
clear;

path_to_data = '../Benchmarking/data/AccuracyFD_';
path_to_figures = '../Benchmarking/figures/AccuracyFD_';

data = readmatrix([path_to_data, 'Robots.csv']);

alpha = unique(data(:,1));
rows_per_dataset = length(alpha);

datasets = cell(length(data(:,1)) / rows_per_dataset, 1);
robot_names = {'Tello','Humanoid','Mini Cheetah'};

for i = 1:length(datasets)
    datasets{i} = data((i-1)*rows_per_dataset + 1:i*rows_per_dataset, :);
end

figure
colors = {'r', 'g', 'b'};
lgd = {};
for i = 1:length(datasets)
    robot_name = robot_names{i};
    lgd = compareFwdDynAccuracy(datasets{i}, robot_name, colors{i}, lgd);
end
saveas(gcf, [path_to_figures, 'Robots.png'])

%% Helper Functions
function lgd = compareInvDynAccuracy(data, robot_name, color, lgd)
    lw = 2.0;
    ms = 10.0;

    semilogy(data(:,1), data(:,2), [color,'o-'], 'Linewidth', lw, 'MarkerSize', ms)
    hold on

    % add to the legend
    lgd = [lgd, [robot_name, ' - Unconstrained']];

    tol = 5;
    if ~isApproxEqual(data(:, 2), data(:, 3), tol)
        semilogy(data(:, 1), data(:, 3), [color, '^--'], 'Linewidth', lw, 'MarkerSize', ms)
        lgd = [lgd, [robot_name, ' - Diagonal Approx']];

        if ~isApproxEqual(data(:, 3), data(:, 4), tol)
            semilogy(data(:, 1), data(:, 4), [color, '*:'], 'Linewidth', lw, 'MarkerSize', ms)
            lgd = [lgd, [robot_name, ' - Block Diagonal Approx']];

        end

    end

    grid on
    xlabel('Percent of Max Acceleration', 'Interpreter', 'latex')
    ylabel('Inverse Dynamics Error ($N\cdot m$)', 'Interpreter', 'latex')
    legend(lgd, 'Location', 'Best', 'Interpreter', 'latex')
    set(gca, 'Fontsize', 20)
    set(gca, 'TickLabelInterpreter', 'latex')
end

function lgd = compareFwdDynAccuracy(data, robot_name, color, lgd)
    lw = 2.0;
    ms = 10.0;

    semilogy(data(:,1), data(:,2), [color,'o-'], 'Linewidth', lw, 'MarkerSize', ms)
    hold on
    lgd = [lgd, [robot_name, ' - Unconstrained']];

    tol = 5;
    if ~isApproxEqual(data(:, 2), data(:, 3), tol)
        semilogy(data(:, 1), data(:, 3), [color, '^--'], 'Linewidth', lw, 'MarkerSize', ms)
        lgd = [lgd, [robot_name, ' - Rotor Inertia Approx']];
    end

    grid on
    xlabel('Percent of Max Torque', 'Interpreter', 'latex')
    ylabel('Forward Dynamics Error ($rad/s^2$)', 'Interpreter', 'latex')
    legend(lgd, 'Location', 'Best', 'Interpreter', 'latex')
    set(gca, 'Fontsize', 20)
    set(gca, 'TickLabelInterpreter', 'latex')
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
