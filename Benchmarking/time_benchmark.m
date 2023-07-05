close all; clear; clc;

path_to_data = '../Benchmarking/data/Timing_';
path_to_figures = '../Benchmarking/figures/Timing_';

%% Compare Forward Dynamics Times as number of
% Revolute Chain
revolute_chain_with_rotors = readmatrix([path_to_data, 'RevoluteChain.csv']);
compareTimes(revolute_chain_with_rotors, 'Revolute w/ Rotor Chain')
saveas(gcf, [path_to_figures, 'RevoluteChain.png'])

% Revolute Pair Chain
revolute_pair_chain_with_rotors = readmatrix([path_to_data, 'RevolutePairChain.csv']);
compareTimes(revolute_pair_chain_with_rotors, 'Revolute Pair w/ Rotors Chain')
saveas(gcf, [path_to_figures, 'RevolutePairChain.png'])

%% Revolute Chain with Multiple Rotors
revolute_chain_with_multiple_rotors = readmatrix([path_to_data, 'RevoluteMultiRotorChain.csv']);
compareMultipleRotorTimes(revolute_chain_with_multiple_rotors)
saveas(gcf, [path_to_figures, 'RevoluteMultiRotorChain.png'])

%% Revolute Chain where some links have rotors and some don't
revolute_chain_with_and_wo_rotors = readmatrix([path_to_data, 'RevoluteWithAndWithoutRotorChain.csv']);
compareWithAndWithoutRotorTimes(revolute_chain_with_and_wo_rotors)
saveas(gcf, [path_to_figures, 'RevoluteWithAndWithoutRotorChain.png'])

%% Helper Function
% Function for getting the number of rows and columns of the subplot
function [rows, cols] = getSubplotRowsAndCols(num_plots)
    rows = floor(sqrt(num_plots));
    cols = ceil(num_plots / rows);
end

function compareTimes(data, robot_name)
    figure
    plot(data(:, 1), data(:, 2), 'ro-', 'Linewidth', 2.0, 'MarkerSize', 10.)
    hold on
    plot(data(:, 1), data(:, 3), 'go-', 'Linewidth', 2.0, 'MarkerSize', 10.)
    plot(data(:, 1), data(:, 4), 'go--', 'Linewidth', 2.0, 'MarkerSize', 10.)
    plot(data(:, 1), data(:, 5), 'bo-', 'Linewidth', 2.0, 'MarkerSize', 10.)
    plot(data(:, 1), data(:, 6), 'ko-', 'Linewidth', 2.0, 'MarkerSize', 10.)

    grid on

    xlabel('Degrees of Freedom', 'Interpreter', 'latex')
    ylabel('Computation Time (ms)', 'Interpreter', 'latex')
    title(['Forward Dynamics - ', robot_name], 'Interpreter', 'latex')
    legend({'Cluster-Based', 'Lagrange Multipliers (Custom)', 'Lagrange Multipliers (Eigen)', 'Projection', 'Reflected Inertia Approximation'}, 'Location', 'Northwest', 'Interpreter', 'latex')
    set(gca, 'Fontsize', 20)
    set(gca, 'TickLabelInterpreter', 'latex')
end

function compareMultipleRotorTimes(data)
    dofs = unique(data(:, 1));
    [rows, cols] = getSubplotRowsAndCols(length(dofs));

    figure

    for i = 1:length(dofs)
        subplot(rows, cols, i)
        idx = find(data(:, 1) == dofs(i));
        plot(data(idx, 2), data(idx, 3), 'ro-', 'Linewidth', 2.0, 'MarkerSize', 10.)
        hold on
        plot(data(idx, 2), data(idx, 4), 'go-', 'Linewidth', 2.0, 'MarkerSize', 10.)
        plot(data(idx, 2), data(idx, 5), 'go--', 'Linewidth', 2.0, 'MarkerSize', 10.)
        plot(data(idx, 2), data(idx, 6), 'bo-', 'Linewidth', 2.0, 'MarkerSize', 10.)
        plot(data(idx, 2), data(idx, 7), 'ko-', 'Linewidth', 2.0, 'MarkerSize', 10.)

        grid on
        xlabel('Rotors Per Link', 'Interpreter', 'latex')
        ylabel('Computation Time (ms)', 'Interpreter', 'latex')
        title(['Forward Dynamics - Revolute Multi Rotor Chain - ', num2str(dofs(i)), ' DoFs'], 'Interpreter', 'latex')
        legend({'Cluster-Based', 'Lagrange Multipliers (Custom)', 'Lagrange Multipliers (Eigen)', 'Projection', 'Reflected Inertia Approximation'}, 'Location', 'Northwest', 'Interpreter', 'latex')
        set(gca, 'Fontsize', 10)
        set(gca, 'TickLabelInterpreter', 'latex')
    end

end

function compareWithAndWithoutRotorTimes(data)
    figure
    plot(data(:, 1), data(:, 3), 'ro-', 'Linewidth', 2.0, 'MarkerSize', 10.)
    hold on
    plot(data(:, 1), data(:, 4), 'go-', 'Linewidth', 2.0, 'MarkerSize', 10.)
    plot(data(:, 1), data(:, 5), 'go--', 'Linewidth', 2.0, 'MarkerSize', 10.)
    plot(data(:, 1), data(:, 6), 'bo-', 'Linewidth', 2.0, 'MarkerSize', 10.)
    plot(data(:, 1), data(:, 7), 'ko-', 'Linewidth', 2.0, 'MarkerSize', 10.)

    grid on

    xlabel('Number of Links w/ Rotor', 'Interpreter', 'latex')
    ylabel('Computation Time (ms)', 'Interpreter', 'latex')
    title(['Forward Dynamics - ', num2str(sum(data(1, 1:2))), ' DoFs'], 'Interpreter', 'latex')
    legend({'Cluster-Based', 'Lagrange Multipliers (Custom)', 'Lagrange Multipliers (Eigen)', 'Projection', 'Reflected Inertia Approximation'}, 'Location', 'Northwest', 'Interpreter', 'latex')
    set(gca, 'Fontsize', 20)
    set(gca, 'TickLabelInterpreter', 'latex')
end
