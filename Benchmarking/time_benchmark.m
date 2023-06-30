close all; clear; clc;

path_to_data = '../Benchmarking/data/Timing_';
path_to_figures = '../Benchmarking/figures/Timing_';

% Revolute Chain
revolute_chain_with_rotors = readmatrix([path_to_data,'RevoluteChain.csv']);
compareTimes(revolute_chain_with_rotors, 'Revolute w/ Rotor Chain')
saveas(gcf, [path_to_figures,'RevoluteChain.png'])

% Revolute Pair Chain
revolute_pair_chain_with_rotors = readmatrix([path_to_data,'RevolutePairChain.csv']);
compareTimes(revolute_pair_chain_with_rotors, 'Revolute Pair w/ Rotors Chain')
saveas(gcf, [path_to_figures,'RevolutePairChain.png'])

% Revolute Chain with Multiple Rotors
revolute_chain_with_multiple_rotors = readmatrix([path_to_data,'RevoluteMultiRotorChain.csv']);
compareTimes(revolute_chain_with_multiple_rotors, 'Revolute w/ Mutli Rotors Chain')
saveas(gcf, [path_to_figures,'RevoluteMultiRotorChain.png'])

% Revolute Chain where some links have rotors and some don't
revolute_chain_with_and_wo_rotors = readmatrix([path_to_data,'RevoluteWithAndWithoutRotorChain.csv']);
compareTimes(revolute_chain_with_and_wo_rotors, 'Revolute w/ \& w/o Rotors Chain')
saveas(gcf, [path_to_figures,'RevoluteWithAndWithoutRotorChain.png'])

%% Helper Function
function compareTimes(data, robot_name)
    figure
    plot(data(:, 1), data(:, 2), 'ro-', 'Linewidth', 2.0, 'MarkerSize', 10.)
    hold on
    plot(data(:, 1), data(:, 3), 'go-', 'Linewidth', 2.0, 'MarkerSize', 10.)
    plot(data(:, 1), data(:, 4), 'bo-', 'Linewidth', 2.0, 'MarkerSize', 10.)
    plot(data(:, 1), data(:, 5), 'ko-', 'Linewidth', 2.0, 'MarkerSize', 10.)

    grid on

    xlabel('Degrees of Freedom', 'Interpreter', 'latex')
    ylabel('Computation Time (ms)', 'Interpreter', 'latex')
    title(['Forward Dynamics - ', robot_name], 'Interpreter', 'latex')
    legend({'Cluster-Based', 'Lagrange Multipliers', 'Projection', 'Reflected Inertia Approximation'}, 'Location', 'Northwest', 'Interpreter', 'latex')
    set(gca, 'Fontsize', 20)
    set(gca, 'TickLabelInterpreter', 'latex')

end
