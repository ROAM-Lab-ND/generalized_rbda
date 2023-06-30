close all; clear; clc;

path_to_data = '../Benchmarking/data/Accuracy_';
path_to_figures = '../Benchmarking/figures/Accuracy_';

% Revolute Chain
revolute_chain_with_rotors = readmatrix([path_to_data,'RevoluteChain.csv']);
compareAccuracy(revolute_chain_with_rotors, 'Revolute w/ Rotor Chain')
saveas(gcf, [path_to_figures,'RevoluteChain.png'])

% Revolute Pair Chain
revolute_pair_chain_with_rotors = readmatrix([path_to_data,'RevolutePairChain.csv']);
compareAccuracy(revolute_pair_chain_with_rotors, 'Revolute Pair w/ Rotors Chain')
saveas(gcf, [path_to_figures,'RevolutePairChain.png'])

%% Helper Functions
function compareAccuracy(data, robot_name)
    figure
    plot(data(:, 1), data(:, 2), 'ro-', 'Linewidth', 2.0, 'MarkerSize', 10.)
    hold on
    plot(data(:, 1), data(:, 3), 'go-', 'Linewidth', 2.0, 'MarkerSize', 10.)

    grid on

    xlabel('Degrees of Freedom', 'Interpreter', 'latex')
    ylabel('Forward Dynamics Error (rad/s$^2$)', 'Interpreter', 'latex')
    title(['Forward Dynamics - ', robot_name], 'Interpreter', 'latex')
    legend({'Block Diagonal Reflected Inertia', 'Diagonal Reflected Inertia'}, 'Location', 'Northwest', 'Interpreter', 'latex')
    set(gca, 'Fontsize', 20)
    set(gca, 'TickLabelInterpreter', 'latex')
end
