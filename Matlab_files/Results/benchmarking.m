close all; clear; clc;

%% Time
% Revolute Chain
revolute_chain_with_rotors = readmatrix('RevoluteChain.csv');
compareTimes(revolute_chain_with_rotors, 'Revolute w/ Rotor Chain')

% Revolute Pair Chain
revolute_pair_chain_with_rotors = readmatrix('RevolutePairChain.csv');
compareTimes(revolute_pair_chain_with_rotors, 'Revolute Pair w/ Rotors Chain')

% Revolute Chain with Multiple Rotors
revolute_chain_with_multiple_rotors = readmatrix('RevoluteMultiRotorChain.csv');
compareTimes(revolute_chain_with_multiple_rotors, 'Revolute w/ Mutli Rotors Chain')

revolute_chain_with_and_wo_rotors = readmatrix('RevoluteWithAndWithoutRotorChain.csv');
compareTimes(revolute_chain_with_and_wo_rotors, 'Revolute w/ \& w/o Rotors Chain')

%% Accuracy
% Revolute Chain
revolute_chain_with_rotors = readmatrix('RevoluteChainAccuracy.csv');
% compareAccuracy(revolute_chain_with_rotors, 'Revolute w/ Rotor Chain')

% Revolute Pair Chain
revolute_pair_chain_with_rotors = readmatrix('RevolutePairChainAccuracy.csv');
% compareAccuracy(revolute_pair_chain_with_rotors, 'Revolute Pair w/ Rotors Chain')

%% Helper Functions
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

function compareAccuracy(data, robot_name)
    figure
    plot(data(:, 1), data(:, 2), 'ro-', 'Linewidth', 2.0, 'MarkerSize', 10.)
    hold on
    plot(data(:, 1), data(:, 3), 'go-', 'Linewidth', 2.0, 'MarkerSize', 10.)

    grid on

    xlabel('Degrees of Freedom', 'Interpreter', 'latex')
    ylabel('Forward Dynamics Error (rad/s^2)', 'Interpreter', 'latex')
    title(['Forward Dynamics - ', robot_name], 'Interpreter', 'latex')
    legend({'Block Diagonal Reflected Inertia', 'Diagonal Reflected Inertia'}, 'Location', 'Northwest', 'Interpreter', 'latex')
    set(gca, 'Fontsize', 20)
    set(gca, 'TickLabelInterpreter', 'latex')
end
