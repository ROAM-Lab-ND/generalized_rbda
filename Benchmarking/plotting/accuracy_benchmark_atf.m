close all; clear; clc;

path_to_data = '../Benchmarking/data/Accuracy';
path_to_figures = '../Benchmarking/figures/Accuracy';

% Revolute Chain
revolute_chain_with_rotors = readmatrix([path_to_data, 'ATF_RevoluteChain.csv']);
compareApplyTestForceAccuracy(revolute_chain_with_rotors, 'Revolute w/ Rotor Chain')
saveas(gcf, [path_to_figures, 'ATF_RevoluteChain.png'])

% Revolute Pair Chain
revolute_pair_chain_with_rotors = readmatrix([path_to_data, 'ATF_RevolutePairChain.csv']);
compareApplyTestForceAccuracy(revolute_pair_chain_with_rotors, 'Revolute Pair w/ Rotors Chain')
saveas(gcf, [path_to_figures, 'ATF_RevolutePairChain.png'])

%% Helper Functions
function compareApplyTestForceAccuracy(data, robot_name)
    figure
    subplot(1, 2, 1)
    plot(data(:, 1), data(:, 2), 'ro-', 'Linewidth', 2.0, 'MarkerSize', 10.)
    hold on
    plot(data(:, 1), data(:, 3), 'go-', 'Linewidth', 2.0, 'MarkerSize', 10.)

    grid on

    xlabel('Degrees of Freedom', 'Interpreter', 'latex')
    ylabel('$\Lambda_{ii}^{-1}$ error (units?)', 'Interpreter', 'latex')
    title(['Apply Test Force - ', robot_name], 'Interpreter', 'latex')
    legend({'Block Diagonal Reflected Inertia', 'Diagonal Reflected Inertia'}, 'Location', 'Northwest', 'Interpreter', 'latex')
    set(gca, 'Fontsize', 12)
    set(gca, 'TickLabelInterpreter', 'latex')

    subplot(1, 2, 2)
    plot(data(:, 1), data(:, 4), 'ro-', 'Linewidth', 2.0, 'MarkerSize', 10.)
    hold on
    plot(data(:, 1), data(:, 5), 'go-', 'Linewidth', 2.0, 'MarkerSize', 10.)

    grid on

    xlabel('Degrees of Freedom', 'Interpreter', 'latex')
    ylabel('$\ddot{q}$ error (rad/s$^2$)', 'Interpreter', 'latex')
    title(['Apply Test Force - ', robot_name], 'Interpreter', 'latex')
    legend({'Block Diagonal Reflected Inertia', 'Diagonal Reflected Inertia'}, 'Location', 'Northwest', 'Interpreter', 'latex')
    set(gca, 'Fontsize', 12)
    set(gca, 'TickLabelInterpreter', 'latex')

end
