close all; clear; clc;

path_to_data = '../Benchmarking/data/CL_';
path_to_figures = '../Benchmarking/figures/CL_';

% Revolute Chain
revolute_chain_with_rotors = readmatrix([path_to_data, 'RevoluteChain.csv']);
plotTimingStats(revolute_chain_with_rotors, 'Revolute w/ Rotor Chain')
saveas(gcf, [path_to_figures, 'RevoluteChain.png'])
plotPassTiming(revolute_chain_with_rotors, 'Revolute w/ Rotor Chain')
saveas(gcf, [path_to_figures, 'Passes_RevoluteChain.png'])

% Revolute Pair Chain
revolute_pair_chain_with_rotors = readmatrix([path_to_data, 'RevolutePairChain.csv']);
plotTimingStats(revolute_pair_chain_with_rotors, 'Revolute Pair w/ Rotors Chain')
saveas(gcf, [path_to_figures, 'RevolutePairChain.png'])
plotPassTiming(revolute_pair_chain_with_rotors, 'Revolute Pair w/ Rotors Chain')
saveas(gcf, [path_to_figures, 'Passes_RevolutePairChain.png'])

% Robots
robots = readmatrix([path_to_data, 'Robots.csv']);
plotEfpaTimingStats(robots(1, 2:end), 'Humanoid')
saveas(gcf, [path_to_figures, 'EFPA_Humanoid.png'])

%% Helper Function
function plotTimingStats(data, robot_name)

    figure
    bar(data(:, 1), data(:, 2:end), 'stacked')
    legend('Forward Kinematics', 'Update Articulated Bodies', 'Foward Pass 1', 'External Forces', 'Backward Pass', 'Forward Pass 2', 'Interpreter', 'latex', 'Location', 'northwest')

    xlabel('Number of Joints', 'Interpreter', 'latex')
    ylabel('Time (ms)', 'Interpreter', 'latex')
    title(['Forward Dynamics Timing for ', robot_name], 'Interpreter', 'latex')
    grid on
    set(gca, 'FontSize', 14)
    set(gca, 'TickLabelInterpreter', 'latex')

end

function plotPassTiming(data_in, robot_name)

    data = zeros(size(data_in, 1), 4);
    data(:, 1) = data_in(:, 1);
    data(:, 2) = data_in(:, 2) + data_in(:, 4);
    data(:, 3) = data_in(:, 3) + data_in(:, 6);
    data(:, 4) = data_in(:, 7);

    figure
    bar(data(:, 1), data(:, 2:end), 'stacked')
    legend('Foward Pass 1', 'Backward Pass', 'Forward Pass 2', 'Interpreter', 'latex', 'Location', 'northwest')

    xlabel('Number of Joints', 'Interpreter', 'latex')
    ylabel('Time (ms)', 'Interpreter', 'latex')
    title(['Forward Dynamics Timing for ', robot_name], 'Interpreter', 'latex')
    grid on
    set(gca, 'FontSize', 14)
    set(gca, 'TickLabelInterpreter', 'latex')

end

function plotEfpaTimingStats(data, robot_name)

    figure
    bar(data)
    xticklabels({'Forward Kinematics', 'Update Articulated Bodies', 'Update Force Propagators', 'Reset EE Force Propagators', 'Backward Pass', 'Forward Pass'})
    ylabel('Time (ms)', 'Interpreter', 'latex')
    title(['EFPA Timing for ', robot_name], 'Interpreter', 'latex')
    grid on
    set(gca, 'FontSize', 14)
    set(gca, 'TickLabelInterpreter', 'latex')

end
