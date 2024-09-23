close all; clear; clc;

path_to_data = '../Benchmarking/data/TimingFD_';
path_to_figures = '../Benchmarking/figures/TimingFD_';

%% Compare Forward Dynamics Times as number of DoFs increases
% Revolute Chain
figure
revolute_chain_with_rotors = readmatrix([path_to_data, 'RevoluteChain.csv']);
compareTimes(revolute_chain_with_rotors)
saveas(gcf, [path_to_figures, 'RevoluteChains.png'])

% Revolute Pair Chain
figure
revolute_pair_chain_with_rotors = readmatrix([path_to_data, 'RevolutePairChain.csv']);
compareTimes(revolute_pair_chain_with_rotors)
saveas(gcf, [path_to_figures, 'RevolutePairChains.png'])

%% Helper Functions
function plotWithColor(x, y, rgb)
    plot(x, y, 'o-', 'Color', rgb, 'Linewidth', 2.0, 'MarkerSize', 10., 'MarkerFaceColor', rgb)
end

function compareTimes(data)
    load('../Benchmarking/plotting/custom_colors.mat')

    plot_both_lg = false;

    plotWithColor(data(:, 1), data(:, 2), subdued_red)
    hold on
    plotWithColor(data(:, 1), data(:, 3), subdued_green)
    if plot_both_lg
        plotWithColor(data(:, 1), data(:, 4), subdued_green)
    end
    plotWithColor(data(:, 1), data(:, 5), subdued_blue)
    plotWithColor(data(:, 1), data(:, 6), subdued_yellow)
    grid on

    xlabel('Degrees of Freedom', 'Interpreter', 'latex')
    ylabel('Computation Time (ms)', 'Interpreter', 'latex')

    if plot_both_lg
        legend({'Cluster-Based', 'Lagrange Multipliers (Custom)', 'Lagrange Multipliers (Eigen)', 'Projection', 'Original ABA'}, 'Location', 'Best', 'Interpreter', 'latex')
    else
        legend({'Cluster-Based', 'Lagrange Multipliers', 'Projection', 'Approximate ABA'}, 'Location', 'Best', 'Interpreter', 'latex')
    end

    set(gca, 'Fontsize', 16)
    set(gca, 'TickLabelInterpreter', 'latex')
    axis([0 25 0 0.25])
end
