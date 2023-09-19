close all; clear; clc;

path_to_data = '../Benchmarking/data/TimingIOSIM_';
path_to_figures = '../Benchmarking/figures/TimingIOSIM_';

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

    hold on
    plotWithColor(data(:, 1), data(:, 4), subdued_yellow)
    plotWithColor(data(:, 1), data(:, 2), subdued_red)
    plotWithColor(data(:, 1), data(:, 3), subdued_blue)

    grid on

    xlabel('Degrees of Freedom', 'Interpreter', 'latex')
    ylabel('Computation Time (ms)', 'Interpreter', 'latex')
    legend({'EFPA', 'C-EFPA', 'Projected IOSI'}, 'Location', 'Best', 'Interpreter', 'latex')
    set(gca, 'Fontsize', 16)
    set(gca, 'TickLabelInterpreter', 'latex')
    axis([0 25 0 0.2])
end
