close all; clear; clc;

% TODO(@MatthewChignoli): test all matlab plotting scripts

path_to_data = '../Benchmarking/data/Timing_';
path_to_figures = '../Benchmarking/figures/Timing_';

%% Compare Forward Dynamics Times as number of DoFs increases
figure

% Revolute Chain
figure
revolute_chain_with_rotors = readmatrix([path_to_data, 'RevoluteChain.csv']);
compareTimes(revolute_chain_with_rotors)

% Revolute Pair Chain
figure
revolute_pair_chain_with_rotors = readmatrix([path_to_data, 'RevolutePairChain.csv']);
compareTimes(revolute_pair_chain_with_rotors)

saveas(gcf, [path_to_figures, 'RevoluteChains.png'])

%% Helper Function
% Function for getting the number of rows and columns of the subplot
function [rows, cols] = getSubplotRowsAndCols(num_plots)
    rows = floor(sqrt(num_plots));
    cols = ceil(num_plots / rows);
end

function compareTimes(data)
    
    plot_both_lg = false;

    plot(data(:, 1), data(:, 6), 'yo-', 'Linewidth', 2.0, 'MarkerSize', 10.,'MarkerFaceColor','y')
    hold on
    plot(data(:, 1), data(:, 2), 'ro-', 'Linewidth', 2.0, 'MarkerSize', 10.,'MarkerFaceColor','r')
    plot(data(:, 1), data(:, 3), 'go-', 'Linewidth', 2.0, 'MarkerSize', 10.,'MarkerFaceColor','g')
    if plot_both_lg
        plot(data(:, 1), data(:, 4), 'go--', 'Linewidth', 2.0, 'MarkerSize', 10.)
    end
    plot(data(:, 1), data(:, 5), 'bo-', 'Linewidth', 2.0, 'MarkerSize', 10.,'MarkerFaceColor','b')
    
    grid on

    xlabel('Degrees of Freedom', 'Interpreter', 'latex')
    ylabel('Computation Time (ms)', 'Interpreter', 'latex')
    if plot_both_lg
        legend({'Cluster-Based', 'Lagrange Multipliers (Custom)', 'Lagrange Multipliers (Eigen)', 'Projection', 'Original ABA'}, 'Location', 'Best', 'Interpreter', 'latex')
    else
        legend({'Cluster-Based', 'Lagrange Multipliers', 'Projection', 'Original ABA'}, 'Location', 'Best', 'Interpreter', 'latex')
    end
    set(gca, 'Fontsize', 16)
    set(gca, 'TickLabelInterpreter', 'latex')
    axis([0 25 0 0.3])
end
