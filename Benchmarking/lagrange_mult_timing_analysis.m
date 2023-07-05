close all; clear; clc;

path_to_data = '../Benchmarking/data/LgMlt_';
path_to_figures = '../Benchmarking/figures/LgMlt_';

% TODO(@MatthewChignoli): Add timing data for the projection method as a comparison

% Revolute Chain
revolute_chain_with_rotors = readmatrix([path_to_data, 'RevoluteChain.csv']);
plotTimingStats(revolute_chain_with_rotors, 'Revolute w/ Rotor Chain')
saveas(gcf, [path_to_figures, 'RevoluteChain.png'])

% Revolute Pair Chain
revolute_pair_chain_with_rotors = readmatrix([path_to_data, 'RevolutePairChain.csv']);
plotTimingStats(revolute_pair_chain_with_rotors, 'Revolute Pair w/ Rotors Chain')
saveas(gcf, [path_to_figures, 'RevolutePairChain.png'])

%% Helper Function
function plotTimingStats(data, robot_name)

    figure
    bar(data(:, 1), data(:, 2:end), 'stacked')
    legend('LTL', '$\tau''$', '$Y,z$', '$A,b$', '$\lambda$', '$\ddot{q}$', 'Interpreter', 'latex','Location','northwest')

    xlabel('Number of Joints','Interpreter','latex')
    ylabel('Time (ms)','Interpreter','latex')
    title(['Forward Dynamics Timing for ', robot_name],'Interpreter','latex')
    grid on
    set(gca, 'FontSize', 14)
    set(gca,'TickLabelInterpreter','latex')

end
