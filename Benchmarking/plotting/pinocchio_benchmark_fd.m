close all; clear; clc;

path_to_data = '../data/InstructionPinocchioFD_';
urdf_list = {'revolute_chain','revolute_pair_chain'};

% csv format: # branches, cluster tree depth, c-aba instr count, pinochhio instr count
for j = 1:length(urdf_list)
    urdf = urdf_list{j};

    %% Read data
    data = readmatrix([path_to_data, urdf, '.csv']);
    branches = unique(data(:, 1))';

    for i = branches
        caba.branches{i} = data(data(:, 1) == i, [2 3]);
        pin.branches{i} = data(data(:, 1) == i, [2 4]);
    end

    %% Plotting options
    symbols = {'o--', 's--', 'd--', '^--', '*--', 'p--', 'h--', 'x--', '+--'};
    marker_size = 10;
    line_width = 2.0;
    font_size = 16;

    %% Dummy entries for the legend
    figure
    hold on
    legend_entries = {};

    for i = branches
        plot(NaN, NaN, symbols{i}, 'Color', 'k', 'MarkerSize', marker_size, 'MarkerFaceColor', 'k')
        legend_entries{end + 1} = [num2str(i), ' branches'];
    end

    plot(NaN, NaN, '-', 'Color', 'r', 'LineWidth', line_width)
    plot(NaN, NaN, '-', 'Color', 'b', 'LineWidth', line_width)
    legend_entries{end + 1} = 'Cluster-Based ABA';
    legend_entries{end + 1} = 'Pinocchio ABA';

    %% Plot Data
    for i = branches
        plot(caba.branches{i}(:, 1), caba.branches{i}(:, 2), symbols{i}, 'Color', 'r', 'MarkerSize', marker_size, 'MarkerFaceColor', 'r')
        plot(pin.branches{i}(:, 1), pin.branches{i}(:, 2), symbols{i}, 'Color', 'b', 'MarkerSize', marker_size, 'MarkerFaceColor', 'b')
    end

    %% Format
    grid on

    xlabel('Cluster Tree Depth', 'Interpreter', 'latex')
    ylabel('Instruction Count', 'Interpreter', 'latex')
    title(urdf, 'Interpreter', 'latex')

    legend(legend_entries, 'Location', 'Best', 'Interpreter', 'latex')

    set(gca, 'Fontsize', font_size)
    set(gca, 'TickLabelInterpreter', 'latex')

end
