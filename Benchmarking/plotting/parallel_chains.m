close all; clear; clc;

path_to_data = '../data/PinocchioParallelChainFD_';

% csv format: # depth, loop size, c-aba instr count, pinochhio instr count
exp_data = readmatrix([path_to_data, 'ExplicitInstructionCount.csv']);
depths = unique(exp_data(:, 1))';

for i = depths
    caba.depths{i} = exp_data(exp_data(:, 1) == i, [2 3]);
    pin.depths{i} = exp_data(exp_data(:, 1) == i, [2 4]);
end

% TODO(@MatthewChignoli): for now we assume there are 4 depths
if length(depths) ~= 4
    error('Expected 4 depths, got %d', length(depths))
end

%% Plotting options
marker_size = 10;
line_width = 2.0;
font_size = 16;
figure
sp_cnt = 1;

for i = depths
    subplot(2, 2, sp_cnt)
    hold on
    plot(caba.depths{i}(:, 1), caba.depths{i}(:, 2), 'o-', 'Color', 'r', 'MarkerSize', marker_size, 'MarkerFaceColor', 'r', 'Linewidth', line_width, 'DisplayName', 'Cluster-Based ABA')
    plot(pin.depths{i}(:, 1), pin.depths{i}(:, 2), 'o-', 'Color', 'b', 'MarkerSize', marker_size, 'MarkerFaceColor', 'b', 'Linewidth', line_width, 'DisplayName', 'Pinocchio ABA')
    sp_cnt = sp_cnt + 1;

    % Label
    xlabel('Loop Size', 'Interpreter', 'latex')
    ylabel('Instruction Count', 'Interpreter', 'latex')
    title(['Spanning Tree Depth: ', num2str(i)], 'Interpreter', 'latex')
    legend('Location', 'Best', 'Interpreter', 'latex')

    % Format
    grid on
    set(gca, 'YScale', 'log')
    set(gca, 'Fontsize', font_size)
    set(gca, 'TickLabelInterpreter', 'latex')
end
