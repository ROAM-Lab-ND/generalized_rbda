close all; clear; clc;

path_to_data = '../data/PinocchioParallelChainFD_';

% csv format: # depth, loop size, c-aba instr count, pinochhio instr count
expl_data = readmatrix([path_to_data, 'ExplicitInstructionCount.csv']);
impl_data = readmatrix([path_to_data, 'ImplicitInstructionCount.csv']);
expl_depths = unique(expl_data(:, 1))';
impl_depths = unique(impl_data(:, 1))';

for i = expl_depths
    caba.expl.depths{i} = expl_data(expl_data(:, 1) == i, [2 3]);
    pin.expl.depths{i} = expl_data(expl_data(:, 1) == i, [2 4]);
end

for i = impl_depths
    caba.impl.depths{i} = impl_data(impl_data(:, 1) == i, [2 3]);
    pin.impl.depths{i} = impl_data(impl_data(:, 1) == i, [2 4]);
end

% TODO(@MatthewChignoli): for now we assume there are 4 depths
if length(expl_depths) ~= 4
    error('Expected 4 depths, got %d', length(expl_depths))
end
if length(impl_depths) ~= 4
    error('Expected 4 depths, got %d', length(impl_depths))
end

%% Plotting options
marker_size = 10;
line_width = 2.0;
font_size = 16;
figure
sp_cnt = 1;

for i = expl_depths
    subplot(2, 2, sp_cnt)
    hold on
    plot(caba.expl.depths{i}(:, 1), caba.expl.depths{i}(:, 2), 'o-', 'Color', 'r', 'MarkerSize', marker_size, 'MarkerFaceColor', 'r', 'Linewidth', line_width, 'DisplayName', 'Cluster-Based ABA')
    plot(pin.expl.depths{i}(:, 1), pin.expl.depths{i}(:, 2), 'o-', 'Color', 'b', 'MarkerSize', marker_size, 'MarkerFaceColor', 'b', 'Linewidth', line_width, 'DisplayName', 'Pinocchio ABA')
    plot(caba.impl.depths{i}(:, 1), caba.impl.depths{i}(:, 2), 's--', 'Color', 'r', 'MarkerSize', marker_size, 'MarkerFaceColor', 'r', 'Linewidth', line_width, 'DisplayName', 'Cluster-Based ABA')
    plot(pin.impl.depths{i}(:, 1), pin.impl.depths{i}(:, 2), 's--', 'Color', 'b', 'MarkerSize', marker_size, 'MarkerFaceColor', 'b', 'Linewidth', line_width, 'DisplayName', 'Pinocchio ABA')
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
