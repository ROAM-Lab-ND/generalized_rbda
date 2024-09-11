close all; clear; clc;

path_to_data = '../data/InstructionPinocchioFD_';

% csv format: # depth, loop size, c-aba instr count, pinochhio instr count
expl_data = readmatrix([path_to_data, 'Explicit.csv']);
impl_data = readmatrix([path_to_data, 'Implicit.csv']);
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
ceaba_color = 'r';
caba_name = 'Constraint-Embedding ABA, ';
pin_color = 'b';
pin_name = 'Proximal and Sparse, ';
expl_cnstr_name = 'Transmission';
impl_cnstr_name = 'Connecting Rod';
marker_size = 10;
line_width = 2.0;
font_size = 20;
figure
sp_cnt = 1;

axes = [[1 5 8e2 1e5];
         [1 8 2e3 1e5];
         [1 15 3e3 1e6];
         [1 20 1e4 1e6]];

plot_opts.MarkerSize = marker_size;
plot_opts.LineWidth = line_width;

for i = expl_depths
    % subplot(2, 2, sp_cnt)
    if (i == 5)
        axes = axes(2:end, :);
        continue;
    end
    subplot(1, 3, sp_cnt)
    hold on

    plot_opts.Color = ceaba_color;
    plot_opts.MarkerFaceColor = ceaba_color;
    plot_opts.DisplayName = [caba_name, expl_cnstr_name];
    plot(caba.expl.depths{i}(:, 1) / 2, caba.expl.depths{i}(:, 2), 'o-', plot_opts)

    plot_opts.DisplayName = [caba_name, impl_cnstr_name];
    plot((caba.impl.depths{i}(:, 1) - 1) / 2, caba.impl.depths{i}(:, 2), 's--', plot_opts)

    base_cnt = pin.expl.depths{i}(1, 2);
    plot_opts.Color = pin_color;
    plot_opts.MarkerFaceColor = pin_color;
    plot_opts.DisplayName = [pin_name, expl_cnstr_name];
    plot(pin.expl.depths{i}(:, 1) / 2, pin.expl.depths{i}(:, 2), 'o-', plot_opts)

    base_cnt = pin.impl.depths{i}(1, 2);
    plot_opts.DisplayName = [pin_name, impl_cnstr_name];
    plot((pin.impl.depths{i}(:, 1) - 1) / 2, pin.impl.depths{i}(:, 2), 's--', plot_opts)

    % Label
    xlabel('$d_l$', 'Interpreter', 'latex')
    ylabel('Instruction Count', 'Interpreter', 'latex')
    title(['$d_t$ = ', num2str(i)], 'Interpreter', 'latex')
    legend('Location', 'Northwest', 'Interpreter', 'latex')

    % Format
    grid on
    axis(axes(sp_cnt, :))
    set(gca, 'YScale', 'log')
    set(gca, 'Fontsize', font_size)
    set(gca, 'TickLabelInterpreter', 'latex')

    sp_cnt = sp_cnt + 1;
end

saveas(gcf, '../figures/ParallelChains.png')
