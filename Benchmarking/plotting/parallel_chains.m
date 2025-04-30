close all; clear; clc;
run plottingOptions.m

%% Load Data
% path_to_data = '../data/InstructionPinocchioFD_';
path_to_data = '../data/TimingPinocchioFD_';

expl_data = readmatrix([path_to_data, 'Explicit.csv']);
impl_data = readmatrix([path_to_data, 'Implicit.csv']);
expl_depths = unique(expl_data(:, 1))';
impl_depths = unique(impl_data(:, 1))';

for i = expl_depths
    caba.expl.depths{i} = expl_data(expl_data(:, 1) == i, [2 3]);
    pin_fd.expl.depths{i} = expl_data(expl_data(:, 1) == i, [2 5]);
    pin_cd1.expl.depths{i} = expl_data(expl_data(:, 1) == i, [2 6]);
    pin_cd2.expl.depths{i} = expl_data(expl_data(:, 1) == i, [2 7]);
    pin_cd5.expl.depths{i} = expl_data(expl_data(:, 1) == i, [2 8]);
    aba.expl.depths{i} = expl_data(expl_data(:, 1) == i, [2 9]);
    pin_aba.expl.depths{i} = expl_data(expl_data(:, 1) == i, [2 10]);
end

for i = impl_depths
    caba.impl.depths{i} = impl_data(impl_data(:, 1) == i, [2 3]);
    pin_fd.impl.depths{i} = impl_data(impl_data(:, 1) == i, [2 5]);
    pin_cd1.impl.depths{i} = impl_data(impl_data(:, 1) == i, [2 6]);
    pin_cd2.impl.depths{i} = impl_data(impl_data(:, 1) == i, [2 7]);
    pin_cd5.impl.depths{i} = impl_data(impl_data(:, 1) == i, [2 8]);
    aba.impl.depths{i} = impl_data(impl_data(:, 1) == i, [2 9]);
    pin_aba.impl.depths{i} = impl_data(impl_data(:, 1) == i, [2 10]);
end

if length(expl_depths) ~= 4
    error('Expected 4 depths, got %d', length(expl_depths))
end

if length(impl_depths) ~= 4
    error('Expected 4 depths, got %d', length(impl_depths))
end

%% Plot
axes = [[1 6 8e2 2e4];
         [1 9 2e3 1e5];
         [1 16 3e3 1e6];
         [1 21 1e4 1e6]];

caba_data{1} = caba.expl;
caba_data{2} = caba.impl;

pin_fd_data{1} = pin_fd.expl;
pin_fd_data{2} = pin_fd.impl;

pin_cd1_data{1} = pin_cd1.expl;
pin_cd1_data{2} = pin_cd1.impl;

pin_cd2_data{1} = pin_cd2.expl;
pin_cd2_data{2} = pin_cd2.impl;

pin_cd5_data{1} = pin_cd5.expl;
pin_cd5_data{2} = pin_cd5.impl;

aba_data{1} = aba.expl;
aba_data{2} = aba.impl;

pin_aba_data{1} = pin_aba.expl;
pin_aba_data{2} = pin_aba.impl;

data_names = {'Transmission', 'Connecting Rod'};

figure
size_scale = 3500;
size_ratio = 0.5;
set(gcf, 'Position', [200, 100, size_scale, size_scale * size_ratio]);

t = tiledlayout(2, 4);
t.TileSpacing = 'compact';
t.Padding = 'compact';

for j = 1:2
    col_idx = 1;

    for i = expl_depths

        sp_idx = 4 * (j - 1) + col_idx;

        nexttile
        hold on

        plotDepths(caba_data{j}.depths{i}, caba_opts)
        plotDepths(pin_fd_data{j}.depths{i}, pin_fd_opts)

        if j == 2
            plotDepths(pin_cd1_data{j}.depths{i}, pin_cd1_opts)
            plotDepths(pin_cd2_data{j}.depths{i}, pin_cd2_opts)
            plotDepths(pin_cd5_data{j}.depths{i}, pin_cd5_opts)
        end

        plotDepths(aba_data{j}.depths{i}, aba_opts)
        plotDepths(pin_aba_data{j}.depths{i}, pin_aba_opts)

        % Format applied to all subplots
        grid on
        set(gca, 'XScale', 'linear')
        % set(gca, 'YScale', 'log')
        set(gca, 'Fontsize', font_size)
        set(gca, 'TickLabelInterpreter', 'latex')
        set(gca, 'XLim', axes(col_idx, 1:2))
        ax = gca;
        % ax.YAxis.Exponent = 5;
        % ax.YRuler.ExponentMode = 'manual';
        text(1.15, 0.95 * get(gca).YLim(2), ['Loop Type = ', data_names{j}], text_opts)

        % Format Top row
        if j == 1
            title(['Tree Depth ($d_t$) = ', num2str(i)], 'Interpreter', 'latex')
        end

        % Format Bottom row
        if j == 2
            xlabel('Loop Size ($d_l$)', 'Interpreter', 'latex')
        end

        % Format Bottom Right
        if (col_idx == 4 && j == 2)
            l = legend();
            l.Location = 'Best';
            l.Interpreter = 'latex';
            l.Orientation = 'horizontal';
            l.IconColumnWidth = 80.;
            l.Position = [0.0423,0.491,0.921,0.0218];
            l.FontSize = 8.0;
        end

        col_idx = col_idx + 1;

    end

    % Format Left column
    if col_idx == 1
        ylabel('Instruction Count', 'Interpreter', 'latex')
    end

    col_idx = col_idx + 1;

end

saveas(gcf, '../figures/ParallelChains.png')

%% Helpers
function plotDepths(data, opts)
    plot(data(:, 1) / 2, data(:, 2), 'o-', opts)
end
