close all; clear; clc;
run plottingOptions.m

%% Load Data
data_type = 'Timing';
% data_type = 'Instruction';
path_to_data = ['../data/', data_type, 'PinocchioFD_'];

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
axes = [[1 6 2e-3 1e-1];
         [1 9 4e-3 2e-1];
         [1 16 1e-2 1e0];
         [1 21 1e-2 2e0]];

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
size_scale = 2200;
size_ratio = 0.6;
set(gcf, 'Position', [200, 100, size_scale, size_scale * size_ratio]);

t = tiledlayout(2, 5);
t.TileSpacing = 'compact';
t.Padding = 'compact';

for j = 1:2
    col_idx = 1;

    for i = [expl_depths 0]

        sp_idx = 4 * (j - 1) + col_idx;

        ax = nexttile;

        if i == 0
            continue
        end

        hold on
        plotDepths(caba_data{j}.depths{i}, caba_opts)
        plotDepths(aba_data{j}.depths{i}, aba_opts)
        
        plotDepths(pin_fd_data{j}.depths{i}, pin_fd_opts)

        if j == 2
            plotDepths(pin_cd1_data{j}.depths{i}, pin_cd1_opts)
            plotDepths(pin_cd2_data{j}.depths{i}, pin_cd2_opts)
            plotDepths(pin_cd5_data{j}.depths{i}, pin_cd5_opts)
        end

        
        plotDepths(pin_aba_data{j}.depths{i}, pin_aba_opts)

        % Format applied to all subplots
        grid on
        set(gca, 'XScale', 'linear')
        set(gca, 'YScale', 'log')
        set(gca, 'Fontsize', font_size)
        set(gca, 'TickLabelInterpreter', 'latex')
        set(gca, 'XLim', axes(col_idx, 1:2))
        set(gca, 'YLim', axes(col_idx, 3:4))
        % ax = gca;
        if strcmp(data_type, 'Instruction')
            ax.YAxis.Exponent = 5;
            ax.YRuler.ExponentMode = 'manual';
        end
        ax.YAxis.Exponent = -2;
        ax.YRuler.ExponentMode = 'manual';
        % text(1.15, 0.98 * get(gca).YLim(2), ['Loop Type: ', data_names{j}], text_opts)
        text(1.15, 0.96 * get(gca).YLim(2), ['Tree Depth ($d_t$) = ', num2str(i)], text_opts)

        % Format Top row
        % if j == 1
        %     title(['Tree Depth ($d_t$) = ', num2str(i)], 'Interpreter', 'latex')
        % end

        % Format Bottom row
        if j == 2
            xlabel('Loop Size ($d_l$)', 'Interpreter', 'latex')
        end

        % Format Bottom Right
        if (col_idx == 4 && j == 2)
            l = legend();
            % l.Location = 'Best';
            l.Interpreter = 'latex';
            % l.Orientation = 'horizontal';
            % l.IconColumnWidth = 80.;
            l.Position = [0.771161583432518,0.430801897780805,0.223156643931198,0.164939341238279];
            l.FontSize = 10.5;
        end

        % Format Left column
        if col_idx == 1

            if strcmp(data_type, 'Instruction')
                ylabel('Instruction Count', 'Interpreter', 'latex')
            else
                ylabel('Computation Time (ms)', 'Interpreter', 'latex')
            end

        end

        col_idx = col_idx + 1;

    end

    col_idx = col_idx + 1;

end

% Delete the two axes on the right to make space for the legend
delete(t.Children(1))
delete(t.Children(6))

saveas(gcf, ['../figures/', data_type, 'ParallelChains.png'])

%% Helpers
function plotDepths(data, opts)
    plot(data(:, 1) / 2, data(:, 2), 'o-', opts)
end
