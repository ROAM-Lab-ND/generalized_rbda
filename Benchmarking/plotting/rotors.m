close all; clear; clc;
load('custom_colors.mat')

% csv format: # branches, cluster tree depth, c-aba instr count, pinochhio instr count
path_to_data = '../data/InstructionPinocchioFD_';

% TODO(@MatthewChignoli): Add these to custom colors?
green1 = [0.1, 0.8, 0.4]; % A lighter, minty green
green2 = [0.2, 0.6, 0.2]; % A medium, natural green
green3 = [0.0, 0.4, 0.0]; % A dark forest green

%% Systems
rev_chain.urdf_name = 'revolute_chain';
rev_chain.title = 'Link and Rotor';
rev_chain.axis = [1 10 0 4e4];
rev_chain.yscale = 'linear';
rev_chain.xscale = 'linear';
rev_chain.implicit = false;

rev_pair_chain.urdf_name = 'revolute_pair_chain';
rev_pair_chain.title = 'Parallel Belt Transmission';
rev_pair_chain.axis = [1 7 0 10e4];
rev_pair_chain.yscale = 'linear';
rev_pair_chain.xscale = 'linear';
rev_pair_chain.implicit = false;

four_bar.urdf_name = 'four_bar_chain';
four_bar.title = 'Four Bar Mechanism';
four_bar.axis = [1 7 0 10e4];
four_bar.yscale = 'linear';
four_bar.xscale = 'linear';
four_bar.implicit = true;

system_list{1} = rev_chain;
system_list{2} = rev_pair_chain;
system_list{3} = four_bar;

%% Plot
% Options
% symbols = {'o--', 's--', 'd--', '^--', '*--', 'p--', 'h--', 'x--', '+--'};
symbols = {'o--', 'o--', 'o--', 'o--', 'o--', 'o--', 'o--', 'o--', 'o--'};
plot_opts.MarkerSize = 4;
plot_opts.LineWidth = 2.0;
font_size = 16;

text_opts.Interpreter = 'latex';
text_opts.FontSize = font_size;
text_opts.Color = 'k';
text_opts.VerticalAlignment = 'top';

figure

% Set figure size
size_scale = 3500;
size_ratio = 0.5;
set(gcf, 'Position', [200, 100, size_scale, size_scale * size_ratio]);

for j = 1:length(system_list)
    sys = system_list{j};

    % Read data
    data = readmatrix([path_to_data, sys.urdf_name, '.csv']);
    branches = unique(data(:, 1))';
    branches = branches([1 2 3]);

    for i = branches
        caba.branches{i} = data(data(:, 1) == i, [2 3]);
        pin_fd.branches{i} = data(data(:, 1) == i, [2 5]);
        pin_cd1.branches{i} = data(data(:, 1) == i, [2 6]);
        pin_cd2.branches{i} = data(data(:, 1) == i, [2 7]);
        pin_cd5.branches{i} = data(data(:, 1) == i, [2 8]);
    end

    % Plot Data
    row_idx = 0;

    for i = branches
        sp_idx = 3 * row_idx + j;
        row_idx = row_idx + 1;
        subplot(length(branches), length(system_list), sp_idx)
        hold on

        plot_opts.Color = subdued_red;
        plot_opts.MarkerFaceColor = subdued_red;
        plot_opts.DisplayName = 'Constraint-Embedding ABA (Our Implementation)';
        plot(caba.branches{i}(:, 1), caba.branches{i}(:, 2), symbols{i}, plot_opts)

        plot_opts.Color = subdued_blue;
        plot_opts.MarkerFaceColor = subdued_blue;
        plot_opts.DisplayName = 'Cholesky (Pinocchio)';
        plot(pin_fd.branches{i}(:, 1), pin_fd.branches{i}(:, 2), symbols{i}, plot_opts)

        if sys.implicit
            plot_opts.Color = green1;
            plot_opts.MarkerFaceColor = green1;
            plot_opts.DisplayName = 'Proximal and Sparse, 1 iter (Pinocchio)';
            plot(pin_cd1.branches{i}(:, 1), pin_cd1.branches{i}(:, 2), symbols{i}, plot_opts)

            plot_opts.Color = green2;
            plot_opts.MarkerFaceColor = green2;
            plot_opts.DisplayName = 'Proximal and Sparse, 2 iter (Pinocchio)';
            plot(pin_cd2.branches{i}(:, 1), pin_cd2.branches{i}(:, 2), symbols{i}, plot_opts)

            plot_opts.Color = green3;
            plot_opts.MarkerFaceColor = green3;
            plot_opts.DisplayName = 'Proximal and Sparse, 5 iter (Pinocchio)';
            plot(pin_cd5.branches{i}(:, 1), pin_cd5.branches{i}(:, 2), symbols{i}, plot_opts)
        end

        % Format applied to all subplots
        grid on
        set(gca, 'YScale', sys.yscale)
        set(gca, 'XScale', sys.xscale)
        set(gca, 'Fontsize', font_size)
        set(gca, 'TickLabelInterpreter', 'latex')
        set(gca, 'XLim', sys.axis(1:2))
        ax = gca;
        ax.YAxis.Exponent = 3;
        ax.YRuler.ExponentMode = 'manual';
        text(1.15, 0.95 * get(gca).YLim(2), ['Branch Length ($b_a$) = ', num2str(i)], text_opts)
        
        % Format Left column
        if j == 1
            ylabel('Instruction Count', 'Interpreter', 'latex')
        end

    end

    % Format top row
    row_idx = 0;
    sp_idx = 3 * row_idx + j;
    subplot(length(branches), length(system_list), sp_idx)
    title(sys.title, 'Interpreter', 'latex')

    % Format bottom row
    row_idx = length(branches) - 1;
    sp_idx = 3 * row_idx + j;
    subplot(length(branches), length(system_list), sp_idx)
    xlabel('Branch Depth ($d_a$)', 'Interpreter', 'latex')

    % Format bottom right
    if j == length(system_list)
        l = legend();
        l.Location = 'Best';
        l.Interpreter = 'latex';
        l.Orientation = 'horizontal';
    end

end
