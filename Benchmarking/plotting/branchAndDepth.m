close all; clear; clc;
run plottingOptions.m

load('custom_colors.mat')

% path_to_data = '../data/InstructionPinocchioFD_';
path_to_data = '../data/TimingPinocchioFD_';

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
figure
size_scale = 3500;
size_ratio = 0.5;
set(gcf, 'Position', [200, 100, size_scale, size_scale * size_ratio]);

t = tiledlayout(3, length(system_list));
t.TileSpacing = 'compact';
t.Padding = 'compact';
t.TileIndexing = 'columnmajor';

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
        aba.branches{i} = data(data(:, 1) == i, [2 9]);
        pin_aba.branches{i} = data(data(:, 1) == i, [2 10]);
    end

    % Plot Data
    row_idx = 0;

    for i = branches
        row_idx = row_idx + 1;
        ax = nexttile;
        hold on

        plot(caba.branches{i}(:, 1), caba.branches{i}(:, 2), symbols{i}, caba_opts)
        plot(pin_fd.branches{i}(:, 1), pin_fd.branches{i}(:, 2), symbols{i}, pin_fd_opts)

        if sys.implicit
            plot(pin_cd1.branches{i}(:, 1), pin_cd1.branches{i}(:, 2), symbols{i}, pin_cd1_opts)
            plot(pin_cd2.branches{i}(:, 1), pin_cd2.branches{i}(:, 2), symbols{i}, pin_cd2_opts)
            plot(pin_cd5.branches{i}(:, 1), pin_cd5.branches{i}(:, 2), symbols{i}, pin_cd5_opts)
        end

        plot(aba.branches{i}(:, 1), aba.branches{i}(:, 2), symbols{i}, aba_opts)
        plot(pin_aba.branches{i}(:, 1), pin_aba.branches{i}(:, 2), symbols{i}, pin_aba_opts)

        % Format applied to all subplots
        grid on
        set(gca, 'YScale', sys.yscale)
        set(gca, 'XScale', sys.xscale)
        set(gca, 'Fontsize', font_size)
        set(gca, 'TickLabelInterpreter', 'latex')
        set(gca, 'XLim', sys.axis(1:2))

        % ax.YAxis.Exponent = 5;
        % ax.YRuler.ExponentMode = 'manual';

        if (row_idx ~= 1 ||  j ~= length(system_list))
            annotation('textbox', [ax.Position(1:2) 0.1 0.25], ...
                'String', ['Branch Length ($b_a$) = ', num2str(i)], ...
                text_opts)
        end

        % Format top row
        if row_idx == 1
            title(sys.title, 'Interpreter', 'latex')
        end

        % Format bottom row
        if row_idx == length(branches)
            xlabel('Branch Depth ($d_a$)', 'Interpreter', 'latex')
        end

        % Format Left column
        if j == 1
            ylabel('Instruction Count', 'Interpreter', 'latex')
        end

        % Format Bottom Right
        if (row_idx == length(branches) && j == length(system_list))
            l = legend();
            l.Location = 'Best';
            l.Interpreter = 'latex';
            l.Orientation = 'vertical';
            % l.IconColumnWidth = 80.;
            l.Position = [0.6796,0.8528,0.1982,0.1009];
        end

    end

end

saveas(gcf, '../figures/BranchAndDepth.png')
