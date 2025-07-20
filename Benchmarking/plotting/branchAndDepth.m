close all; clear; clc;
run plottingOptions.m

data_type = 'Timing';
% data_type = 'Instruction';
% data_type = 'ErrorL2';
path_to_data = ['../data/', data_type, 'PinocchioFD_'];

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

font_size = 14;
text_opts.FontSize = 14;

%% Plot
figure
size_scale = 1500;cd 
size_ratio = 0.45;
set(gcf, 'Position', [200, 100, size_scale, size_scale * size_ratio]);

t = tiledlayout(3, length(system_list)+1);
t.TileSpacing = 'compact';
t.Padding = 'compact';
t.TileIndexing = 'rowmajor';

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

    for i = [branches 0]
        row_idx = row_idx + 1;
        ax = nexttile;

        if i == 0
            continue
        end

        hold on

        plot(caba.branches{i}(:, 1), caba.branches{i}(:, 2), caba_opts)
        plot(aba.branches{i}(:, 1), aba.branches{i}(:, 2), aba_opts)

        plot(pin_fd.branches{i}(:, 1), pin_fd.branches{i}(:, 2), pin_fd_opts)

        if sys.implicit
            plot(pin_cd1.branches{i}(:, 1), pin_cd1.branches{i}(:, 2), pin_cd1_opts)
            plot(pin_cd2.branches{i}(:, 1), pin_cd2.branches{i}(:, 2), pin_cd2_opts)
            plot(pin_cd5.branches{i}(:, 1), pin_cd5.branches{i}(:, 2), pin_cd5_opts)
        end

        plot(pin_aba.branches{i}(:, 1), pin_aba.branches{i}(:, 2), pin_aba_opts)

        % Format applied to all subplots
        grid on
        set(gca, 'YScale', sys.yscale)
        set(gca, 'XScale', sys.xscale)
        set(gca, 'Fontsize', font_size)
        set(gca, 'TickLabelInterpreter', 'latex')
        set(gca, 'XLim', sys.axis(1:2))

        if strcmp(data_type, 'Instruction')
            ax.YAxis.Exponent = 5;
            ax.YRuler.ExponentMode = 'manual';
        end

        if (row_idx ~= length(branches) || j ~= 2)
            annotation('textbox', [ax.Position(1:2)+[0.01 0] 0.1 0.26], ...
                'String', ['Branches ($b_a$) = ', num2str(i)], ...
                text_opts)
        end

        % % Format top row
        % if row_idx == 1
        %     title(sys.title, 'Interpreter', 'latex')
        % end

        % Format bottom row
        if j == length(system_list)
            xlabel('Branch Depth ($d_a$)', 'Interpreter', 'latex')
        end

        % Format Left column
        if row_idx == 1

            if strcmp(data_type, 'Instruction')
                ylabel('Instruction Count', 'Interpreter', 'latex')
            else
                ylabel({'Computation Time (ms)','via Code Generation'}, 'Interpreter', 'latex')
            end

        end

        % Format Bottom Right
        if (row_idx == length(branches) && j == 3)
            l = legend();
            set(l, 'ItemTokenSize', [18, 15]);
            l.Location = 'Best';
            l.Interpreter = 'latex';
            l.Orientation = 'vertical';
            l.Position = [0.519333333333333,0.506469047193037,0.13101118648021,0.134814814814815];
            l.FontSize = 10;
        end

    end

end

% Delete the two axes on the right to make space for the mechanism diagrams
delete(t.Children(1))
delete(t.Children(5))
delete(t.Children(8))

saveas(gcf, ['../figures/', data_type, 'branchAndDepth.png'])
exportgraphics(gcf,['../figures/', data_type, 'branchAndDepth.pdf'],'ContentType','vector')
