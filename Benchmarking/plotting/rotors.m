close all; clear; clc;

% csv format: # branches, cluster tree depth, c-aba instr count, pinochhio instr count

path_to_data = '../data/InstructionPinocchioFD_';

%% Systems
rev_chain.urdf_name = 'revolute_chain';
rev_chain.title = 'Link and Rotor';
rev_chain.axis = [1 10 0 4e4];
rev_chain.yscale = 'linear';

rev_pair_chain.urdf_name = 'revolute_pair_chain';
rev_pair_chain.title = 'Parallel Belt Transmission';
rev_pair_chain.axis = [1 7 0 10e4];
rev_pair_chain.yscale = 'linear';

four_bar = rev_pair_chain;
four_bar.title = 'Four Bar (WIP)';

%% Plotting options
symbols = {'o--', 's--', 'd--', '^--', '*--', 'p--', 'h--', 'x--', '+--'};
plot_opts.MarkerSize = 10;
plot_opts.LineWidth = 2.0;
font_size = 16;

%% Plot
system_list{1} = rev_chain;
system_list{2} = rev_pair_chain;
system_list{3} = four_bar;

figure
for j = 1:length(system_list)
    sys = system_list{j};

    % Read data
    data = readmatrix([path_to_data, sys.urdf_name, '.csv']);
    branches = unique(data(:, 1))';

    for i = branches
        caba.branches{i} = data(data(:, 1) == i, [2 3]);
        pin.branches{i} = data(data(:, 1) == i, [2 4]);
    end

    % Dummy entries for the legend
    subplot(1, length(system_list), j)
    plot_opts.HandleVisibility = 'on';
    hold on

    for i = branches
        plot_opts.Color = 'k';
        plot_opts.MarkerFaceColor = 'k';
        plot_opts.DisplayName = ['$b_a$ =', num2str(i)];
        plot(NaN, NaN, symbols{i}, plot_opts)
    end

    plot_opts.Color = 'r';
    plot_opts.DisplayName = 'Constraint-Embedding ABA (ours)';
    plot(NaN, NaN, '-', plot_opts)

    plot_opts.Color = 'b';
    plot_opts.DisplayName = 'Proximal and Sparse (Pinocchio)';
    plot(NaN, NaN, '-', plot_opts)

    % Plot Data
    for i = branches
        plot_opts.Color = 'r';
        plot_opts.MarkerFaceColor = 'r';
        plot_opts.HandleVisibility = 'off';
        plot(caba.branches{i}(:, 1), caba.branches{i}(:, 2), symbols{i}, plot_opts)

        plot_opts.Color = 'b';
        plot_opts.MarkerFaceColor = 'b';
        plot(pin.branches{i}(:, 1), pin.branches{i}(:, 2), symbols{i}, plot_opts)
    end

    % Format
    grid on
    axis(sys.axis)

    xlabel('$d_a$', 'Interpreter', 'latex')
    ylabel('Instruction Count', 'Interpreter', 'latex')
    title(sys.title, 'Interpreter', 'latex')
    legend('Location', 'Best', 'Interpreter', 'latex')

    set(gca, 'YScale', sys.yscale)
    set(gca, 'Fontsize', font_size)
    set(gca, 'TickLabelInterpreter', 'latex')

end
