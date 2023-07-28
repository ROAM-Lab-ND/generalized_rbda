close all; clear; clc;

path_to_data = '../Benchmarking/data/';
path_to_figures = '../Benchmarking/figures/';

% Tello
tello = readmatrix([path_to_data, 'CL_Tello.csv']);
plotTimingStats(tello, 'Tello')
saveas(gcf, [path_to_figures, 'CL_Tello.png'])
plotPassTiming(tello, 'Tello')
saveas(gcf, [path_to_figures, 'CL_Passes_Tello.png'])

function plotTimingStats(data, robot_name)
    figure
    bar(1, data, 'stacked', 'BarWidth', 0.25)
    legend('Forward Kinematics', 'Update Articulated Bodies', 'Foward Pass 1', 'External Forces', 'Backward Pass', 'Forward Pass 2', 'Interpreter', 'latex','Location','northwest')

    ylabel('Time (ms)','Interpreter','latex')
    title(['Forward Dynamics Timing for ', robot_name],'Interpreter','latex')
    grid on
    set(gca,'xlim',[0 1.2])
    set(gca, 'FontSize', 14)
    set(gca,'TickLabelInterpreter','latex')

end

function plotPassTiming(data_in, robot_name)
    data = zeros(size(data_in, 1), 3);
    data(:, 1) = data_in(:, 1) + data_in(:, 3);
    data(:, 2) = data_in(:, 2) + data_in(:, 5);
    data(:, 3) = data_in(:, 6);

    figure
    bar(1, data, 'stacked')
    legend('Foward Pass 1', 'Backward Pass', 'Forward Pass 2', 'Interpreter', 'latex','Location','northwest')

    ylabel('Time (ms)','Interpreter','latex')
    title(['Forward Dynamics Timing for ', robot_name],'Interpreter','latex')
    grid on
    set(gca, 'FontSize', 14)
    set(gca,'TickLabelInterpreter','latex')

end
