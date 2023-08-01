%% Forward Dynamics
close all; clear; clc;
path_to_data = '../Benchmarking/data/TimingFD_';
path_to_figures = '../Benchmarking/figures/TimingFD_';

% load the data
data = readmatrix([path_to_data, 'Robots.csv']);

num_robots = length(data(:, 1));
num_algos = length(data(1, :));

% Create vector for x-axis tick labels
spacing = 1;
x = 1:spacing:(spacing * num_robots);

% Create a figure and axis
figure
hold on

% Define the width of each bar group
groupWidth = 0.6;
barWidth = groupWidth / num_algos;
offset = 0.5 * barWidth;

bar(x - 3 * offset, data(:, 5), barWidth, 'y', 'DisplayName', 'ABA');
bar(x - offset, data(:, 1), barWidth, 'r', 'DisplayName', 'C-ABA');
bar(x + offset, data(:, 4), barWidth, 'b', 'DisplayName', 'Projection Method');
bar(x + 3 * offset, data(:, 3), barWidth, 'g', 'DisplayName', 'Lagrange Multipler Method');

% Customize the plot
ylabel('Time (ms)','Interpreter','latex')
legend('Location', 'best')
grid on
set(gca,'Fontsize',18)
set(gca,'TickLabelInterpreter','latex')
set(gca,'FontName','Times New Roman')

% Set the x-axis tick labels
xticks(x)
xticklabels({'Mini Cheetah', 'Tello', 'MIT Humanoid', 'Tello w/ Arms'})

% Adjust the y-axis limits
ylim([0 max(data, [], 'all') * 1.2])

% Save the figure
saveas(gcf, [path_to_figures, 'Robots.png'])

%% Inverse Dynamics
clear;
path_to_data = '../Benchmarking/data/TimingID_';
path_to_figures = '../Benchmarking/figures/TimingID_';

% load the data
data = readmatrix([path_to_data, 'Robots.csv']);

num_robots = length(data(:, 1));
num_algos = length(data(1, :));

% Create vector for x-axis tick labels
spacing = 1;
x = 1:spacing:(spacing * num_robots);

% Create a figure and axis
figure
hold on

% Define the width of each bar group
groupWidth = 0.6;
barWidth = groupWidth / num_algos;
offset = 0.5 * barWidth;

bar(x - 2 * offset, data(:, 3), barWidth, 'y', 'DisplayName', 'Approximate RNEA');
bar(x, data(:, 1), barWidth, 'r', 'DisplayName', 'C-RNEA');
bar(x + 2 * offset, data(:, 2), barWidth, 'b', 'DisplayName', 'Projected RNEA');

% Customize the plot
ylabel('Time (ms)','Interpreter','latex')
legend('Location', 'best')
grid on
set(gca,'Fontsize',18)
set(gca,'TickLabelInterpreter','latex')
set(gca,'FontName','Times New Roman')

% Set the x-axis tick labels
xticks(x)
xticklabels({'Mini Cheetah', 'Tello', 'MIT Humanoid', 'Tello w/ Arms'})

% Adjust the y-axis limits
ylim([0 max(data, [], 'all') * 1.2])

% Save the figure
saveas(gcf, [path_to_figures, 'Robots.png'])

%%
clear;
path_to_data = '../Benchmarking/data/TimingATF_';
path_to_figures = '../Benchmarking/figures/TimingATF_';

% load the data
data = readmatrix([path_to_data, 'Robots.csv']);

num_robots = length(data(:, 1));
num_algos = length(data(1, :));

% Create vector for x-axis tick labels
spacing = 1;
x = 1:spacing:(spacing * num_robots);

% Create a figure and axis
figure
hold on

% Define the width of each bar group
groupWidth = 0.6;
barWidth = groupWidth / num_algos;
offset = 0.5 * barWidth;

bar(x - 2 * offset, data(:, 3), barWidth, 'y', 'DisplayName', 'EFPA');
bar(x, data(:, 2), barWidth, 'r', 'DisplayName', 'C-EFPA');
bar(x + 2 * offset, data(:, 1), barWidth, 'b', 'DisplayName', 'Projected IOSI');

% Customize the plot
ylabel('Time (ms)'  ,'Interpreter','latex')
legend('Location', 'best')
grid on
set(gca,'Fontsize',18)
set(gca,'TickLabelInterpreter','latex')
set(gca,'FontName','Times New Roman')

% Set the x-axis tick labels
xticks(x)
xticklabels({'Mini Cheetah', 'Tello', 'MIT Humanoid', 'Tello w/ Arms'})

% Adjust the y-axis limits
ylim([0 max(data, [], 'all') * 1.2])

% Save the figure
saveas(gcf, [path_to_figures, 'Robots.png'])
