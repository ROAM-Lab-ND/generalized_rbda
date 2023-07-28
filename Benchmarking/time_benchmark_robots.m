close all; clear; clc;

path_to_data = '../Benchmarking/data/Timing_';
path_to_figures = '../Benchmarking/figures/Timing_';


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

bar(x - 3 * offset, data(:, 5), barWidth, 'y', 'DisplayName', 'ABA w/ Reflected Inertia');
bar(x - offset, data(:, 1), barWidth, 'r', 'DisplayName', 'Cluster-Based ABA');
bar(x + offset, data(:, 4), barWidth, 'b', 'DisplayName', 'Projection Method');
bar(x + 3 * offset, data(:, 3), barWidth, 'g', 'DisplayName', 'Lagrange Multipler Method');

% Customize the plot
xlabel('Robot')
ylabel('Time (ms)')
title('Forward Dynamics Benchmark')
legend('Location','best')

% Set the x-axis tick labels
xticks(x)
xticklabels({'Tello','Humanoid', 'Mini Cheetah'})

% Adjust the y-axis limits
ylim([0 max(data,[],'all') * 1.2])

% Save the figure
saveas(gcf, [path_to_figures, 'Robots.png'])
