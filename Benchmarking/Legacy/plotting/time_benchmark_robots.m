%% Forward Dynamics
close all; clear; clc;
load('../Benchmarking/plotting/custom_colors.mat')
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
groupWidth = 1.0;
barWidth = groupWidth / num_algos;
offset = 0.5 * barWidth;

bar(x - 3 * offset, data(:, 5), barWidth, 'FaceColor', subdued_yellow, 'DisplayName', 'Approximate ABA');
bar(x - offset, data(:, 1), barWidth, 'FaceColor', subdued_red, 'DisplayName', 'C-ABA');
bar(x + offset, data(:, 4), barWidth, 'FaceColor', subdued_blue, 'DisplayName', 'Projection Method');
bar(x + 3 * offset, data(:, 3), barWidth, 'FaceColor', subdued_green, 'DisplayName', 'Lagrange Multipler Method');

% Customize the plot
ylabel('Time (ms)', 'Interpreter', 'latex')
legend('Location', 'best')
grid on
set(gca, 'Fontsize', 18)
set(gca, 'TickLabelInterpreter', 'latex')
set(gca, 'FontName', 'Times New Roman')

% Set the x-axis tick labels
xticks(x)
xticklabels({'12 Link Serial Chain (GT)', '12 Link Serial Chain (PBT)', 'Mini Cheetah', 'MIT Humanoid', 'Tello Humanoid', 'JVRC-1 Humanoid'})

% Adjust the y-axis limits
ylim([0 max(data, [], 'all') * 1.2])

% Save the figure
saveas(gcf, [path_to_figures, 'Robots.png'])

% Compute percent speed reductions/additions relative to C-ABA
for i = 1:size(data,1)
    disp(['Robot ', num2str(i)])
    disp(['Approximate ABA: ', num2str(100 * (data(i, 5) - data(i, 1)) / data(i, 1)), '%'])
    disp(['Projection Method: ', num2str(100 * (data(i, 4) - data(i, 1)) / data(i, 1)), '%'])
    disp(['Lagrange Multipler Method: ', num2str(100 * (data(i, 3) - data(i, 1)) / data(i, 1)), '%'])
end

%% Inverse Dynamics
clear;
load('../Benchmarking/plotting/custom_colors.mat')
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
groupWidth = 0.75;
barWidth = groupWidth / num_algos;
offset = 0.5 * barWidth;

bar(x - 2 * offset, data(:, 3), barWidth, 'FaceColor', subdued_yellow, 'DisplayName', 'Approximate RNEA');
bar(x, data(:, 1), barWidth, 'FaceColor', subdued_red, 'DisplayName', 'C-RNEA');
bar(x + 2 * offset, data(:, 2), barWidth, 'FaceColor', subdued_blue, 'DisplayName', 'Projected RNEA');

% Customize the plot
ylabel('Time (ms)', 'Interpreter', 'latex')
legend('Location', 'best')
grid on
set(gca, 'Fontsize', 18)
set(gca, 'TickLabelInterpreter', 'latex')
set(gca, 'FontName', 'Times New Roman')

% Set the x-axis tick labels
xticks(x)
xticklabels({'12 Link Serial Chain (GT)', '12 Link Serial Chain (PBT)', 'Mini Cheetah', 'MIT Humanoid', 'Tello Humanoid', 'JVRC-1 Humanoid'})

% Adjust the y-axis limits
ylim([0 max(data, [], 'all') * 1.2])

% Save the figure
saveas(gcf, [path_to_figures, 'Robots.png'])

% Compute percent speed reductions/additions relative to C-RNEA
disp(' ')
for i = 1:size(data,1)
    disp(['Robot ', num2str(i)])
    disp(['Approximate RNEA: ', num2str(100 * (data(i, 3) - data(i, 1)) / data(i, 1)), '%'])
    disp(['Projected RNEA: ', num2str(100 * (data(i, 2) - data(i, 1)) / data(i, 1)), '%'])
end

%% Apply Test Force
clear;
load('../Benchmarking/plotting/custom_colors.mat')
path_to_data = '../Benchmarking/data/TimingIOSIM_';
path_to_figures = '../Benchmarking/figures/TimingIOSIM_';

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
groupWidth = 0.75;
barWidth = groupWidth / num_algos;
offset = 0.5 * barWidth;

bar(x - 2 * offset, data(:, 3), barWidth, 'FaceColor', subdued_yellow, 'DisplayName', 'Approximate EFPA');
bar(x, data(:, 1), barWidth, 'FaceColor', subdued_red, 'DisplayName', 'C-EFPA');
bar(x + 2 * offset, data(:, 2), barWidth, 'FaceColor', subdued_blue, 'DisplayName', 'Projected IOSI');

% Customize the plot
ylabel('Time (ms)', 'Interpreter', 'latex')
legend('Location', 'best')
grid on
set(gca, 'Fontsize', 18)
set(gca, 'TickLabelInterpreter', 'latex')
set(gca, 'FontName', 'Times New Roman')

% Set the x-axis tick labels
xticks(x)
xticklabels({'12 Link Serial Chain (GT)', '12 Link Serial Chain (PBT)', 'Mini Cheetah', 'MIT Humanoid', 'Tello Humanoid', 'JVRC-1 Humanoid'})

% Adjust the y-axis limits
ylim([0 max(data, [], 'all') * 1.2])

% Save the figure
saveas(gcf, [path_to_figures, 'Robots.png'])

% Compute percent speed reductions/additions relative to C-EFPA
disp(' ')
for i = 1:size(data,1)
    disp(['Robot ', num2str(i)])
    disp(['Approximate EFPA: ', num2str(100 * (data(i, 3) - data(i, 1)) / data(i, 1)), '%'])
    disp(['Projected IOSI: ', num2str(100 * (data(i, 2) - data(i, 1)) / data(i, 1)), '%'])
end
