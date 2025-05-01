close all; clear; clc;
run plottingOptions.m

%% Load data
data_type = 'Timing';
% data_type = 'Instruction';
path_to_data = ['../data/', data_type, 'PinocchioFD_Approx'];

data = readmatrix(path_to_data);
data_as_text = readmatrix(path_to_data,'OutputType','string');

num_robots = size(data,1);
num_algos = size(data, 2) - 1;

%% Systems
mini_cheetah.urdf_name = 'mini_cheetah';
mini_cheetah.title = 'Mini Cheetah';

mit_humanoid.urdf_name = 'mit_humanoid';
mit_humanoid.title = 'MIT Humanoid';

tello.urdf_name = 'tello';
tello.title = 'Tello Humanoid';

jvrc1.urdf_name = 'jvrc1';
jvrc1.title = 'JVRC1';

system_list{1} = mini_cheetah;
system_list{2} = mit_humanoid;
system_list{3} = tello;
system_list{4} = jvrc1;

%% Plot
% Options
font_size = 20;
groupWidth = 0.5;
barWidth = groupWidth / num_algos;
offset = 0.5 * barWidth;
plot_opts = struct;

% Create vector for x-axis tick labels
spacing = 1;
x = 1:spacing:(spacing * num_robots);

% Create a figure and axis
figure
size_scale = 1800;
size_ratio = 0.55;
set(gcf, 'Position', [200, 100, size_scale, size_scale * size_ratio]);

hold on

bar(x - 2 * offset, data(:, 3), barWidth, 'FaceColor', aba_color, 'DisplayName', 'Approximate ABA (GRBDA)');
bar(x, data(:, 2), barWidth,  'FaceColor', subdued_red, 'DisplayName', 'Constraint-Embedding ABA (GRBDA)');
bar(x + 2 * offset, data(:, 4), barWidth, 'FaceColor', pin_fd_color, 'DisplayName', 'Cholesky (Pinocchio)');

% Customize the plot
if strcmp(data_type, 'Instruction')
    ylabel('Instruction Count', 'Interpreter', 'latex')
else
    ylabel('Computation Time (ms)', 'Interpreter', 'latex')
end

l = legend();
l.Interpreter = 'latex';
l.Position = [0.139916488272131,0.737218813905932,0.434527956172313,0.174964447427726];
grid on
set(gca, 'Fontsize', 20)
set(gca, 'TickLabelInterpreter', 'latex')
set(gca, 'FontName', 'Times New Roman')

% Adjust the y-axis limits
xlim([0.5 num_robots+0.5])
ylim([0 max(data, [], 'all') * 1.2])

% Set the x-axis tick labels
xticks(x)
xticklabels({'Mini Cheetah', 'MIT Humanoid', 'Tello Humanoid', 'JVRC1'})

saveas(gcf, ['../figures/', data_type, 'Approx.png'])
