%% Trajectory Tracking
close all; clear; clc;

% Define RGB colors
purple = [0.5, 0.0, 1.0];
dark_green = [0.0, 0.5, 0.0];
gold = [0.8, 0.6, 0.0];

% Define paths for data and figures
path_to_data = '../Benchmarking/data/AccuracyEET_';
path_to_figures = '../Benchmarking/figures/AccuracyEET_';

%% End-Effector Trajectory Tracking Results
ee_data = readmatrix([path_to_data, 'HumanoidLeg_end_eff.csv']);
ee_trajectories = extractEndEffTrajectories(ee_data);

% Plot average tracking error over the sampled frequencies
figure
hold on
line_style = {'-', '--', ':'};
for i = 1:(length(ee_trajectories.omega)-1)
    plot(ee_trajectories.t, ee_trajectories.diag.error(i, :), line_style{i}, 'Linewidth', 2.0, 'MarkerSize', 8.0, 'Color', dark_green)
    plot(ee_trajectories.t, ee_trajectories.none.error(i, :), line_style{i}, 'Linewidth', 2.0, 'MarkerSize', 8.0, 'Color', purple)
end

xlabel('Time (s)', 'Interpreter', 'latex')
ylabel('End-Effector Tracking Error (m)', 'Interpreter', 'latex')
legend({['Approximate RNEA, $\omega = \frac{1}{4}$'], ...
        ['Unconstrained RNEA, $\omega = \frac{1}{4}$'], ...
        ['Approximate RNEA, $\omega = \frac{1}{2}$'], ...
        ['Unconstrained RNEA, $\omega = \frac{1}{2}$'], ...
        ['Approximate RNEA, $\omega = \frac{3}{4}$'], ...
        ['Unconstrained RNEA, $\omega = \frac{3}{4}$']}, ...
        'Location', 'northwest', 'Interpreter', 'latex')
grid on
set(gca, 'FontSize', 14,'TickLabelInterpreter','latex')

saveas(gcf, [path_to_figures, 'HumanoidLeg_end_eff.png'])

%% Torque Prediction Results
torque_data = readmatrix([path_to_data, 'HumanoidLeg_torque.csv']);
tau_trajectories = extractTorqueTrajectories(torque_data);

% Plot the torque trajectories for each  model
figure
joint_idx = 4;
plot(tau_trajectories.t, tau_trajectories.exact(joint_idx,:), '-', 'Linewidth', 2.0, 'MarkerSize', 8.0, 'Color', gold)
hold on
plot(tau_trajectories.t, tau_trajectories.diag(joint_idx,:), '-', 'Linewidth', 2.0, 'MarkerSize', 8.0, 'Color', dark_green)
plot(tau_trajectories.t, tau_trajectories.none(joint_idx,:), '-', 'Linewidth', 2.0, 'MarkerSize', 8.0, 'Color', purple)

xlabel('Time (s)', 'Interpreter', 'latex')
ylabel('Joint Torque (Nm)', 'Interpreter', 'latex')
legend({'C-RNEA', 'Approximate RNEA', 'Unconstrained RNEA'}, 'Location', 'northwest', 'Interpreter', 'latex')
grid on
set(gca, 'FontSize', 14,'TickLabelInterpreter','latex')
axis([-inf inf -12 20])

saveas(gcf, [path_to_figures, 'HumanoidLeg_torque.png'])

% Compute the RMS error of the torque trajectories
diag_error = rms(tau_trajectories.diag(joint_idx,:) - tau_trajectories.exact(joint_idx,:))
none_error = rms(tau_trajectories.none(joint_idx,:) - tau_trajectories.exact(joint_idx,:))

%% Helper Functions
function ee_trajectories = extractEndEffTrajectories(ee_data)
    % Separate data by wave frequency
    omega = unique(ee_data(:, 1));
    ee_trajectories.omega = omega;
    num_freq_sets = length(omega);
    freq_set = cell(num_freq_sets, 1);

    for i = 1:length(omega)
        freq_set{i} = ee_data(ee_data(:, 1) == omega(i), :);
    end

    % Extract the length of the trajectory
    t = unique(ee_data(:, 2));
    traj_length = length(t);

    % Collect the tracking error for each frequency
    exact_error = zeros(num_freq_sets, traj_length);
    diag_error = zeros(num_freq_sets, traj_length);
    none_error = zeros(num_freq_sets, traj_length);

    for i = 1:(num_freq_sets - 1)
        data_subset = freq_set{i};

        % Separate the data by model type
        ideal_traj = data_subset(1:traj_length, 3:5)';
        exact_traj = data_subset(traj_length + 1:2 * traj_length, 3:5)';
        diag_traj = data_subset(2 * traj_length + 1:3 * traj_length, 3:5)';
        none_traj = data_subset(3 * traj_length + 1:4 * traj_length, 3:5)';

        % Compute the norm of the error at every timestep
        for j = 1:traj_length
            exact_error(i, j) = norm(exact_traj(:, j) - ideal_traj(:, j));
            diag_error(i, j) = norm(diag_traj(:, j) - ideal_traj(:, j));
            none_error(i, j) = norm(none_traj(:, j) - ideal_traj(:, j));
        end

    end

    ee_trajectories.t = t;

    ee_trajectories.exact.error = exact_error;
    ee_trajectories.diag.error = diag_error;
    ee_trajectories.none.error = none_error;

    ee_trajectories.exact.mean = mean(exact_error, 1);
    ee_trajectories.diag.mean = mean(diag_error, 1);
    ee_trajectories.none.mean = mean(none_error, 1);

    ee_trajectories.exact.std = std(exact_error, 1);
    ee_trajectories.diag.std = std(diag_error, 1);
    ee_trajectories.none.std = std(none_error, 1);
end

function tau_trajectories = extractTorqueTrajectories(tau_data)
    omega = unique(tau_data(:, 1));
    data_subset = tau_data(tau_data(:, 1) == omega(end), :);
    tau_trajectories.t = data_subset(:, 2);
    tau_trajectories.exact = data_subset(:, 3:7)';
    tau_trajectories.diag = data_subset(:, 8:12)';
    tau_trajectories.none = data_subset(:, 13:17)';
end

function plotMean(t, traj, color)
    plot(t, traj.mean, '-', 'Linewidth', 2.0, 'MarkerSize', 8.0, 'Color', color)
end

function plotStd(t, traj, color)
    x = [t', fliplr(t')];
    y = [traj.mean - traj.std, fliplr(traj.mean + traj.std)];
    fill(x, y, color, 'EdgeColor', 'none', 'FaceAlpha', 0.3);
end
