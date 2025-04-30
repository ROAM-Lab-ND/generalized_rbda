load('custom_colors.mat')
green1 = [0.1, 0.8, 0.4]; % A lighter, minty green
green2 = [0.2, 0.6, 0.2]; % A medium, natural green
green3 = [0.0, 0.4, 0.0]; % A dark forest green

% symbols = {'o--', 's--', 'd--', '^--', '*--', 'p--', 'h--', 'x--', '+--'};
symbols = {'o--', 'o--', 'o--', 'o--', 'o--', 'o--', 'o--', 'o--', 'o--'};

marker_size = 8;
line_width = 2.0;
font_size = 14;

plot_opts.MarkerSize = marker_size;
plot_opts.LineWidth = line_width;

text_opts.Interpreter = 'latex';
text_opts.FontSize = font_size;
text_opts.Color = 'k';
text_opts.VerticalAlignment = 'top';
text_opts.EdgeColor = 'none';

caba_opts = plot_opts;
caba_opts.Color = subdued_red;
caba_opts.MarkerFaceColor = caba_opts.Color;
caba_opts.DisplayName = 'Constraint-Embedding ABA (Our Implementation)';

pin_fd_opts = plot_opts;
pin_fd_opts.Color = subdued_blue;
pin_fd_opts.MarkerFaceColor = pin_fd_opts.Color;
pin_fd_opts.DisplayName = 'Cholesky (Pinocchio)';

pin_cd1_opts = plot_opts;
pin_cd1_opts.Color = [0.1, 0.8, 0.4]; % A lighter, minty green
pin_cd1_opts.MarkerFaceColor = pin_cd1_opts.Color;
pin_cd1_opts.DisplayName = 'Proximal and Sparse, 1 iter (Pinocchio)';

pin_cd2_opts = plot_opts;
pin_cd2_opts.Color = [0.2, 0.6, 0.2]; % A medium, natural green
pin_cd2_opts.MarkerFaceColor = pin_cd2_opts.Color;
pin_cd2_opts.DisplayName = 'Proximal and Sparse, 2 iter (Pinocchio)';

pin_cd5_opts = plot_opts;
pin_cd5_opts.Color = [0.0, 0.4, 0.0]; % A dark forest green
pin_cd5_opts.MarkerFaceColor = pin_cd5_opts.Color;
pin_cd5_opts.DisplayName = 'Proximal and Sparse, 5 iter (Pinocchio)';

aba_opts = plot_opts;
aba_opts.Color  = [255, 141, 161] / 255; % A light pink 
aba_opts.MarkerFaceColor = aba_opts.Color;
aba_opts.DisplayName = 'ABA (Our Implementation)';

pin_aba_opts = plot_opts;
pin_aba_opts.Color = [222, 65, 255] / 255; % A light magenta
pin_aba_opts.MarkerFaceColor = pin_aba_opts.Color;
pin_aba_opts.DisplayName = 'ABA (Pinocchio)';
