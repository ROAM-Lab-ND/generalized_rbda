load('custom_colors.mat')

caba_color = subdued_red;
aba_color  = [255, 180, 66] / 255;
pin_fd_color  = [0.1, 0.4, 0.7];       % Steel blue
pin_cd1_color = [0.6, 0.9, 0.6];       % Mint green
pin_cd2_color = [0.3, 0.7, 0.3];       % Medium green
pin_cd5_color = [0.1, 0.5, 0.1];       % Darker green
pin_aba_color = [0.7, 0.5, 1.0];       % Light violet

marker_size = 6;
line_width = 2.0;
font_size = 14;

grbda_marker = 'o';
grbda_line = '-';

pin_marker = '^';
pin_line = '--';

plot_opts.MarkerSize = marker_size;
plot_opts.LineWidth = line_width;

text_opts.Interpreter = 'latex';
text_opts.FontSize = 12.;
text_opts.Color = 'k';
text_opts.VerticalAlignment = 'top';
text_opts.EdgeColor = 'none';

caba_opts = plot_opts;
caba_opts.Color = caba_color;
caba_opts.MarkerFaceColor = caba_opts.Color;
caba_opts.Marker = grbda_marker;
caba_opts.LineStyle = grbda_line;
caba_opts.DisplayName = 'grbda::constraintEmbeddingABA()';

pin_fd_opts = plot_opts;
pin_fd_opts.Color = pin_fd_color;
pin_fd_opts.MarkerFaceColor = pin_fd_opts.Color;
pin_fd_opts.Marker = pin_marker;
pin_fd_opts.LineStyle = pin_line;
pin_fd_opts.DisplayName = 'pinocchio::forwardDynamics()';

pin_cd1_opts = plot_opts;
pin_cd1_opts.Color = pin_cd1_color;
pin_cd1_opts.MarkerFaceColor = pin_cd1_opts.Color;
pin_cd1_opts.Marker = pin_marker;
pin_cd1_opts.LineStyle = pin_line;
pin_cd1_opts.DisplayName = 'pinocchio::constraintDynamics(), 1 iter';

pin_cd2_opts = plot_opts;
pin_cd2_opts.Color = pin_cd2_color;
pin_cd2_opts.MarkerFaceColor = pin_cd2_opts.Color;
pin_cd2_opts.Marker = pin_marker;
pin_cd2_opts.LineStyle = pin_line;
pin_cd2_opts.DisplayName = 'pinocchio::constraintDynamics(), 2 iter';

pin_cd5_opts = plot_opts;
pin_cd5_opts.Color = pin_cd5_color;
pin_cd5_opts.MarkerFaceColor = pin_cd5_opts.Color;
pin_cd5_opts.Marker = pin_marker;
pin_cd5_opts.LineStyle = pin_line;
pin_cd5_opts.DisplayName = 'pinocchio::constraintDynamics(), 5 iter';

aba_opts = plot_opts;
aba_opts.Color  = aba_color;
aba_opts.MarkerFaceColor = aba_opts.Color;
aba_opts.Marker = grbda_marker;
aba_opts.LineStyle = grbda_line;
aba_opts.DisplayName = 'grbda::aba()';

pin_aba_opts = plot_opts;
pin_aba_opts.Color = pin_aba_color;
pin_aba_opts.MarkerFaceColor = pin_aba_opts.Color;
pin_aba_opts.Marker = pin_marker;
pin_aba_opts.LineStyle = pin_line;
pin_aba_opts.DisplayName = 'pinocchio::aba()';
