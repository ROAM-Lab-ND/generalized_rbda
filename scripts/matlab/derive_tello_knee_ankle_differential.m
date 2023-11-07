function tkad = derive_tello_knee_ankle_differential_fn()
    import casadi.*

    N = 6; % gear reduction for both motors
    qr = SX.sym('qr', 2); % independent max coordinate: pre-gearbox rotor angle
    ql = SX.sym('ql', 2); % dependent max coordinate: joint (link) angle
    y = qr / N; % min coordinate: post-gearbox rotor angle
    qr_dot = SX.sym('qr_dot', 2);
    ql_dot = SX.sym('ql_dot', 2);
    ql_ddot = SX.sym('ql_ddot', 2);
    y_dot = qr_dot / N;
    q = [qr(1); qr(2); ql(1); ql(2)]; % maximal coordinate
    q_dot = [qr_dot(1); qr_dot(2); ql_dot(1); ql_dot(2)]; % maximal velocity

    % Inverse kinematics
    alpha_0 = (180 - 59.16) * pi / 180;
    beta_0 = (180 - 66) * pi / 180;
    beta_ = ql(2) + beta_0;
    a3 =- (21 * sin(pi / 9)) / 6250 - (7 * sin(beta_)) / 2500;
    b3 = 13/625 - (7 * cos(beta_)) / 2500 - (21 * cos(pi / 9)) / 6250;
    c3 = (273 * cos(pi / 9)) / 12500 + (91 * cos(beta_)) / 5000 - ...
        (147 * cos(pi / 9) * cos(beta_)) / 50000 - ...
        (147 * sin(pi / 9) * sin(beta_)) / 50000 -163349/6250000;
    d3 = sqrt(a3 ^ 2 + b3 ^ 2 - c3 ^ 2);
    alpha_ = atan2(b3 * d3 + a3 * c3, b3 * c3 - a3 * d3);

    ik_y = [ql(1) - (alpha_ - alpha_0); ...
                ql(1) + (alpha_ - alpha_0)]; % post-gearbox rotor angle
    ik_qr = N * ik_y; % pre-gearbox rotor angle

    ik_y_dot = jtimes(ik_y, ql, ql_dot); % post-gearbox rotor velocity
    ik_qr_dot = N * ik_y_dot; % pre-gearbox rotor velocity

    % Implicit constraint (position)
    phi = SX.sym('phi', 1, 2);
    phi(1) = ...
        (21 * cos(y(1) / 2 - y(2) / 2 + (1979 * pi) / 4500)) / 6250 - ...
        (13 * cos(y(1) / 2 - y(2) / 2 + (493 * pi) / 1500)) / 625 - (273 * cos(pi / 9)) / 12500 - ...
        (7 * sin(y(1) / 2 - y(2) / 2 + ql(2) + (231 * pi) / 500)) / 2500 + ...
        (91 * sin(ql(2) + (2 * pi) / 15)) / 5000 - (147 * sin(ql(2) + pi / 45)) / 50000 + ...
        163349/6250000;

    phi(2) = ql(1) - y(2) / 2 - y(1) / 2;

    % Differential kinematics (velocity)
    % s.t. ql_dot = J_drq_2_dql * qr_dot
    %      ql_dot = J_dy_2_dql * y_dot
    Ki = jacobian(phi, qr); % (2,2)
    Kd = jacobian(phi, ql); % (2,2)
    J_dqr_2_dql =- (Kd \ Ki); % (2,2)
    J_dy_2_dql = N * J_dqr_2_dql; %(2,2)

    % Get k = -K_dot * q_dot
    K = SX.zeros(2, 4);
    K(1:end, 3:end) = eye(2);
    K(1:end, 1:2) = -J_dqr_2_dql;
    K_qdot = K * q_dot;
    k = -jtimes(K_qdot, q, q_dot);

    % Get G
    G = SX.zeros(4, 2);
    G(1:2, 1:2) = N * eye(2);
    G(3:end, 1:2) = J_dy_2_dql;

    % Get g = G_dot * y_dot
    G_dot = jtimes(G, qr, qr_dot) + jtimes(G, ql, ql_dot); % (4,2)
    g = G_dot * y_dot; % (4,1)

    % Casadi functions
    tkad.phi = Function('tkad_phi', {qr, ql}, {phi});
    tkad.G = Function('tkad_G', {qr, ql}, {G});
    tkad.G_dot = Function('tkad_G_dot', {qr, ql, qr_dot, ql_dot}, {G_dot});
    tkad.g = Function('tkad_small_g', {qr, ql, qr_dot, ql_dot}, {g}); % CasADi function name is case insensitive
    tkad.K = Function('tkad_K', {qr, ql}, {K});
    tkad.k = Function('tkad_small_k', {qr, ql, qr_dot, ql_dot}, {k});
    tkad.IK_pos = Function('tkad_IK_pos', {ql}, {ik_y});
    tkad.IK_vel = Function('tkad_IK_vel', {ql, ql_dot}, {ik_y_dot});
    tkad.jacobian = Function('tkad_jacobian', {qr, ql}, {G, K});
    tkad.bias = Function('tkad_bias', {qr, ql, qr_dot, ql_dot}, {g, k});

    % TODO(@nicholasadr): is this needed?
    % % Generate C code
    % opts = struct('mex', true);
    % tkad.G.generate('gen_tkad_G.c', opts);
    % tkad.G_dot.generate('gen_tkad_G_dot.c', opts);
    % tkad.IK_pos.generate('gen_tkad_IK_ql_to_y', opts);
    % tkad.IK_vel.generate('gen_tkad_IK_dql_to_dy', opts);
