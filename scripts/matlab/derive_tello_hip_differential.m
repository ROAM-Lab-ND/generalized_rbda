function thd = derive_tello_hip_differential_fn()
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
    ik_y = SX.sym('ik_y', 2);
    a1 = 57/2500 - (8 * cos(ql(1)) * sin(ql(2))) / 625 - (7 * sin(ql(1))) / 625;
    b1 =- (8 * cos(ql(2))) / 625;
    c1 = (49 * cos(ql(1))) / 5000 + (399 * sin(ql(1))) / 20000 + ...
        (57 * cos(ql(1)) * sin(ql(2))) / 2500 - (7 * sin(ql(1)) * sin(ql(2))) / 625 -3021/160000;
    d1 = sqrt(a1 ^ 2 + b1 ^ 2 - c1 ^ 2);
    a2 = (7 * sin(ql(1))) / 625 - (8 * cos(ql(1)) * sin(ql(2))) / 625 +57/2500;
    b2 =- (8 * cos(ql(2))) / 625;
    c2 = (49 * cos(ql(1))) / 5000 - (399 * sin(ql(1))) / 20000 + ...
        (57 * cos(ql(1)) * sin(ql(2))) / 2500 + (7 * sin(ql(1)) * sin(ql(2))) / 625 -3021/160000;
    d2 = sqrt(a2 ^ 2 + b2 ^ 2 - c2 ^ 2);

    ik_y = [atan2(-b1 * d1 + a1 * c1, b1 * c1 + a1 * d1); ...
                atan2(-b2 * d2 + a2 * c2, b2 * c2 + a2 * d2)]; % post-gearbox rotor angle
    ik_qr = N * ik_y; % pre-gearbox rotor angle

    ik_y_dot = jtimes(ik_y, ql, ql_dot); % post-gearbox rotor velocity
    ik_qr_dot = N * ik_y_dot; % pre-gearbox rotor velocity

    % Implicit constraint (position)
    phi = SX.sym('phi', 1, 2);
    phi(1) = ...
        (57 * sin(y(1))) / 2500 - (49 * cos(ql(1))) / 5000 - (399 * sin(ql(1))) / 20000 - ...
        (8 * cos(y(1)) * cos(ql(2))) / 625 - (57 * cos(ql(1)) * sin(ql(2))) / 2500 - ...
        (7 * sin(y(1)) * sin(ql(1))) / 625 + (7 * sin(ql(1)) * sin(ql(2))) / 625 - ...
        (8 * cos(ql(1)) * sin(y(1)) * sin(ql(2))) / 625 +3021/160000;

    phi(2) = ...
        (57 * sin(y(2))) / 2500 - (49 * cos(ql(1))) / 5000 + (399 * sin(ql(1))) / 20000 - ...
        (8 * cos(y(2)) * cos(ql(2))) / 625 - (57 * cos(ql(1)) * sin(ql(2))) / 2500 + ...
        (7 * sin(y(2)) * sin(ql(1))) / 625 - (7 * sin(ql(1)) * sin(ql(2))) / 625 - ...
        (8 * cos(ql(1)) * sin(y(2)) * sin(ql(2))) / 625 +3021/160000;

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
    thd.phi = Function('thd_phi', {qr, ql}, {phi});
    thd.G = Function('thd_G', {qr, ql}, {G});
    thd.G_dot = Function('thd_G_dot', {qr, ql, qr_dot, ql_dot}, {G_dot});
    thd.g = Function('thd_small_g', {qr, ql, qr_dot, ql_dot}, {g}); % CasADi function name is case insensitive
    thd.K = Function('thd_K', {qr, ql}, {K});
    thd.k = Function('thd_small_k', {qr, ql, qr_dot, ql_dot}, {k});
    thd.IK_pos = Function('thd_IK_pos', {ql}, {ik_y});
    thd.IK_vel = Function('thd_IK_vel', {ql, ql_dot}, {ik_y_dot});
    thd.jacobian = Function('thd_jacobian', {qr, ql}, {G, K});
    thd.bias = Function('thd_bias', {qr, ql, qr_dot, ql_dot}, {g, k});
