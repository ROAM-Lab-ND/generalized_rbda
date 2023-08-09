function thd = derive_tello_hip_differential_fn()
    import casadi.*

    y = SX.sym('y',2);
    qd = SX.sym('qd',2);
    y_dot = SX.sym('y',2);
    qd_dot = SX.sym('qd_dot',2);
    qd_ddot = SX.sym('qd_ddot',2);

    % Inverse kinematics
    ik_y = SX.sym('ik_y',2);
    a1 = 57/2500 - (8*cos(qd(1))*sin(qd(2)))/625 - (7*sin(qd(1)))/625;
    b1 = -(8*cos(qd(2)))/625;
    c1 = (49*cos(qd(1)))/5000 + (399*sin(qd(1)))/20000 + ...
         (57*cos(qd(1))*sin(qd(2)))/2500 - (7*sin(qd(1))*sin(qd(2)))/625 - 3021/160000;
    d1 = sqrt(a1^2 + b1^2 - c1^2);
    a2 = (7*sin(qd(1)))/625 - (8*cos(qd(1))*sin(qd(2)))/625 + 57/2500;
    b2 = -(8*cos(qd(2)))/625;
    c2 = (49*cos(qd(1)))/5000 - (399*sin(qd(1)))/20000 + ...
         (57*cos(qd(1))*sin(qd(2)))/2500 + (7*sin(qd(1))*sin(qd(2)))/625 - 3021/160000;
    d2 = sqrt(a2^2 + b2^2 - c2^2);

    ik_y = [atan2(-b1*d1 + a1*c1, b1*c1+a1*d1);...
            atan2(-b2*d2 + a2*c2, b2*c2+a2*d2)];

    ik_y_dot = jtimes(ik_y, qd, qd_dot);
    dy_dqd = jacobian(y,qd);
    ik_y_ddot = dy_dqd * qd_ddot + jtimes(ik_y_dot,qd,qd_dot);

    % Implicit constraint (position)
    phi = SX.sym('phi',1,2);
    phi(1) = ...
        (57*sin(y(1)))/2500 - (49*cos(qd(1)))/5000 - (399*sin(qd(1)))/20000 - ...
        (8*cos(y(1))*cos(qd(2)))/625 - (57*cos(qd(1))*sin(qd(2)))/2500 - ...
        (7*sin(y(1))*sin(qd(1)))/625 + (7*sin(qd(1))*sin(qd(2)))/625 - ...
        (8*cos(qd(1))*sin(y(1))*sin(qd(2)))/625 + 3021/160000;
  
    phi(2) = ...
        (57*sin(y(2)))/2500 - (49*cos(qd(1)))/5000 + (399*sin(qd(1)))/20000 - ...
        (8*cos(y(2))*cos(qd(2)))/625 - (57*cos(qd(1))*sin(qd(2)))/2500 + ...
        (7*sin(y(2))*sin(qd(1)))/625 - (7*sin(qd(1))*sin(qd(2)))/625 - ...
        (8*cos(qd(1))*sin(y(2))*sin(qd(2)))/625 + 3021/160000;

    % Differential kinematics (velocity)
    Ki = jacobian(phi,y); % (2,2)
    Kd = jacobian(phi,qd); % (2,2)
    J_dy_2_dqd = -Kd\Ki; % (2,2)

    % Get k = -K_dot * q_dot
    K = SX.zeros(2,4);
    K(1:end,3:end) = eye(2);
    K(1:end,1:2) = -J_dy_2_dqd;
    q = [y(1); y(2); qd(1); qd(2)];
    q_dot = [y_dot(1); y_dot(2); qd_dot(1); qd_dot(2)];
    K_qdot = K * q_dot;
    B = jtimes(K,q,q_dot);
    k = -jtimes(K_qdot,q,q_dot);

    % Get G
    G = SX.zeros(4,2);
    G(1:2,1:2) = eye(2);
    G(3:end,1:2) = J_dy_2_dqd;

    % Get g = G_dot * y_dot
    G_dot = jtimes(G,qd,qd_dot) + jtimes(G,y,y_dot); % (4,2)
    g = G_dot * y_dot; % (4,1)

    % Casadi functions
    thd.phi = Function('thd_phi', {y,qd}, {phi});
    thd.G = Function('thd_G',{y,qd,y_dot,qd_dot}, {G});
    thd.G_dot = Function('thd_G_dot',{y,qd,y_dot,qd_dot}, {G_dot});
    thd.g = Function('thd_small_g', {y,qd,y_dot,qd_dot}, {g}); % CasADi function name is case insensitive
    thd.K = Function('thd_K',{y,qd,y_dot,qd_dot}, {K});
    thd.k = Function('thd_small_k', {y,qd,y_dot,qd_dot}, {k});
    thd.IK_pos = Function('thd_IK_pos', {qd}, {ik_y});
    thd.IK_vel = Function('thd_IK_vel', {qd,qd_dot}, {ik_y_dot});
    thd.IK_acc = Function('thd_IK_acc', {qd,qd_dot,qd_ddot}, {ik_y_ddot});
    thd.jacobian = Function('thd_jacobian', {y,qd}, {G,K});
    thd.bias = Function('thd_bias', {y,qd,y_dot,qd_dot}, {g,k});
    

