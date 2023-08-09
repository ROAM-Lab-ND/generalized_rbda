function tkad = derive_tello_knee_ankle_differential_fn()
    import casadi.*

    y = SX.sym('y',2);
    qd = SX.sym('qd',2);
    y_dot = SX.sym('y',2);
    qd_dot = SX.sym('qd_dot',2);
    qd_ddot = SX.sym('qd_ddot',2);
 
    % Inverse kinematics
    alpha_0 = (180-59.16)*pi/180;
    beta_0 = (180-66)*pi/180;
    beta_ = qd(2) + beta_0;
    a3 = - (21*sin(pi/9))/6250 - (7*sin(beta_))/2500;
    b3 = 13/625 - (7*cos(beta_))/2500 - (21*cos(pi/9))/6250;
    c3 = (273*cos(pi/9))/12500 + (91*cos(beta_))/5000 - ...
         (147*cos(pi/9)*cos(beta_))/50000 - ...
         (147*sin(pi/9)*sin(beta_))/50000 - 163349/6250000;
    d3 = sqrt(a3^2 + b3^2 - c3^2);
    alpha_ = atan2( b3*d3 + a3*c3, b3*c3-a3*d3);

    ik_y = [qd(1) - (alpha_ - alpha_0);...
            qd(1) + (alpha_ - alpha_0)];

    ik_y_dot = jtimes(ik_y, qd, qd_dot);
    dy_dqd = jacobian(y,qd);
    ik_y_ddot = dy_dqd * qd_ddot + jtimes(ik_y_dot,qd,qd_dot);

    % Implicit constraint (position)
    phi = SX.sym('phi',1,2);
    phi(1) = ...
        (21*cos(y(1)/2 - y(2)/2 + (1979*pi)/4500))/6250 - ...
        (13*cos(y(1)/2 - y(2)/2 + (493*pi)/1500))/625 - (273*cos(pi/9))/12500 - ...
        (7*sin(y(1)/2 - y(2)/2 + qd(2) + (231*pi)/500))/2500 + ...
        (91*sin(qd(2) + (2*pi)/15))/5000 - (147*sin(qd(2) + pi/45))/50000 + ...
        163349/6250000;
  
    phi(2) = qd(1) - y(2)/2 - y(1)/2;

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
    tkad.phi = Function('tkad_phi', {y,qd}, {phi});
    tkad.G = Function('tkad_G',{y,qd,y_dot,qd_dot}, {G});
    tkad.G_dot = Function('tkad_G_dot',{y,qd,y_dot,qd_dot}, {G_dot});
    tkad.g = Function('tkad_small_g',{y,qd,y_dot,qd_dot},{g});
    tkad.K = Function('tkad_K',{y,qd,y_dot,qd_dot}, {K});
    tkad.k = Function('tkad_small_k',{y,qd,y_dot,qd_dot},{k});
    tkad.IK_pos = Function('tkad_IK_pos', {qd}, {ik_y});
    tkad.IK_vel = Function('tkad_IK_vel', {qd,qd_dot}, {ik_y_dot});
    tkad.IK_acc = Function('tkad_IK_acc', {qd,qd_dot,qd_ddot},{ik_y_ddot});
    tkad.jacobian = Function('tkad_jacobian', {y,qd}, {G, K});
    tkad.bias = Function('tkad_bias', {y,qd,y_dot,qd_dot}, {g,k});

