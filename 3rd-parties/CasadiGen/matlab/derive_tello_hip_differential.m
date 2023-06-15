function thd = derive_tello_hip_differential_fn()
    import casadi.*

    y = SX.sym('y',2);
    qd = SX.sym('qd',2);
    y_dot = SX.sym('y',2);
    qd_dot = SX.sym('qd_dot',2);

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

    Ki = jacobian(phi,y); % (2,2)
    Kd = jacobian(phi,qd); % (2,2)

    Ki_dot = jtimes(Ki,y,y_dot) + jtimes(Ki,qd,qd_dot); % (4,1)
    Kd_dot = jtimes(Kd,y,y_dot) + jtimes(Kd,qd,qd_dot); % (4,1)
    Ki_dot = reshape(Ki_dot,2,2); % (2,2)
    Kd_dot = reshape(Kd_dot,2,2); % (2,2)

    J_dy_2_dqd = -Kd\Ki;

    % Get k = -K_dot * q_dot

    K = SX.zeros(2,4);
    K(1:end,3:end) = eye(2);
    K(1:end,1:2) = -J_dy_2_dqd;

    q = [y(1); y(2); qd(1); qd(2)];
    q_dot = [y_dot(1); y_dot(2); qd_dot(1); qd_dot(2)]
    K_qdot = K * q_dot;
    B = jtimes(K,q,q_dot);
    k = -jtimes(K_qdot,q,q_dot);

    % Get g = G_dot * y_dot;

    G = SX.zeros(4,2);
    G(1:2,1:2) = eye(2);
    G(3:end,1:2) = J_dy_2_dqd;

    G_y_dot = G * y_dot;
    g = jtimes(G_y_dot,qd,qd_dot) + jtimes(G_y_dot,y,y_dot);

    % Casadi functions
    thd.phi = Function('thd_phi', {y,qd}, {phi});
    thd.kikd = Function('thd_kikd',{y,qd,y_dot,qd_dot},{Ki,Kd,Ki_dot,Kd_dot});
    thd.J_dy_2_dqd = Function('thd_J_dy_2_dqd', {y,qd}, {J_dy_2_dqd});
    thd.k = Function('thd_k',{y,qd,y_dot,qd_dot},{k});
    thd.g = Function('thd_g',{y,qd,y_dot,qd_dot},{g});

