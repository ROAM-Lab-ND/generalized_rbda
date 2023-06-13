import casadi.*

y = SX.sym('y',2);
y_dot = SX.sym('y_dot',2);
qd = SX.sym('qd',2);
qd_dot = SX.sym('qd_dot',2);

J_q_hip = SX.zeros(2,2);
J_q_hip(1,1) = ...
    (49*sin(qd(1)))/5000 - (399*cos(qd(1)))/20000 - (7*cos(qd(1))*sin(y(1)))/625 + ...
    (7*cos(qd(1))*sin(qd(2)))/625 + (57*sin(qd(1))*sin(qd(2)))/2500 + ...
    (8*sin(y(1))*sin(qd(1))*sin(qd(2)))/625;
J_q_hip(1,2) = ...
    (8*cos(y(1))*sin(qd(2)))/625 - ...
    (57*cos(qd(1))*cos(qd(2)))/2500 + (7*cos(qd(2))*sin(qd(1)))/625 - ...
    (8*cos(qd(1))*cos(qd(2))*sin(y(1)))/625;
J_q_hip(2,1) = ...
    (399*cos(qd(1)))/20000 + (49*sin(qd(1)))/5000 + (7*cos(qd(1))*sin(y(2)))/625 - ...
    (7*cos(qd(1))*sin(qd(2)))/625 + (57*sin(qd(1))*sin(qd(2)))/2500 + ...
    (8*sin(y(2))*sin(qd(1))*sin(qd(2)))/625;
J_q_hip(2,2) = ...
    (8*cos(y(2))*sin(qd(2)))/625 - (57*cos(qd(1))*cos(qd(2)))/2500 - ...
    (7*cos(qd(2))*sin(qd(1)))/625 - (8*cos(qd(1))*cos(qd(2))*sin(y(2)))/625;
  
J_p_hip = SX.zeros(2,2);
J_p_hip(1,1) = ...
    (57*cos(y(1)))/2500 - (7*cos(y(1))*sin(qd(1)))/625 + (8*cos(qd(2))*sin(y(1)))/625 - ...
    (8*cos(y(1))*cos(qd(1))*sin(qd(2)))/625; ...
J_p_hip(2,2) = ...
    (57*cos(y(2)))/2500 + (7*cos(y(2))*sin(qd(1)))/625 + ...
    (8*cos(qd(2))*sin(y(2)))/625 - (8*cos(y(2))*cos(qd(1))*sin(qd(2)))/625;
  
J_dp_2_dq_hip = -J_q_hip\J_p_hip;

% Get k = -K_dot * q_dot

K = SX.zeros(2,4);
K(1:end,3:end) = eye(2);
K(1:end,1:2) = -J_dp_2_dq_hip;

q = [y(1); y(2); qd(1); qd(2)];
q_dot = [y_dot(1); y_dot(2); qd_dot(1); qd_dot(2)]
K_qdot = K * q_dot;
B = jtimes(K,q,q_dot);
k = -jtimes(K_qdot,q,q_dot);

f1 = Function('k_gen',{y,qd,y_dot,qd_dot},{k});

% Get g = G_dot * y_dot;

G = SX.zeros(4,2);
G(1:2,1:2) = eye(2);
G(3:end,1:2) = J_dp_2_dq_hip;

%G_qd_dot = G * qd_dot;
G_y_dot = G * y_dot;
g = jtimes(G_y_dot,qd,qd_dot) + jtimes(G_y_dot,y,y_dot);
f2 = Function('g_gen',{y,qd,y_dot,qd_dot},{g});

opts = struct('cpp', true, 'with_header', true);
gen_code(f1, 'k_gen', opts);
gen_code(f2, 'g_gen', opts);
