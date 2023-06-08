% second actuation group - hip differential
% {hip roll (qd1), hip pitch (qd2)} - {actuator2 (y1), actuator3 (y2)}

import casadi.*

qd = SX.sym('qd',2);
qd_dot = SX.sym('qd_dot',2);
qd_ddot = SX.sym('qd_ddot',2);
y = SX.sym('y',2);

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

y(1) = atan2(-b1*d1 + a1*c1, b1*c1+a1*d1);
y(2) = atan2(-b2*d2 + a2*c2, b2*c2+a2*d2);

% Inverse kinematics from qd to y

f1 = Function('IK_dependent_state_to_y',{qd},{y});

% Differentiate y to get y_dot

y_dot = jtimes(y, qd, qd_dot);

f2 = Function('IK_dependent_state_to_y_dot',{qd,qd_dot},{y_dot});

% Differentiate y_dot to get y_ddot

dy_dqd = jacobian(y,qd);
%y_ddot = dy_dqd * qd_ddot + ...
%         (jtimes(dy_dqd,qd,qd_dot) + ...
%          jtimes(dy_dqd,qd_dot,qd_ddot)) * qd_dot;
y_ddot = dy_dqd * qd_ddot + jtimes(y_dot,qd,qd_dot);

f3 = Function('IK_dependent_state_to_y_ddot',{qd,qd_dot,qd_ddot},{y_ddot});

% to test:
% f1([1, 2])
% f2([1; 2],[3; 4])
% f3([1; 2],[3; 4],[5; 6])

% code gen:
% opts = struct('cpp', true, 'with_header', true);
% f1.generate('IK_dependent_state_to_y.cpp',opts);
% f2.generate('IK_dependent_state_to_y_dot.cpp',opts);
% f3.generate('IK_dependent_state_to_y_ddot.cpp',opts);
