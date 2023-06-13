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

f = Function('kikd_gen',{y,qd,y_dot,qd_dot},{Ki,Kd,Ki_dot,Kd_dot});

opts = struct('cpp', true, 'with_header', true);
gen_code(f, 'kikd_gen', opts);
