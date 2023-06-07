#include "CartPoleDynLQR.hpp"
#include <utils/Utilities/LQR.hpp>

CartPoleDyn::CartPoleDyn()
    : _lcm_cart("udpm://239.255.76.67:7667?ttl=255") {

    _dim = 4;
    _init_state = DVec<double>(_dim); // [ x; theta; d_x; d_theta ]
    _init_state2 = DVec<double>(_dim); // [ z; theta; d_z; d_theta ]
    _init_state.setZero();
    _init_state2.setZero();
    _init_state[1] = M_PI+0.3; // Initialize the theta value (Upward: pi)
    _init_state2[1] = M_PI-0.3; // Initialize the theta value (Upward: pi)
    //_init_state[3] = -.1;
    //_init_state2[3] = -.1;
    _curr_state = _init_state;
    _curr_state2 = _init_state2;

    printf("[CartPoleDynLQR] Constructed\n");
}

void CartPoleDyn::lqr() {
    A << 0.0,   0.0,   1.0,   0.0,
         0.0,   0.0,   0.0,   1.0,
         0.0,   0.0,   0.0,   0.0,
         0.0,   0.0,   0.0,   0.0;
    A.block(2, 0, 2, 2) << 0.0, m2 * g / m1, 0.0, ((m1+ m2) * g) / (m1 * l);
    B <<  0.0,
          0.0,
          1 / m1,
          1 / (m1 * l);
    Q <<  1.0,   0.0,   0.0,   0.0,
          0.0,   1.0,   0.0,   0.0,
          0.0,   0.0,   1.0,   0.0,
          0.0,   0.0,   0.0,   1.0;
    R << 0.05;

    // LQR function solving Algebraic Riccati Eq. in this case
    LQR(A, B, Q, R, S);
    K = R.inverse() * B.transpose() * S;
}

void CartPoleDyn::step() {
    DVec<double> err = _curr_state;
    DVec<double> err2 = _curr_state2;

    err[1] -= M_PI; // Minimized error to be pi (Upward configuration)
    err[0] -= 0.01*_step;
    err2[1] -= M_PI; // Minimized error to be pi (Upward configuration)
    err2[0] -= 0.01*_step;

    input = -(K * err)[0];
    input2 = -(K * err2)[0];

    double Fx(input); // Input -Kx to the horizontal force
    double Fx2(input2); // Input -Kx to the horizontal force
    double theta(_curr_state[1]);
    double theta2(_curr_state2[1]);
    double d_theta(_curr_state[3]);
    double d_theta2(_curr_state2[3]);
    double dd_x =
      (Fx + (m2 * sin(theta) * ((l * d_theta * d_theta) + (g * cos(theta)))))
      / (m1 + (m2 * sin(theta) * sin(theta)));
    double dd_x2 =
      (Fx2 + (m2 * sin(theta2) * ((l * d_theta2 * d_theta2) + (g * cos(theta2)))))
      / (m1 + (m2 * sin(theta2) * sin(theta2)));
    double dd_theta =
      ((-Fx * cos(theta)) - (m2 * l * d_theta * d_theta * cos(theta)
      * sin(theta)) - ((m1 + m2) * g * sin(theta)))
      / (l * (m1 + (m2 * sin(theta) * sin(theta))));
    double dd_theta2 =
      ((-Fx2 * cos(theta2)) - (m2 * l * d_theta2 * d_theta2 * cos(theta2)
      * sin(theta2)) - ((m1 + m2) * g * sin(theta2)))
      / (l * (m1 + (m2 * sin(theta2) * sin(theta2))));

    _curr_state[2] += dd_x * dt;
    _curr_state2[2] += dd_x2 * dt;
    _curr_state[3] += dd_theta * dt;
    _curr_state2[3] += dd_theta2 * dt;
    _curr_state.head(2) += _curr_state.tail(2) * dt;
    _curr_state2.head(2) += _curr_state2.tail(2) * dt;

    while(theta > M_PI || theta < -M_PI) {
      if(theta > M_PI) {
        theta -= 2 * M_PI;
      }
      else {
        theta += 2 * M_PI;
      }
    }
    while(theta2 > M_PI || theta2 < -M_PI) {
      if(theta2 > M_PI) {
        theta2 -= 2 * M_PI;
      }
      else {
        theta2 += 2 * M_PI;
      }
    }
    ++_step;
}

void CartPoleDyn::send_message() {
    _UpdateKinematics(_curr_state, _curr_state2);
    _lcm_cart.publish("cart_pole", &_kin_msg);
}

void CartPoleDyn::_UpdateKinematics(DVec<double> & state, DVec<double> & state2) {
    double theta = state[1];
    double theta2 = state2[1];

    _kin_msg.link1_pos[0] = state[0]; // x
    _kin_msg.link1_pos[1] = state2[0]; // z
    _kin_msg.link1_pos[2] = 0.;

    _kin_msg.link2_pos[0] = state[0]+(l*sin(theta)); // x
    _kin_msg.link2_pos[1] = -l*cos(theta); // y
    _kin_msg.link2_pos[2] = theta;

    _kin_msg.link3_pos[0] = state2[0]+(l*sin(theta2)); // z
    _kin_msg.link3_pos[1] = -l*cos(theta2); // y
    _kin_msg.link3_pos[2] = theta2;
}
