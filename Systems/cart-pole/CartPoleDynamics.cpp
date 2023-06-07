#include "CartPoleDynamics.hpp"
#include <Utilities/pretty_print.h>

CartPoleDynamics::CartPoleDynamics():
    _lcm_cart("udpm://239.255.76.67:7667?ttl=255")
{
    _dim = 4;
    _ini_state = DVec<double>(_dim);
    _ini_state.setZero();

    _ini_state[1] = 1.*M_PI/180.;
    //_ini_state[1] = 10.*M_PI/180.;
    //_ini_state[3] = 5.*M_PI/180.;
    _curr_state = _ini_state;

    printf("[CartPoleDynamics] Constructed\n");
}

void CartPoleDynamics::step(const DVec<double> & input){
    _curr_state.head(2) += _curr_state.tail(2)*dt;
    double F(input[0]);
    double theta(_curr_state[1]);
    double d_theta(_curr_state[3]);

    double dd_x = 
      (F + m2*g*cos(theta)*sin(theta) - m2*d_theta*d_theta*l*sin(theta))/
      ((m1+m2)-m2*cos(theta)*cos(theta));
    double dd_theta = 
      (F + (m1+m2)*g*tan(theta) - m2*d_theta*d_theta*l*sin(theta))/
      ((m1+m2)*l/cos(theta) - m2*l*cos(theta));


    _curr_state[2] += dd_x*dt; // first link x
    _curr_state[3] += dd_theta*dt;

    while( theta>M_PI || theta < -M_PI ){
      if(theta > M_PI){
        theta -= 2*M_PI;
      }
      else {
        theta += 2*M_PI;
      }
    }
    ++_step;
    //pretty_print(_curr_state, std::cout, "curr state");
}

void CartPoleDynamics::_UpdateKinematics(const DVec<double> & state){
    _kin_msg.link1_pos[0] = state[0];
    _kin_msg.link1_pos[1] = height;
    _kin_msg.link1_pos[2] = 0.;

    double theta = state[1];
    _kin_msg.link2_pos[0] = state[0]-l*sin(theta);
    _kin_msg.link2_pos[1] = _kin_msg.link1_pos[1]+l*cos(theta);
    _kin_msg.link2_pos[2] = theta;
}

void CartPoleDynamics::send_message(){
    _UpdateKinematics(_curr_state);
    _lcm_cart.publish("cart_pole", &_kin_msg);
}

void CartPoleDynamics::give_disturbance(const DVec<double> & vel_disturbance){
    _curr_state[2] += vel_disturbance[0];
    _curr_state[3] += vel_disturbance[1];
}
