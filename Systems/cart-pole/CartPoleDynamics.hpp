#ifndef CART_POLE_DYNAMICS_HPP
#define CART_POLE_DYNAMICS_HPP

#include <cart_pole_lcmt.hpp>
#include <lcm/lcm-cpp.hpp>
#include <cppTypes.h>

class CartPoleDynamics{
    public:
        CartPoleDynamics();
        ~CartPoleDynamics(){}
        void step(const DVec<double> & input);

        void send_message();
        void give_disturbance(const DVec<double> & vel_disturbance);
        bool reset(){
            _curr_state = _ini_state;
            _step = 0;
            return true;
        }
        int getStep(){ return _step; }
        const DVec<double> &  getState(){ return _curr_state; }

    protected:
        DVec<double> _ini_state;
        DVec<double> _curr_state;
        int _step = 0;
        int _dim = 0;

        double dt = 0.001;
        constexpr static double g = 9.81;
        constexpr static double m1 = 1.0;
        constexpr static double m2 = 0.1;
        constexpr static double height = 0.7; // z of link 1
        constexpr static double l = 1.; // CoM of link 2

        cart_pole_lcmt _kin_msg;
        lcm::LCM _lcm_cart;

        void _UpdateKinematics(const DVec<double> & state);
        void _SendKinematicsData();
};
#endif
