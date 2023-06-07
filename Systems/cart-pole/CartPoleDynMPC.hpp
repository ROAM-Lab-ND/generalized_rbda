#ifndef __CARTPOLEDYNMPC_HPP__
#define __CARTPOLEDYNMPC_HPP__

#include <cart_pole_lcmt.hpp>
#include <lcm/lcm-cpp.hpp>
#include <cppTypes.h>
// Casadi setting
#include <casadi/casadi_c.h>
#include <casadi/casadi.hpp>
#include <Configuration.h>

using namespace std;


class CartPoleDyn {
    private:
        DVec<double> _init_state;
        DVec<double> _init_state2;
        DVec<double> _curr_state;
        DVec<double> _curr_state2;
        int _step = 0;
        int _dim = 0;
        double dt = 0.001;
        double input;
        double input2;
        constexpr static double g = 9.806;
        constexpr static double m1 = 1.0;
        constexpr static double m2 = 0.1;
        constexpr static double l = 1.;
        int mpc_step = 0;
        double mpc_time = 0;
        // LQR Parameters
        DMat<double> A = DMat<double>::Zero(4,4);
        DMat<double> B = DMat<double>::Zero(4,1);
        DMat<double> Q = DMat<double>::Zero(4,4);
        DMat<double> R = DMat<double>::Zero(1,1);
        DMat<double> S = DMat<double>::Zero(4,4);
        DMat<double> K = DMat<double>::Zero(1,1);
        cart_pole_lcmt _kin_msg;
        lcm::LCM _lcm_cart;

        //int ret = casadi_c_push_file(THIS_COM"systems/cart-pole/m_cartpoleMPC_ipopt.casadi"); // ipopt
        int ret = casadi_c_push_file(THIS_COM"systems/cart-pole/m_cartpoleMPC_knitro.casadi"); // knitro
        int id = casadi_c_id("M");

    public:
        CartPoleDyn();
        void lqr();
        void step();
        void send_message();
        void _UpdateKinematics(DVec<double>& state, DVec<double>& state2);
        void run_mpc();
        ~CartPoleDyn() {}
};

#endif
