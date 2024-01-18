#include "gtest/gtest.h"

#include <random>
#include "testHelpers.hpp"
#include "grbda/Dynamics/ClusterJoints/ClusterJointTypes.h"

using namespace grbda;

class fourBarLoopConstraintTestHelper
{
private:
    using SX = casadi::SX;
    using FourBar = LoopConstraint::FourBar<double>;
    using CasFcn = casadi::Function;
    using FourBarSX = LoopConstraint::FourBar<SX>;

    std::shared_ptr<FourBarSX> four_bar_sym_;

public:
    std::shared_ptr<FourBar> four_bar_num;

    struct
    {
        CasFcn K;
        CasFcn k;
    } casadi_fcns;

public:
    fourBarLoopConstraintTestHelper()
    {
        // Find link lengths that satisfy the Grashof condition
        Eigen::Vector<double, 4> link_lengths;
        bool grashof_satisfied = false;
        while (!grashof_satisfied)
        {
            double scale = 2.0;
            link_lengths = Eigen::Vector<double, 4>::Constant(scale) +
                           scale * Eigen::Vector<double, 4>::Random();
            grashof_satisfied = checkGrashofCondition(link_lengths);
        }

        // Assign the independent coordinate
        // NOTE: The fixed angle b/t joints of link 1 and link 3 cannot be the independent coord
        Eigen::Index min_length_idx;
        double min_length = link_lengths.minCoeff(&min_length_idx);
        int independent_coordinate = min_length_idx != 3 ? min_length_idx : 0;

        // Assign the differenent paths from the parent joint of the cluster to the constraint joint
        std::vector<SX> path1_lengths_sym, path2_lengths_sym;
        std::vector<double> path1_lengths, path2_lengths;
        for (int i = 0; i < 3; i++)
        {
            if (randomBool())
            {
                path1_lengths_sym.push_back(link_lengths[i]);
                path1_lengths.push_back(link_lengths[i]);
            }
            else
            {
                path2_lengths_sym.push_back(link_lengths[i]);
                path2_lengths.push_back(link_lengths[i]);
            }
        }

        // Offset between fixed locations of joint 1 and joint 3
        double offset_angle = random<double>();
        Vec2<SX> offset_sym{link_lengths[3] * std::cos(offset_angle),
                            link_lengths[3] * std::sin(offset_angle)};
        Vec2<double> offset{link_lengths[3] * std::cos(offset_angle),
                            link_lengths[3] * std::sin(offset_angle)};

        // Create Loop Constraints
        four_bar_sym_ = std::make_shared<FourBarSX>(path1_lengths_sym, path2_lengths_sym,
                                                    offset_sym, independent_coordinate);
        four_bar_num = std::make_shared<FourBar>(path1_lengths, path2_lengths,
                                                 offset, independent_coordinate);

        // Create casadi functions
        four_bar_sym_->createRandomStateHelpers();
        createImplictConstraintFcns();
    }

    std::vector<casadi::DM> randomJointState() const
    {
        // Find a feasible joint position
        casadi::DMDict arg;
        arg["x0"] = random<casadi::DM>(four_bar_sym_->numSpanningPos());
        casadi::DM q_dm = four_bar_sym_->random_state_helpers_.phi_root_finder(arg).at("x");
        DVec<double> q(four_bar_sym_->numSpanningPos());
        casadi::copy(q_dm, q);
        JointCoordinate<double> joint_pos(q, false);

        // Compute the explicit constraint jacobians
        four_bar_num->updateJacobians(joint_pos);
        DMat<double> G = four_bar_num->G();

        // Compute a valid joint velocity
        DVec<double> v = G * DVec<double>::Random(four_bar_num->numIndependentVel());
        casadi::DM v_dm = casadi::DM::zeros(four_bar_sym_->numSpanningVel(), 1);
        casadi::copy(v, v_dm);
        JointCoordinate<double> joint_vel(v, false);

        // Compute the constraint biases
        four_bar_num->updateBiases(JointState<double>(joint_pos, joint_vel));
        DVec<double> g = four_bar_num->g();

        // Compute a valid joint acceleration
        DVec<double> vd = G * DVec<double>::Random(four_bar_num->numIndependentVel()) + g;
        casadi::DM vd_dm = casadi::DM::zeros(four_bar_sym_->numSpanningVel(), 1);
        casadi::copy(vd, vd_dm);

        return std::vector<casadi::DM>{q_dm, v_dm, vd_dm};
    }

private:
    void createImplictConstraintFcns()
    {
        // Define symbolic variables
        SX cs_q_sym = SX::sym("q", four_bar_sym_->numSpanningPos(), 1);
        DVec<SX> q_sym(four_bar_sym_->numSpanningPos());
        casadi::copy(cs_q_sym, q_sym);

        SX cs_v_sym = SX::sym("v", four_bar_sym_->numSpanningVel(), 1);
        DVec<SX> v_sym(four_bar_sym_->numSpanningVel());
        casadi::copy(cs_v_sym, v_sym);

        // Set state and update kinematics
        JointCoordinate<SX> joint_pos(q_sym, false);

        // Compute constraint violation
        DVec<SX> phi = four_bar_sym_->phi(joint_pos);
        SX cs_phi = casadi::SX(casadi::Sparsity::dense(phi.size(), 1));
        casadi::copy(phi, cs_phi);

        // Compute constraint jacobian
        SX cs_K = jacobian(cs_phi, cs_q_sym);

        // Compute constraint bias
        SX cs_Kdot = SX(cs_K.size1(), cs_K.size2());
        for (size_t i = 0; i < cs_K.size2(); i++)
        {
            cs_Kdot(casadi::Slice(), i) = jtimes(cs_K(casadi::Slice(), i), cs_q_sym, cs_v_sym);
        }
        SX cs_k = -casadi::SX::mtimes(cs_Kdot, cs_v_sym);

        // Create casadi functions
        {
            std::vector<SX> args{cs_q_sym};
            std::vector<SX> res{cs_K};
            casadi_fcns.K = casadi::Function("K", args, res);
        }

        {
            std::vector<SX> args{cs_q_sym, cs_v_sym};
            std::vector<SX> res{cs_k};
            casadi_fcns.k = casadi::Function("k", args, res);
        }
    }

    bool checkGrashofCondition(const Eigen::Vector<double, 4> &link_lengths) const
    {
        Eigen::Index max_length_idx;
        double max_length = link_lengths.maxCoeff(&max_length_idx);

        Eigen::Index min_length_idx;
        double min_length = link_lengths.minCoeff(&min_length_idx);

        std::vector<Eigen::Index> remaining_indices{0, 1, 2, 3};
        remaining_indices.erase(std::remove(remaining_indices.begin(), remaining_indices.end(), max_length_idx), remaining_indices.end());
        remaining_indices.erase(std::remove(remaining_indices.begin(), remaining_indices.end(), min_length_idx), remaining_indices.end());

        double middle_lengths_sum = link_lengths(remaining_indices[0]) +
                                    link_lengths(remaining_indices[1]);
        return min_length + max_length <= middle_lengths_sum;
    }
};

GTEST_TEST(LoopConstraint, fourBar)
{
    // This test:
    // 1) Verifies that the method of generating random joint states results in feasible states
    // that satisfy the implicit loop constraints
    // 2) Verifies that the analytical method of computing the implicit loop constraint jacobian
    // and bias returns the same result as the autodiff method
    // 3) Verifies that the implicit and explicit constraint jacobians and biases are consistent

    for (int i = 0; i < 25; i++)
    {
        fourBarLoopConstraintTestHelper helper;
        std::shared_ptr<LoopConstraint::FourBar<double>> four_bar_num = helper.four_bar_num;

        std::vector<casadi::DM> q_v_vd_dm = helper.randomJointState();
        casadi::DM q_dm = q_v_vd_dm[0];
        casadi::DM v_dm = q_v_vd_dm[1];
        casadi::DM vd_dm = q_v_vd_dm[2];
        std::vector<casadi::DM> state_dm{q_dm, v_dm};

        DVec<double> q(3);
        casadi::copy(q_dm, q);

        DVec<double> v(3);
        casadi::copy(v_dm, v);

        DVec<double> vd(3);
        casadi::copy(vd_dm, vd);

        JointCoordinate<double> joint_pos(q, false);
        JointCoordinate<double> joint_vel(v, false);
        JointState<double> joint_state(joint_pos, joint_vel);

        DVec<double> phi = four_bar_num->phi(joint_pos);
        four_bar_num->updateJacobians(joint_pos);
        DMat<double> K = four_bar_num->K();
        four_bar_num->updateBiases(joint_state);
        DVec<double> k = four_bar_num->k();

        // Verify that the constraints are satisfied
        GTEST_ASSERT_LE(phi.norm(), 1e-8) << "phi:\n"
                                          << phi;

        GTEST_ASSERT_LE((K * v).norm(), 1e-8) << "K*v:\n"
                                              << K * v;

        GTEST_ASSERT_LE((K * vd - k).norm(), 1e-8) << "K*vd + k:\n"
                                                   << K * vd + k;

        // Compare analytical vs. autodiff implicit constraints
        DMat<double> K_ad(2, 3);
        casadi::copy(helper.casadi_fcns.K(q_dm)[0], K_ad);
        GTEST_ASSERT_LE((K - K_ad).norm(), 1e-10) << "K:\n"
                                                  << K << "\nK_ad:\n"
                                                  << K_ad;

        DVec<double> k_ad(2);
        casadi::copy(helper.casadi_fcns.k(state_dm)[0], k_ad);
        GTEST_ASSERT_LE((k - k_ad).norm(), 1e-10) << "k:\n"
                                                  << k << "\nk_ad:\n"
                                                  << k_ad;

        // Verify agreement between implicit and explicit jacobians and biases
        DMat<double> G = four_bar_num->G();
        GTEST_ASSERT_LE((K * G).norm(), 1e-10) << "K*G:\n"
                                               << K * G;

        DVec<double> g = four_bar_num->g();
        GTEST_ASSERT_LE((K * g - k).norm(), 1e-10) << "K*g:\n"
                                                   << K * g << "\nk:\n"
                                                   << k;
    }
}

// TODO(@MatthewChignoli): How to test this?
GTEST_TEST(LoopConstraint, GenericImplicit)
{
    using SX = casadi::SX;

    std::function<DVec<SX>(const JointCoordinate<SX> &)> phi_fcn =
        [](const JointCoordinate<SX> &joint_pos)
    {
        DVec<SX> phi(3);
        phi(0) = joint_pos[0] + joint_pos[1] + joint_pos[2];
        phi(1) = joint_pos[0] * joint_pos[1] + joint_pos[1] * joint_pos[2];
        phi(2) = joint_pos[0] * joint_pos[1] * joint_pos[2];
        return phi;
    };

    LoopConstraint::GenericImplicit<double> generic_implicit(3, phi_fcn);

    DVec<double> q = DVec<double>::Random(3);
    DVec<double> v = DVec<double>::Random(3);
    JointCoordinate<double> joint_pos = JointCoordinate<double>(q, true);
    JointCoordinate<double> joint_vel = JointCoordinate<double>(v, true);
    JointState<double> joint_state(joint_pos, joint_vel);

    generic_implicit.updateJacobians(joint_pos);
    generic_implicit.updateBiases(joint_state);

    std::cout << "q:\n"
              << q << std::endl;
    std::cout << "phi:\n"
              << generic_implicit.phi(joint_pos) << std::endl;
    std::cout << "K:\n"
              << generic_implicit.K() << std::endl;
    std::cout << "k:\n"
              << generic_implicit.k() << std::endl;
}
