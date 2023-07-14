#ifndef CASADIGEN_H
#define CASADIGEN_H

#define int_T long long int

#include "cppTypes.h"
#include <eigen3/Eigen/StdVector>

#include "rev_w_rotor_2dof_FD.h"
#include "rev_w_rotor_2dof_FD_ref_inertia.h"
#include "rev_w_rotor_2dof_FD_ref_inertia_diag.h"
#include "rev_w_rotor_4dof_FD.h"
#include "rev_w_rotor_4dof_FD_ref_inertia.h"
#include "rev_w_rotor_4dof_FD_ref_inertia_diag.h"
#include "rev_w_rotor_2dof_ID.h"
#include "rev_w_rotor_2dof_ID_ref_inertia.h"
#include "rev_w_rotor_2dof_ID_ref_inertia_diag.h"
#include "rev_w_rotor_4dof_ID.h"
#include "rev_w_rotor_4dof_ID_ref_inertia.h"
#include "rev_w_rotor_4dof_ID_ref_inertia_diag.h"

#include "rev_pair_w_rotor_2dof_FD.h"
#include "rev_pair_w_rotor_2dof_FD_ref_inertia.h"
#include "rev_pair_w_rotor_2dof_FD_ref_inertia_diag.h"
#include "rev_pair_w_rotor_4dof_FD.h"
#include "rev_pair_w_rotor_4dof_FD_ref_inertia.h"
#include "rev_pair_w_rotor_4dof_FD_ref_inertia_diag.h"
#include "rev_pair_w_rotor_2dof_ID.h"
#include "rev_pair_w_rotor_2dof_ID_ref_inertia.h"
#include "rev_pair_w_rotor_2dof_ID_ref_inertia_diag.h"
#include "rev_pair_w_rotor_4dof_ID.h"
#include "rev_pair_w_rotor_4dof_ID_ref_inertia.h"
#include "rev_pair_w_rotor_4dof_ID_ref_inertia_diag.h"

#include "thd_phi.h"
#include "thd_kikd.h"
#include "thd_J_dy_2_dqd.h"
#include "thd_g.h"
#include "thd_k.h"
#include "thd_IK_pos.h"
#include "thd_IK_vel.h"
#include "thd_IK_acc.h"

#include "tkad_phi.h"
#include "tkad_kikd.h"
#include "tkad_J_dy_2_dqd.h"
#include "tkad_g.h"
#include "tkad_k.h"
#include "tkad_IK_pos.h"
#include "tkad_IK_vel.h"
#include "tkad_IK_acc.h"

namespace grbda
{

  /*
    @brief: Get the numerical evaluation of a CasadiGen function and the output sparsity pattern
    @params:
            arg: T ptr to an array of pointers whose element points to an input variable
            res: T ptr to an array of pointers whose element points to an output variable
            max_sz_res: maximum size of output variables
  */
  template <typename T>
  void casadi_interface(std::vector<T *> ARG, std::vector<T *> RES, int max_sz_res,
                        int f(const T **, T **, int_T *, T *, int),
                        const int_T *f_sparse_out(int_T),
                        int f_work(int_T *, int_T *, int_T *, int_T *));

  template <typename T>
  void casadi_interface(
      std::vector<DVec<typename T::Scalar>> &arg, std::vector<Eigen::MatrixBase<T> *> &res,
      int f(const typename T::Scalar **, typename T::Scalar **, int_T *, typename T::Scalar *, int),
      const int_T *f_sparse_out(int_T),
      int f_work(int_T *, int_T *, int_T *, int_T *))
  {
    std::vector<typename T::Scalar *> ARG;
    for (size_t idx_arg = 0; idx_arg < arg.size(); idx_arg++)
    {
      ARG.push_back(arg[idx_arg].data());
    }

    std::vector<typename T::Scalar *> RES;
    int max_res_size = 0;
    for (size_t idx_res = 0; idx_res < res.size(); idx_res++)
    {
      Eigen::MatrixBase<T> *res_ptr = res[idx_res];
      res_ptr->setZero();

      RES.push_back(res_ptr->derived().data());

      if (res_ptr->size() > max_res_size)
      {
        max_res_size = res_ptr->size();
      }
    }

    casadi_interface(ARG, RES, max_res_size, f, f_sparse_out, f_work);
  }

  template <typename T>
  void casadi_interface(
      std::vector<DVec<typename T::Scalar>> &arg, Eigen::MatrixBase<T> &res,
      int f(const typename T::Scalar **, typename T::Scalar **, int_T *, typename T::Scalar *, int),
      const int_T *f_sparse_out(int_T),
      int f_work(int_T *, int_T *, int_T *, int_T *))
  {
    std::vector<Eigen::MatrixBase<T> *> res_vec = {&res};
    casadi_interface(arg, res_vec, f, f_sparse_out, f_work);
  }

} // namespace grbda

#endif // CASADIGEN_H
