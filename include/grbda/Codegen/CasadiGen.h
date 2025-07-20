#ifndef GRBDA_CASADIGEN_H
#define GRBDA_CASADIGEN_H

#include <casadi/casadi.hpp>
#include <eigen3/Eigen/StdVector>
#include "grbda/Utils/cppTypes.h"

#define grbda_int_T long long int

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

namespace grbda
{

  template <typename Scalar>
  struct CasadiHelperFunctions
  {
    typedef int (*casadi_fn)(const Scalar **, Scalar **, long long int *, Scalar *, int);
    typedef const long long int *(*sparsity_out_fn)(long long int);
    typedef int (*work_fn)(long long int *, long long int *, long long int *, long long int *);

    CasadiHelperFunctions() {}
    CasadiHelperFunctions(casadi_fn main, sparsity_out_fn sparsity, work_fn work)
        : main_(main), sparsity_(sparsity), work_(work) {}

    casadi_fn main_;
    sparsity_out_fn sparsity_;
    work_fn work_;
  };

  /*
    @brief: Get the numerical evaluation of a CasadiGen function and the output sparsity pattern
    @params:
            arg: T ptr to an array of pointers whose element points to an input variable
            res: T ptr to an array of pointers whose element points to an output variable
            max_sz_res: maximum size of output variables
  */
  template <typename T>
  void casadi_interface(std::vector<T *> ARG, std::vector<T *> RES, int max_sz_res,
                        int f(const T **, T **, grbda_int_T *, T *, int),
                        const grbda_int_T *f_sparse_out(grbda_int_T),
                        int f_work(grbda_int_T *, grbda_int_T *, grbda_int_T *, grbda_int_T *));

  template <typename T>
  void casadi_interface(
      std::vector<DVec<typename T::Scalar>> &arg, std::vector<Eigen::MatrixBase<T> *> &res,
      int f(const typename T::Scalar **, typename T::Scalar **, grbda_int_T *, typename T::Scalar *, int),
      const grbda_int_T *f_sparse_out(grbda_int_T),
      int f_work(grbda_int_T *, grbda_int_T *, grbda_int_T *, grbda_int_T *))
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
      std::vector<DVec<typename T::Scalar>> &arg, std::vector<Eigen::MatrixBase<T> *> &res,
      const CasadiHelperFunctions<typename T::Scalar>& helpers)
  {
    casadi_interface(arg, res, helpers.main_, helpers.sparsity_, helpers.work_);
  }

  template <typename T>
  void casadi_interface(
      std::vector<DVec<typename T::Scalar>> &arg, Eigen::MatrixBase<T> &res,
      int f(const typename T::Scalar **, typename T::Scalar **, grbda_int_T *, typename T::Scalar *, int),
      const grbda_int_T *f_sparse_out(grbda_int_T),
      int f_work(grbda_int_T *, grbda_int_T *, grbda_int_T *, grbda_int_T *))
  {
    std::vector<Eigen::MatrixBase<T> *> res_vec = {&res};
    casadi_interface(arg, res_vec, f, f_sparse_out, f_work);
  }

  template <typename T>
  void casadi_interface(
      std::vector<DVec<typename T::Scalar>> &arg, Eigen::MatrixBase<T> &res,
      const CasadiHelperFunctions<typename T::Scalar>& helpers)
  {
    casadi_interface(arg, res, helpers.main_, helpers.sparsity_, helpers.work_);
  }

} // namespace grbda

#endif // GRBDA_CASADIGEN_H
