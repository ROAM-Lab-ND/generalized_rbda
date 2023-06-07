/*
 *    This file is part of CasADi.
 *
 *    CasADi -- A symbolic framework for dynamic optimization.
 *    Copyright (C) 2010-2023 Joel Andersson, Joris Gillis, Moritz Diehl,
 *                            KU Leuven. All rights reserved.
 *    Copyright (C) 2011-2014 Greg Horn
 *
 *    CasADi is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    CasADi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with CasADi; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
#ifndef CASADI_CONFIG_H // NOLINT(build/header_guard)
#define CASADI_CONFIG_H // NOLINT(build/header_guard)

#define CASADI_MAJOR_VERSION 3
#define CASADI_MINOR_VERSION 6
#define CASADI_PATCH_VERSION 0
#define CASADI_IS_RELEASE 1
#define CASADI_VERSION_STRING "3.6.0"
#define CASADI_GIT_REVISION "b1e3f3e25daebf40a945864108080a119efc7c93"  // NOLINT(whitespace/line_length)
#define CASADI_GIT_DESCRIBE "3.3.0-1710.b1e3f3e25"  // NOLINT(whitespace/line_length)
#define CASADI_FEATURE_LIST "\n * dynamic-loading, Compile with support for dynamic loading of FMU libraries\n * sundials-interface, Interface to the ODE/DAE integrator suite SUNDIALS.\n * csparse-interface, Interface to the sparse direct linear solver CSparse.\n * superscs-interface, Interface to QP solver SUPERSCS.\n * osqp-interface, Interface to QP solver OSQP.\n * tinyxml-interface, Interface to the XML parser TinyXML.\n * qpoases-interface, Interface to the active-set QP solver qpOASES.\n * blocksqp-interface, Interface to the NLP solver blockSQP.\n * cplex-mockup-build, Use mockup CPLEX (BUILD_MOCKUPS_VERSION=v60) from downloaded source (BUILD_MOCKUPS_GIT_REPO=https://github.com/casadi/mockups.git).\n * snopt-mockup-build, Use mockup SNOPT (BUILD_MOCKUPS_VERSION=v60) from downloaded source (BUILD_MOCKUPS_GIT_REPO=https://github.com/casadi/mockups.git).\n * knitro-mockup-build, Use mockup KNITRO (BUILD_MOCKUPS_VERSION=v60) from downloaded source (BUILD_MOCKUPS_GIT_REPO=https://github.com/casadi/mockups.git).\n * gurobi-mockup-build, Use mockup GUROBI (BUILD_MOCKUPS_VERSION=v60) from downloaded source (BUILD_MOCKUPS_GIT_REPO=https://github.com/casadi/mockups.git).\n * hsl-mockup-build, Use mockup WORHP (BUILD_MOCKUPS_VERSION=v60) from downloaded source (BUILD_MOCKUPS_GIT_REPO=https://github.com/casadi/mockups.git).\n * highs-sourcebuild, Build HiGHS (BUILD_HIGHS_VERSION=v1.4.1) from downloaded source (BUILD_HIGHS_GIT_REPO=https://github.com/ERGO-Code/HiGHS).\n * osqp-sourcebuild, Build OSQP (BUILD_OSQP_VERSION=v0.6.2) from downloaded source (BUILD_OSQP_GIT_REPO=https://github.com/osqp/osqp.git).\n * superscs-sourcebuild, Build SuperSCS (BUILD_SUPERSCS_VERSION=4d2d1bd03ed4cf93e684a880b233760ce34ca69c) from downloaded source (BUILD_SUPERSCS_GIT_REPO=https://github.com/jgillis/scs.git).\n * bonmin-sourcebuild, Build BONMIN (BUILD_BONMIN_VERSION=releases/1.8.8) from downloaded source (BUILD_BONMIN_GIT_REPO=https://github.com/coin-or/Bonmin.git).\n * ipopt-sourcebuild, Build IPOPT (BUILD_IPOPT_VERSION=3.14.11.mod) from downloaded source (BUILD_IPOPT_GIT_REPO=https://github.com/jgillis/Ipopt-1.git).\n * cbc-sourcebuild, Build CBC (BUILD_CBC_VERSION=releases/2.10.6) from downloaded source.\n * clp-sourcebuild, Build CLP (BUILD_CLP_VERSION=releases/1.17.7) from downloaded source (BUILD_CLP_GIT_REPO=https://github.com/coin-or/Clp.git).\n * mumps-sourcebuild, Build MUMPS (BUILD_MUMPS_TP_VERSION=releases/3.0.2) from downloaded source (BUILD_MUMPS_TP_GIT_REPO=https://github.com/coin-or-tools/ThirdParty-Mumps.git).\n * metis-sourcebuild, Build METIS (BUILD_METIS_TP_VERSION=releases/2.0.0) from downloaded source.\n * hpipm-sourcebuild, Build HPIPM (BUILD_HPIPM_VERSION=0e0c9f4e0d4081dceafa9b37c396db50bce0e81a) from downloaded source (BUILD_HPIPM_GIT_REPO=https://github.com/jgillis/hpipm.git).\n * blasfeo-sourcebuild, Build BLASFEO (BUILD_BLASFEO_VERSION=edf92b396adddd9e548b9786f87ad290a0971329) from downloaded source (BUILD_BLASFEO_GIT_REPO=https://github.com/giaf/blasfeo.git).\n * cplex-interface, Interface to the QP solver CPLEX.\n * gurobi-interface, Interface to the (mixed-integer) QP solver GUROBI\n * knitro-interface, Interface to the NLP solver KNITRO.\n * snopt-interface, Interface to the NLP solver KNITRO.\n * lapack-interface, Interface to LAPACK.\n * mumps-interface, Interface to MUMPS.\n * coinutils-sourcebuild, Build COINUTILS (BUILD_COINUTILS_VERSION=releases/2.11.6) from downloaded source.\n * osi-sourcebuild, Build OSI (BUILD_OSI_VERSION=releases/0.108.7) from downloaded source.\n * clp-interface, Interface to the LP solver CLP.\n * cgl-sourcebuild, Build CGL (BUILD_CGL_VERSION=releases/0.60.4) from downloaded source.\n * cbc-interface, Interface to the LP solver CBC.\n * ipopt-interface, Interface to the NLP solver Ipopt.\n * bonmin-interface, Interface to the MINLP framework Bonmin.\n * highs-interface, Interface to the MILP / QP solver HiGHS.\n * ampl-interface, Interface to the AMPL solver library.\n"  // NOLINT(whitespace/line_length)
#define CASADI_BUILD_TYPE "Release"  // NOLINT(whitespace/line_length)
#define CASADI_COMPILER_ID "Clang"  // NOLINT(whitespace/line_length)
#define CASADI_COMPILER "/usr/local/miniconda/envs/compiler/bin/clang++"  // NOLINT(whitespace/line_length)
#define CASADI_COMPILER_FLAGS " -pthread -fPIC    -DUSE_CXX11 -DHAVE_MKSTEMPS -DCASADI_WITH_THREAD -DWITH_DEEPBIND -DCASADI_VERSION=31 -D_USE_MATH_DEFINES -D_SCL_SECURE_NO_WARNINGS -DWITH_DL -DWITH_FMU -DWITH_DEPRECATED_FEATURES"  // NOLINT(whitespace/line_length)
#define CASADI_MODULES "casadi;casadi_linsol_lapacklu;casadi_linsol_lapackqr;casadi_sundials_common;casadi_integrator_cvodes;casadi_integrator_idas;casadi_rootfinder_kinsol;casadi_nlpsol_ipopt;casadi_nlpsol_bonmin;casadi_conic_qpoases;casadi_nlpsol_knitro;casadi_conic_cplex;casadi_conic_clp;casadi_conic_cbc;casadi_linsol_csparse;casadi_linsol_csparsecholesky;casadi_conic_highs;casadi_linsol_ma27;casadi_linsol_mumps;casadi_conic_gurobi;casadi_nlpsol_snopt;casadi_xmlfile_tinyxml;casadi_nlpsol_blocksqp;casadi_conic_hpipm;casadi_conic_superscs;casadi_nlpsol_ampl;casadi_conic_osqp;casadi_conic_nlpsol;casadi_conic_qrqp;casadi_conic_ipqp;casadi_nlpsol_qrsqp;casadi_importer_shell;casadi_integrator_rk;casadi_integrator_collocation;casadi_interpolant_linear;casadi_interpolant_bspline;casadi_linsol_symbolicqr;casadi_linsol_qr;casadi_linsol_ldl;casadi_linsol_tridiag;casadi_linsol_lsqr;casadi_nlpsol_sqpmethod;casadi_nlpsol_feasiblesqpmethod;casadi_nlpsol_scpgen;casadi_rootfinder_newton;casadi_rootfinder_fast_newton;casadi_rootfinder_nlpsol"  // NOLINT(whitespace/line_length)
#define CASADI_PLUGINS "Linsol::lapacklu;Linsol::lapackqr;Integrator::cvodes;Integrator::idas;Rootfinder::kinsol;Nlpsol::ipopt;Nlpsol::bonmin;Conic::qpoases;Nlpsol::knitro;Conic::cplex;Conic::clp;Conic::cbc;Linsol::csparse;Linsol::csparsecholesky;Conic::highs;Linsol::ma27;Linsol::mumps;Conic::gurobi;Nlpsol::snopt;XmlFile::tinyxml;Nlpsol::blocksqp;Conic::hpipm;Conic::superscs;Nlpsol::ampl;Conic::osqp;Conic::nlpsol;Conic::qrqp;Conic::ipqp;Nlpsol::qrsqp;Importer::shell;Integrator::rk;Integrator::collocation;Interpolant::linear;Interpolant::bspline;Linsol::symbolicqr;Linsol::qr;Linsol::ldl;Linsol::tridiag;Linsol::lsqr;Nlpsol::sqpmethod;Nlpsol::feasiblesqpmethod;Nlpsol::scpgen;Rootfinder::newton;Rootfinder::fast_newton;Rootfinder::nlpsol"  // NOLINT(whitespace/line_length)
#define CASADI_INSTALL_PREFIX "/usr/local"  // NOLINT(whitespace/line_length)
#define CASADI_SHARED_LIBRARY_PREFIX "lib"  // NOLINT(whitespace/line_length)
#define CASADI_SHARED_LIBRARY_SUFFIX ".dylib"  // NOLINT(whitespace/line_length)
#define CASADI_OBJECT_FILE_SUFFIX ".o"  // NOLINT(whitespace/line_length)

#endif  // CASADI_CONFIG_H // NOLINT(build/header_guard)
