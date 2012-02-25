#ifndef V_SMC_HPP
#define V_SMC_HPP

#include <vSMC/core/sampler.hpp>
#include <vSMC/helper/sequential.hpp>
#include <vSMC/helper/parallel_tbb.hpp>

#endif // V_SMC_HPP

/*************************************************************************//**
 * \page pf_seq Sequential implementation of particle filter example
 *
 * This example is implemented with sequential code. Most of the code is no
 * more complex than those found in SMCTC. This is only a proof of concept that
 * vSMC can also be used in a similar fashion as in SMCTC. But for sequential
 * implementation, SMCTC is much more stable and matured. For a vectorized
 * implementation, which really takes the advantage of vSMC, see \ref pf_eigen.
 *
 * \include pf/pf_seq.cpp
 *
 * \page pf_eigen Vectorized implementation of particle filter example
 *
 * This example is implemented with the Eigen library. Its syntax is quite
 * elegent and in most case its use shall be self-explained by the code. It
 * also use the vDist library for vectorized computation of pdf, kernel,
 * generating random variates, etc. For a sequential implementation without any
 * use of SIMD see \ref pf_seq.
 *
 * \include pf/pf_eigen.cpp
 ****************************************************************************/
