#ifndef VSMC_CORE_INTEGRATE_HPP
#define VSMC_CORE_INTEGRATE_HPP

#include <vsmc/internal/config.hpp>

#include <vsmc/utility/integrate/importance_sampling.hpp>
#include <vsmc/utility/integrate/numeric_newton_cotes.hpp>

#include <vsmc/utility/integrate/numeric_seq.hpp>

#if VSMC_USE_CILK
#include <vsmc/utility/integrate/numeric_cilk.hpp>
#endif

#if VSMC_USE_OMP
#include <vsmc/utility/integrate/numeric_omp.hpp>
#endif

#if VSMC_USE_STD
#include <vsmc/utility/integrate/numeric_std.hpp>
#endif

#if VSMC_USE_TBB
#include <vsmc/utility/integrate/numeric_tbb.hpp>
#endif

#endif // VSMC_CORE_INTEGRATE_HPP
