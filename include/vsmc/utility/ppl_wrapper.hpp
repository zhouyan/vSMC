#ifndef VSMC_UTILITY_PPL_WRAPPER_HPP
#define VSMC_UTILITY_PPL_WRAPPER_HPP

#include <ppl.h>

#if _MSC_VER >= 1700
namespace ppl = ::concurrency;
#else
namespace ppl = ::Concurrency;
#endif

#endif // VSMC_UTILITY_PPL_WRAPPER_HPP
