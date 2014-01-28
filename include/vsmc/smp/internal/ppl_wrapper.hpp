#ifndef VSMC_SMP_INTERNAL_PPL_WRAPPER_HPP
#define VSMC_SMP_INTERNAL_PPL_WRAPPER_HPP

#include <ppl.h>

namespace vsmc {
#if _MSC_VER >= 1700
namespace ppl = ::concurrency;
#else
namespace ppl = ::Concurrency;
#endif
} // namespace vsmc

#endif // VSMC_SMP_INTERNAL_PPL_WRAPPER_HPP
