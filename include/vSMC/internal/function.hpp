#ifndef V_SMC_INTERNAL_FUNCTION_HPP
#define V_SMC_INTERNAL_FUNCTION_HPP

#include <vSMC/internal/config.hpp>

#ifdef V_SMC_USE_STD_FUNCTION

#include <functional>
namespace vSMC {
using std::function;
}

#else // V_SMC_USE_STD_FUNCTION

#include <boost/function.hpp>
namespace vSMC {
using boost::function;
}

#endif // V_SMC_USE_STD_FUNCTION

#endif // V_SMC_INTERNAL_FUNCTION_HPP
