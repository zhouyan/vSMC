#include <vSMC/internal/config.hpp>

#ifdef V_SMC_USE_STD_FUNCTION

#include <functional>
namespace vSMC { namespace internal {
using std::function;
} }

#else // V_SMC_USE_STD_FUNCTION

#ifdef V_SMC_USE_TR1_FUNCTION

#include <tr1/functional>
namespace vSMC { namespace internal {
using std::tr1::function;
} }

#else // V_SMC_USE_TR1_FUNCTION

#include <boost/function.hpp>
namespace vSMC { namespace internal {
using boost::function;
} }

#endif // V_SMC_USE_TR1_FUNCTION

#endif // V_SMC_USE_STD_FUNCTION
