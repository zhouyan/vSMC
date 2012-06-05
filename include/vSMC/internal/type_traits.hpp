#ifndef V_SMC_INTERNAL_TYPE_TRATIS
#define V_SMC_INTERNAL_TYPE_TRATIS

#include <vSMC/internal/config.hpp>

#ifdef V_SMC_USE_STD_TYPE_TRAITS

#include <type_traits>
namespace vSMC { namespace internal {
using std::true_type;
using std::false_type;
using std::is_base_of;
using std::is_pointer;
using std::remove_pointer;
} }

#else // V_SMC_USE_STD_TYPE_TRAITS

#include <boost/type_traits.hpp>
namespace vSMC { namespace internal {
using boost::true_type;
using boost::false_type;
using boost::is_base_of;
using boost::is_pointer;
using boost::remove_pointer;
} }

#endif // V_SMC_USE_STD_TYPE_TRAITS

#endif // V_SMC_INTERNAL_TYPE_TRATIS
