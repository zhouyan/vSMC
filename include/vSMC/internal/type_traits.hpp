#ifndef VSMC_INTERNAL_TYPE_TRATIS
#define VSMC_INTERNAL_TYPE_TRATIS

#include <vsmc/internal/config.hpp>

#ifdef VSMC_USE_STD_TYPE_TRAITS

#include <type_traits>
namespace vsmc { namespace internal {
using std::true_type;
using std::false_type;
using std::is_base_of;
using std::is_pointer;
using std::remove_pointer;
} }

#else // VSMC_USE_STD_TYPE_TRAITS

#include <boost/type_traits.hpp>
namespace vsmc { namespace internal {
using boost::true_type;
using boost::false_type;
using boost::is_base_of;
using boost::is_pointer;
using boost::remove_pointer;
} }

#endif // VSMC_USE_STD_TYPE_TRAITS

#endif // VSMC_INTERNAL_TYPE_TRATIS
