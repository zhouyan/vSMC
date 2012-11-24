#ifndef VSMC_CXX11_TYPE_TRATIS
#define VSMC_CXX11_TYPE_TRATIS

#include <vsmc/internal/config.hpp>

#if VSMC_HAS_CXX11LIB_TYPE_TRAITS
#include <type_traits>
namespace vsmc { namespace cxx11 {
using std::integral_constant;
using std::true_type;
using std::false_type;
using std::is_base_of;
using std::is_pointer;
using std::is_same;
using std::is_signed;
using std::make_signed;
using std::remove_cv;
using std::remove_pointer;
using std::enable_if;
} }
#else // VSMC_HAS_CXX11LIB_TYPE_TRAITS
#include <boost/type_traits.hpp>
namespace vsmc { namespace cxx11 {
using boost::integral_constant;
using boost::true_type;
using boost::false_type;
using boost::is_base_of;
using boost::is_pointer;
using boost::is_same;
using std::is_signed;
using boost::make_signed;
using boost::remove_cv;
using boost::remove_pointer;
template<bool, class T = void> struct enable_if {};
template<class T> struct enable_if<true, T> {typedef T type;};
} }
#endif // VSMC_HAS_CXX11LIB_TYPE_TRAITS

#endif // VSMC_CXX11_TYPE_TRATIS
