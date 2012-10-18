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
using std::make_signed;
using std::remove_cv;
using std::remove_pointer;
#if VSMC_HAS_LIB_TYPE_TRAITS_COMPLETE
using std::is_void;
using std::is_integral;
using std::is_floating_point;
using std::is_array;
using std::is_lvalue_reference;
using std::is_rvalue_reference;
using std::is_member_object_pointer;
using std::is_member_function_pointer;
using std::is_enum;
using std::is_union;
using std::is_class;
using std::is_function;
using std::is_reference;
using std::is_arithmetic;
using std::is_fundamental;
using std::is_object;
using std::is_scalar;
using std::is_compound;
using std::is_member_pointer;
using std::is_const;
using std::is_volatile;
using std::is_trivial;
using std::is_trivially_copyable;
using std::is_standard_layout;
using std::is_pod;
using std::is_literal_type;
using std::is_empty;
using std::is_polymorphic;
using std::is_abstract;
using std::is_signed;
using std::is_unsigned;
using std::is_constructible;
using std::is_default_constructible;
using std::is_copy_constructible;
using std::is_move_constructible;
using std::is_assignable;
using std::is_copy_assignable;
using std::is_move_assignable;
using std::is_destructible;
using std::is_trivially_constructible;
using std::is_trivially_default_constructible;
using std::is_trivially_copy_constructible;
using std::is_trivially_move_constructible;
using std::is_trivially_assignable;
using std::is_trivially_copy_assignable;
using std::is_trivially_move_assignable;
using std::is_trivially_destructible;
using std::is_nothrow_constructible;
using std::is_nothrow_default_constructible;
using std::is_nothrow_copy_constructible;
using std::is_nothrow_move_constructible;
using std::is_nothrow_assignable;
using std::is_nothrow_copy_assignable;
using std::is_nothrow_move_assignable;
using std::is_nothrow_destructible;
using std::has_virtual_destructor;
using std::alignment_of;
using std::rank;
using std::extent;
using std::is_convertible;
using std::remove_const;
using std::remove_volatile;
using std::add_const;
using std::add_volatile;
using std::add_cv;
using std::remove_reference;
using std::add_lvalue_reference;
using std::add_rvalue_reference;
using std::make_unsigned;
using std::remove_extent;
using std::remove_all_extents;
using std::add_pointer;
using std::aligned_storage;
using std::decay;
using std::enable_if;
using std::conditional;
using std::common_type;
using std::underlying_type;
using std::result_of;
#endif // VSMC_HAS_LIB_TYPE_TRAITS_COMPLETE
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
using boost::make_signed;
using boost::remove_cv;
using boost::remove_pointer;
#if VSMC_HAS_LIB_TYPE_TRAITS_COMPLETE
using boost::is_void;
using boost::is_integral;
using boost::is_floating_point;
using boost::is_array;
using boost::is_lvalue_reference;
using boost::is_rvalue_reference;
using boost::is_member_object_pointer;
using boost::is_member_function_pointer;
using boost::is_enum;
using boost::is_union;
using boost::is_class;
using boost::is_function;
using boost::is_reference;
using boost::is_arithmetic;
using boost::is_fundamental;
using boost::is_object;
using boost::is_scalar;
using boost::is_compound;
using boost::is_member_pointer;
using boost::is_const;
using boost::is_volatile;
// using boost::is_trivial;
// using boost::is_trivially_copyable;
// using boost::is_standard_layout;
using boost::is_pod;
// using boost::is_literal_type;
using boost::is_empty;
using boost::is_polymorphic;
using boost::is_abstract;
using boost::is_signed;
using boost::is_unsigned;
// using boost::is_constructible;
// using boost::is_default_constructible;
// using boost::is_copy_constructible;
// using boost::is_move_constructible;
// using boost::is_assignable;
// using boost::is_copy_assignable;
// using boost::is_move_assignable;
// using boost::is_destructible;
// using boost::is_trivially_constructible;
// using boost::is_trivially_default_constructible;
// using boost::is_trivially_copy_constructible;
// using boost::is_trivially_move_constructible;
// using boost::is_trivially_assignable;
// using boost::is_trivially_copy_assignable;
// using boost::is_trivially_move_assignable;
// using boost::is_trivially_destructible;
// using boost::is_nothrow_constructible;
// using boost::is_nothrow_default_constructible;
// using boost::is_nothrow_copy_constructible;
// using boost::is_nothrow_move_constructible;
// using boost::is_nothrow_assignable;
// using boost::is_nothrow_copy_assignable;
// using boost::is_nothrow_move_assignable;
// using boost::is_nothrow_destructible;
using boost::has_virtual_destructor;
using boost::alignment_of;
using boost::rank;
using boost::extent;
using boost::is_convertible;
using boost::remove_const;
using boost::remove_volatile;
using boost::add_const;
using boost::add_volatile;
using boost::add_cv;
using boost::remove_reference;
using boost::add_lvalue_reference;
using boost::add_rvalue_reference;
using boost::make_unsigned;
using boost::remove_extent;
using boost::remove_all_extents;
using boost::add_pointer;
using boost::aligned_storage;
using boost::decay;
using boost::enable_if;
using boost::conditional;
using boost::common_type;
// using boost::underlying_type;
// using boost::result_of;
#endif // VSMC_HAS_LIB_TYPE_TRAITS_COMPLETE
} }
#endif // VSMC_HAS_CXX11LIB_TYPE_TRAITS

#endif // VSMC_CXX11_TYPE_TRATIS
