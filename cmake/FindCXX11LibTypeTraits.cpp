//============================================================================
// vSMC/cmake/FindCXX11LibTypeTraits.cpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#include <type_traits>
#include <utility>

#ifdef __GLIBCXX__

#define DEFINE_CXX11_TYPE_TRAITS_LIBSTDCPP_TEST(name) template <typename> struct name : public false_type {};

#define DEFINE_CXX11_TYPE_TRAITS_LIBSTDCPP_TYPE(name)

#else // LIBSTDCPP_VERSION

#define DEFINE_CXX11_TYPE_TRAITS_LIBSTDCPP_TEST(name) using std::name;

#define DEFINE_CXX11_TYPE_TRAITS_LIBSTDCPP_TYPE(name) using std::name;

#endif // __GLIBCXX__

// Helper classes
using std::integral_constant;
using std::true_type;
using std::false_type;

// Primary type categories
using std::is_void;
using std::is_integral;
using std::is_floating_point;
using std::is_array;
using std::is_enum;
using std::is_union;
using std::is_class;
using std::is_function;
using std::is_pointer;
using std::is_lvalue_reference;
using std::is_rvalue_reference;
using std::is_member_object_pointer;
using std::is_member_function_pointer;

// Composite type categories
using std::is_fundamental;
using std::is_arithmetic;
using std::is_scalar;
using std::is_object;
using std::is_compound;
using std::is_reference;
using std::is_member_pointer;

// Type properties
using std::is_const;
using std::is_volatile;
using std::is_trivial;
DEFINE_CXX11_TYPE_TRAITS_LIBSTDCPP_TEST(is_trivially_copyable)
using std::is_standard_layout;
using std::is_pod;
using std::is_literal_type;
using std::is_empty;
using std::is_polymorphic;
using std::is_abstract;
using std::is_signed;
using std::is_unsigned;

// Supported operations
using std::is_constructible;
DEFINE_CXX11_TYPE_TRAITS_LIBSTDCPP_TEST(is_trivially_constructible)
using std::is_nothrow_constructible;
using std::is_default_constructible;
DEFINE_CXX11_TYPE_TRAITS_LIBSTDCPP_TEST(is_trivially_default_constructible)
using std::is_nothrow_default_constructible;
using std::is_copy_constructible;
DEFINE_CXX11_TYPE_TRAITS_LIBSTDCPP_TEST(is_trivially_copy_constructible)
using std::is_nothrow_copy_constructible;
using std::is_move_constructible;
DEFINE_CXX11_TYPE_TRAITS_LIBSTDCPP_TEST(is_trivially_move_constructible)
using std::is_nothrow_move_constructible;
using std::is_assignable;
DEFINE_CXX11_TYPE_TRAITS_LIBSTDCPP_TEST(is_trivially_assignable)
using std::is_nothrow_assignable;
using std::is_copy_assignable;
DEFINE_CXX11_TYPE_TRAITS_LIBSTDCPP_TEST(is_trivially_copy_assignable)
using std::is_nothrow_copy_assignable;
using std::is_move_assignable;
DEFINE_CXX11_TYPE_TRAITS_LIBSTDCPP_TEST(is_trivially_move_assignable)
using std::is_nothrow_move_assignable;
using std::is_destructible;
using std::is_trivially_destructible;
using std::is_nothrow_destructible;
using std::has_virtual_destructor;

// Property queries
using std::alignment_of;
using std::rank;
using std::extent;

// Type relations
using std::is_same;
using std::is_base_of;
using std::is_convertible;

// Const-volatility specifiers
using std::remove_cv;
using std::remove_const;
using std::remove_volatile;
using std::add_cv;
using std::add_const;
using std::add_volatile;

// References
using std::remove_reference;
using std::add_lvalue_reference;
using std::add_rvalue_reference;

// Pointers
using std::remove_pointer;
using std::add_pointer;

// Sign modifiers
using std::make_signed;
using std::make_unsigned;

// Arrays
using std::remove_extent;
using std::remove_all_extents;

// Miscellaneous transformations
using std::aligned_storage;
DEFINE_CXX11_TYPE_TRAITS_LIBSTDCPP_TYPE(aligned_union)
using std::decay;
using std::enable_if;
using std::conditional;
using std::common_type;
using std::underlying_type;
using std::result_of;

// Utilities
using std::declval;

int main ()
{
    return 0;
}
