#ifndef V_SMC_INTERNAL_COMPILER_GCC_HPP
#define V_SMC_INTERNAL_COMPILER_GCC_HPP

#if !__has_feature(cxx_explicit_conversions) || __cplusplus < 201103L
#undef V_SMC_HAS_CXX11_EXPLICIT_CONVERSION
#else
#define V_SMC_HAS_CXX11_EXPLICIT_CONVERSION
#endif

#endif // V_SMC_INTERNAL_COMPILER_GCC_HPP
