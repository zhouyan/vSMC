#ifndef V_SMC_INTERNAL_COMPILER_INTEL_HPP
#define V_SMC_INTERNAL_COMPILER_INTEL_HPP

#if __INTEL_COMPILER < 1300 || __cplusplus < 201103L
#undef V_SMC_HAS_CXX11_EXPLICIT_CONVERSION
#else
#define V_SMC_HAS_CXX11_EXPLICIT_CONVERSION
#endif

#endif // V_SMC_INTERNAL_COMPILER_INTEL_HPP
