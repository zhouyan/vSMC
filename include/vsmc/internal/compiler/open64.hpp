#ifndef VSMC_INTERNAL_COMPILER_OPEN64_HPP
#define VSMC_INTERNAL_COMPILER_OPEN64_HPP

#ifndef VSMC_HAS_CXX11_LONG_LONG
#define VSMC_HAS_CXX11_LONG_LONG 1
#endif

// Target specific features

#if defined(__x86_64__) || defined(_M_AMD64) || defined (_M_X64)
#ifndef VSMC_HAS_X86_64
#define VSMC_HAS_X86_64 1
#endif
#endif

#ifndef VSMC_HAS_INLINE_ASSEMBLY
#define VSMC_HAS_INLINE_ASSEMBLY 1
#endif

#endif // VSMC_INTERNAL_COMPILER_OPEN64_HPP
