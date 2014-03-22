#ifndef VSMC_INTERNAL_COMPILER_SUNPRO_HPP
#define VSMC_INTERNAL_COMPILER_SUNPRO_HPP

#pragma error_messages(off, nonewline)
#pragma error_messages(off, anonnotype)

#ifndef VSMC_HAS_CXX11_LONG_LONG
#define VSMC_HAS_CXX11_LONG_LONG 1
#endif

// Target specific features

#if defined(__x86_64__) || defined(_M_AMD64) || defined (_M_X64)
#ifndef VSMC_HAS_X86_64
#define VSMC_HAS_X86_64 1
#endif
#endif

#endif // VSMC_INTERNAL_COMPILER_SUNPRO_HPP
