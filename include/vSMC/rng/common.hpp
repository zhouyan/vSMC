#ifndef V_SMC_RNG_COMMON_HPP
#define V_SMC_RNG_COMMON_HPP

#include <cassert>
#include <cmath>
#include <limits>

#define V_SMC_PREVENT_MIN_MAX

/// The parallel RNG (based on Random123) seed, unsigned
#ifndef V_SMC_CBRNG_SEED
#define V_SMC_CBRNG_SEED 0xdeadbeefU
#endif // V_SMC_CBRNG_SEED

/// The parallel RNG (based on Random123) type, philox or threefry
#ifndef V_SMC_CBRNG_TYPE
#define V_SMC_CBRNG_TYPE r123::Threefry4x64
#endif // V_SMC_CBRNG_TYPE

#endif // V_SMC_RNG_COMMON_HPP
