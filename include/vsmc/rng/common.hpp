#ifndef VSMC_RNG_COMMON_HPP
#define VSMC_RNG_COMMON_HPP

#include <cassert>
#include <cmath>
#include <limits>

#define VSMC_PREVENT_MIN_MAX

/// The parallel RNG (based on Random123) seed, unsigned
#ifndef VSMC_CBRNG_SEED
#define VSMC_CBRNG_SEED 0xdeadbeefU
#endif // VSMC_CBRNG_SEED

/// The parallel RNG (based on Random123) type, philox or threefry
#ifndef VSMC_CBRNG_TYPE
#define VSMC_CBRNG_TYPE r123::Threefry4x64
#endif // VSMC_CBRNG_TYPE

#endif // VSMC_RNG_COMMON_HPP
