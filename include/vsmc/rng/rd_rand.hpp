#ifndef VSMC_RNG_RD_RAND_HPP
#define VSMC_RNG_RD_RAND_HPP

#include <vsmc/rng/generator_wrapper.hpp>
#include <vsmc/rng/engine_result_wrapper.hpp>
#include <immintrin.h>

namespace vsmc {

namespace internal {

template <typename> struct RdRandStep;

template <> struct RdRandStep<uint16_t>
{
    static uint16_t generate ()
    {
        unsigned short r;
        while (!(_rdrand16_step(&r)))
            ;

        return static_cast<uint16_t>(r);
    }
}; // struct RdRandStep

template <> struct RdRandStep<uint32_t>
{
    static uint32_t generate ()
    {
        unsigned r;
        while (!(_rdrand32_step(&r)))
            ;

        return static_cast<uint32_t>(r);
    }
}; // struct RdRandStep

template <> struct RdRandStep<uint64_t>
{
    static uint64_t generate ()
    {
        unsigned long long r;
        while (!(_rdrand64_step(&r)))
            ;

        return static_cast<uint64_t>(r);
    }
}; // struct RdRandStep

} // namespace vsmc::internal

/// \brief C++11 Engine that wraps _rdrand16_step
/// \ingroup RDRNG
typedef GeneratorWrapper<uint16_t, internal::RdRandStep<uint16_t> > RdRand16;

/// \brief C++11 Engine that wraps _rdrand32_step
/// \ingroup RDRNG
typedef GeneratorWrapper<uint32_t, internal::RdRandStep<uint32_t> > RdRand32;

/// \brief C++11 Engine that wraps _rdrand64_step
/// \ingroup RDRNG
typedef GeneratorWrapper<uint64_t, internal::RdRandStep<uint64_t> > RdRand64;

/// \brief C++11 Engine that wraps _rdrand32_step but output 16-bits integers
/// \ingroup RDRNG
typedef EngineResultWrapper<uint16_t, RdRand32> RdRand32_16;

/// \brief C++11 Engine that wraps _rdrand64_step but output 16-bits integers
/// \ingroup RDRNG
typedef EngineResultWrapper<uint16_t, RdRand64> RdRand64_16;

/// \brief C++11 Engine that wraps _rdrand64_step but output 32-bits integers
/// \ingroup RDRNG
typedef EngineResultWrapper<uint32_t, RdRand64> RdRand64_32;

} // namespace vsmc

#endif//  VSMC_RNG_RD_RAND_HPP
