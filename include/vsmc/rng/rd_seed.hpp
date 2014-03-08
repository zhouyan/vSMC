#ifndef VSMC_RD_RSEED_HPP
#define VSMC_RD_RSEED_HPP

#include <vsmc/rng/generator_wrapper.hpp>
#include <vsmc/rng/engine_result_wrapper.hpp>

namespace vsmc {

namespace internal {

template <typename> struct RdSeedStep;

template <> struct RdSeedStep<uint16_t>
{
    static uint16_t generate ()
    {
        unsigned short r;
        while (!(_rdseed16_step(&r)))
            ;

        return static_cast<uint16_t>(r);
    }
};

template <> struct RdSeedStep<uint32_t>
{
    static uint32_t generate ()
    {
        unsigned r;
        while (!(_rdseed32_step(&r)))
            ;

        return static_cast<uint32_t>(r);
    }
};

template <> struct RdSeedStep<uint64_t>
{
    static uint64_t generate ()
    {
        unsigned __int64 r;
        while (!(_rdseed64_step(&r)))
            ;

        return static_cast<uint64_t>(r);
    }
};

} // namespace vsmc::internal

/// \brief C++11 Engine that wraps _rdseed16_step
/// \ingroup RNGWrapper
typedef RdSeedEngine<uint16_t, internal::RdSeedStep<uint16_t> > RdSeed16;

/// \brief C++11 Engine that wraps _rdseed32_step
/// \ingroup RNGWrapper
typedef RdSeedEngine<uint32_t, internal::RdSeedStep<uint32_t> > RdSeed32;

/// \brief C++11 Engine that wraps _rdseed64_step
/// \ingroup RNGWrapper
typedef RdSeedEngine<uint64_t, internal::RdSeedStep<uint64_t> > RdSeed64;

/// \brief C++11 Engine that wraps _rdseed32_step but output 16-bits integers
/// \ingroup RNGWrapper
typedef EngineResultWrapper<uint16_t, RdSeed32> RdSeed32_16;

/// \brief C++11 Engine that wraps _rdseed64_step but output 16-bits integers
/// \ingroup RNGWrapper
typedef EngineResultWrapper<uint16_t, RdSeed64> RdSeed64_16;

/// \brief C++11 Engine that wraps _rdseed64_step but output 32-bits integers
/// \ingroup RNGWrapper
typedef EngineResultWrapper<uint32_t, RdSeed64> RdSeed64_32;

} // namespace vsmc

#endif // VSMC_RD_RSEED_HPP
