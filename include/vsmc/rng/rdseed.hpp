//============================================================================
// include/vsmc/rng/rdseed.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_RNG_RDSEED_HPP
#define VSMC_RNG_RDSEED_HPP

#include <vsmc/rng/generator_wrapper.hpp>

#ifdef _MSC_VER
#include <intrin.h>
#endif

#ifndef VSMC_RDSEED_NTRIAL_MAX
#define VSMC_RDSEED_NTRIAL_MAX 10
#endif

#define VSMC_STATIC_ASSERT_RNG_RDSEED_STEP_RESULT_TYPE(ResultType) \
    VSMC_STATIC_ASSERT((                                                     \
                cxx11::is_same<ResultType, uint16_t>::value ||               \
                cxx11::is_same<ResultType, uint32_t>::value ||               \
                cxx11::is_same<ResultType, uint64_t>::value),                \
            USE_RdSeedStep_WITH_RESULT_TYPE_OTHER_THAN_uint16_t_OR_uint32_t_OR_uint64_t)

#define VSMC_STATIC_ASSERT_RNG_RDSEED_STEP \
    VSMC_STATIC_ASSERT_RNG_RDSEED_STEP_RESULT_TYPE(ResultType);

#define VSMC_RUNTIME_WARNING_RNG_RDSEED_STEP_NTRIAL(ntrial, NTrialMax) \
    VSMC_RUNTIME_WARNING((ntrial < NTrialMax),                                \
            ("**RdSeedStep::generate** MAXIMUM NUMBER OF TRIALS EXCEEDED"))

namespace vsmc {

namespace internal {

template <typename UIntType> inline bool rdseed_step (UIntType *);

#ifdef _MSC_VER

template <> inline bool rdseed_step<uint16_t> (uint16_t *seed)
{
    unsigned short r;
    int cf = _rdseed16_step(&r);
    *seed = static_cast<uint16_t>(r);

    return cf != 0;
}

template <> inline bool rdseed_step<uint32_t> (uint32_t *seed)
{
    unsigned r;
    int cf = _rdseed32_step(&r);
    *seed = static_cast<uint32_t>(r);

    return cf != 0;
}

template <> inline bool rdseed_step<uint64_t> (uint64_t *seed)
{
    unsigned __int64 r;
    int cf = _rdseed64_step(&r);
    *seed = static_cast<uint64_t>(r);

    return cf != 0;
}

#else // _MSC_VER

template <> inline bool rdseed_step<uint16_t> (uint16_t *seed)
{
    unsigned char cf = 0;
    __asm__ volatile(
            "rdseedw %0; setcb %1\n"
            : "=r" (*seed), "=qm" (cf));

    return cf != 0;
}

template <> inline bool rdseed_step<uint32_t> (uint32_t *seed)
{
    unsigned char cf = 0;
    __asm__ volatile(
            "rdseedl %0; setcb %1\n"
            : "=r" (*seed), "=qm" (cf));

    return cf != 0;
}

template <> inline bool rdseed_step<uint64_t> (uint64_t *seed)
{
    unsigned char cf = 0;
    __asm__ volatile(
            "rdseedq %0; setcb %1\n"
            : "=r" (*seed), "=qm" (cf));

    return cf != 0;
}

#endif // _MSC_VER

} // namespace internal

/// \brief RdSeed generator
/// \ingroup RDRNG
template <typename ResultType, std::size_t NTrialMax = VSMC_RDSEED_NTRIAL_MAX>
class RdSeedStep
{
    public :

    typedef ResultType result_type;

    static result_type generate ()
    {
        VSMC_STATIC_ASSERT_RNG_RDSEED_STEP;

        result_type r;
        std::size_t ntrial = 0;
        while (!internal::rdseed_step<result_type>(&r) && ntrial != NTrialMax)
            ++ntrial;
        VSMC_RUNTIME_WARNING_RNG_RDSEED_STEP_NTRIAL(ntrial, NTrialMax);

        return r;
    }
}; // class RdSeedStep

/// \brief C++11 Engine using 16-bits RdSeed instruction
/// \ingroup RDRNG
typedef GeneratorWrapper<uint16_t, RdSeedStep<uint16_t> > RdSeed16;

/// \brief C++11 Engine using 32-bits RdSeed instruction
/// \ingroup RDRNG
typedef GeneratorWrapper<uint32_t, RdSeedStep<uint32_t> > RdSeed32;

/// \brief C++11 Engine using 64-bits RdSeed instruction
/// \ingroup RDRNG
typedef GeneratorWrapper<uint64_t, RdSeedStep<uint64_t> > RdSeed64;

} // namespace vsmc

#endif//  VSMC_RNG_RDSEED_HPP

