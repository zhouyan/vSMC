//============================================================================
// include/vsmc/rng/rdrand.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_RNG_RDRAND_HPP
#define VSMC_RNG_RDRAND_HPP

#include <vsmc/rng/generator_wrapper.hpp>

#ifdef _MSC_VER
#include <intrin.h>
#endif

#ifndef VSMC_RDRAND_NTRIAL_MAX
#define VSMC_RDRAND_NTRIAL_MAX 10
#endif

#define VSMC_STATIC_ASSERT_RNG_RDRAND_STEP_RESULT_TYPE(ResultType) \
    VSMC_STATIC_ASSERT((                                                     \
                cxx11::is_same<ResultType, uint16_t>::value ||               \
                cxx11::is_same<ResultType, uint32_t>::value ||               \
                cxx11::is_same<ResultType, uint64_t>::value),                \
            USE_RdRandStep_WITH_RESULT_TYPE_OTHER_THAN_uint16_t_OR_uint32_t_OR_uint64_t)

#define VSMC_STATIC_ASSERT_RNG_RDRAND_STEP \
    VSMC_STATIC_ASSERT_RNG_RDRAND_STEP_RESULT_TYPE(ResultType);

#define VSMC_RUNTIME_WARNING_RNG_RDRAND_STEP_NTRIAL(ntrial, NTrialMax) \
    VSMC_RUNTIME_WARNING((ntrial < NTrialMax),                                \
            ("**RdSeedStep::generate** MAXIMUM NUMBER OF TRIALS EXCEEDED"))

namespace vsmc {

namespace internal {

template <typename UIntType> inline bool rdrand_step (UIntType *);

#ifdef _MSC_VER

template <> inline bool rdrand_step<uint16_t> (uint16_t *rand)
{
    unsigned short r;
    int cf = _rdrand16_step(&r);
    *rand = static_cast<uint16_t>(r);

    return cf != 0;
}

template <> inline bool rdrand_step<uint32_t> (uint32_t *rand)
{
    unsigned r;
    int cf = _rdrand32_step(&r);
    *rand = static_cast<uint32_t>(r);

    return cf != 0;
}

template <> inline bool rdrand_step<uint64_t> (uint64_t *rand)
{
    unsigned __int64 r;
    int cf = _rdrand64_step(&r);
    *rand = static_cast<uint64_t>(r);

    return cf != 0;
}

#else // _MSC_VER

template <> inline bool rdrand_step<uint16_t> (uint16_t *rand)
{
    unsigned char cf = 0;
    __asm__ volatile(
            "rdrandw %0; setcb %1\n"
            : "=r" (*rand), "=qm" (cf));

    return cf != 0;
}

template <> inline bool rdrand_step<uint32_t> (uint32_t *rand)
{
    unsigned char cf = 0;
    __asm__ volatile(
            "rdrandl %0; setcb %1\n"
            : "=r" (*rand), "=qm" (cf));

    return cf != 0;
}

template <> inline bool rdrand_step<uint64_t> (uint64_t *rand)
{
    unsigned char cf = 0;
    __asm__ volatile(
            "rdrandq %0; setcb %1\n"
            : "=r" (*rand), "=qm" (cf));

    return cf != 0;
}

#endif // _MSC_VER

} // namespace internal

/// \brief RdRand generator
/// \ingroup RDRNG
template <typename ResultType, std::size_t NTrialMax = VSMC_RDRAND_NTRIAL_MAX>
class RdRandStep
{
    public :

    typedef ResultType result_type;

    static result_type generate ()
    {
        VSMC_STATIC_ASSERT_RNG_RDRAND_STEP;

        result_type r;
        std::size_t ntrial = 0;
        while (!internal::rdrand_step<result_type>(&r) && ntrial != NTrialMax)
            ++ntrial;
        VSMC_RUNTIME_WARNING_RNG_RDRAND_STEP_NTRIAL(ntrial, NTrialMax);

        return r;
    }
}; // class RdRandStep

/// \brief C++11 Engine using 16-bits RdRand instruction
/// \ingroup RDRNG
typedef GeneratorWrapper<uint16_t, RdRandStep<uint16_t> > RdRand16;

/// \brief C++11 Engine using 32-bits RdRand instruction
/// \ingroup RDRNG
typedef GeneratorWrapper<uint32_t, RdRandStep<uint32_t> > RdRand32;

/// \brief C++11 Engine using 64-bits RdRand instruction
/// \ingroup RDRNG
typedef GeneratorWrapper<uint64_t, RdRandStep<uint64_t> > RdRand64;

} // namespace vsmc

#endif//  VSMC_RNG_RDRAND_HPP
