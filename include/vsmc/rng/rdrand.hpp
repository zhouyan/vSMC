//============================================================================
// vSMC/include/vsmc/rng/rdrand.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#ifndef VSMC_RNG_RDRAND_HPP
#define VSMC_RNG_RDRAND_HPP

#include <vsmc/rng/internal/common.hpp>
#include <immintrin.h>

#ifndef VSMC_RDRAND_NTRIAL_MAX
#define VSMC_RDRAND_NTRIAL_MAX 0
#endif

#define VSMC_STATIC_ASSERT_RNG_RDRAND_ENGINE_RESULT_TYPE(ResultType)          \
    VSMC_STATIC_ASSERT(((sizeof(ResultType) == sizeof(std::uint16_t) &&       \
                            std::is_unsigned<ResultType>::value) ||           \
                           (sizeof(ResultType) == sizeof(std::uint32_t) &&    \
                               std::is_unsigned<ResultType>::value) ||        \
                           (sizeof(ResultType) == sizeof(std::uint64_t) &&    \
                               std::is_unsigned<ResultType>::value)),         \
        "**RDRANDEngine** USED WITH ResultType OTHER THAN UNSIGNED 16/32/64 " \
        "BITS INTEGER")

#define VSMC_STATIC_ASSERT_RNG_RDRAND_ENGINE                                  \
    VSMC_STATIC_ASSERT_RNG_RDRAND_ENGINE_RESULT_TYPE(ResultType);

#define VSMC_RUNTIME_WARNING_RNG_RDRAND_ENGINE_NTRIAL(ntrial, NTrialMax)      \
    VSMC_RUNTIME_WARNING((ntrial < NTrialMax),                                \
        "**RDRAND::generate** MAXIMUM NUMBER OF TRIALS EXCEEDED")

namespace vsmc
{

/// \brief Invoke the RDRAND instruction and return the carry flag
/// \ingroup RDRAND
template <typename UIntType, std::size_t W>
inline bool rdrand(UIntType *, std::integral_constant<std::size_t, W>);

/// \brief Invoke the 16-bits RDRAND instruction and return the carry flag
/// \ingroup RDRAND
template <typename UIntType>
inline bool rdrand(
    UIntType *rand, std::integral_constant<std::size_t, sizeof(std::uint16_t)>)
{
    unsigned short r;
    int cf = _rdrand16_step(&r);
    *rand = static_cast<UIntType>(r);

    return cf != 0;
}

/// \brief Invoke the 32-bits RDRAND instruction and return the carry flag
/// \ingroup RDRAND
template <typename UIntType>
inline bool rdrand(
    UIntType *rand, std::integral_constant<std::size_t, sizeof(std::uint32_t)>)
{
    unsigned r;
    int cf = _rdrand32_step(&r);
    *rand = static_cast<UIntType>(r);

    return cf != 0;
}

/// \brief Invoke the 64-bits RDRAND instruction and return the carry flag
/// \ingroup RDRAND
template <typename UIntType>
inline bool rdrand(
    UIntType *rand, std::integral_constant<std::size_t, sizeof(std::uint64_t)>)
{
    unsigned VSMC_INT64 r;
    int cf = _rdrand64_step(&r);
    *rand = static_cast<UIntType>(r);

    return cf != 0;
}

/// \brief RDRAND generator
/// \ingroup RDRAND
template <typename ResultType, std::size_t NTrialMax = VSMC_RDRAND_NTRIAL_MAX>
class RDRANDEngine
{

    public:
    using result_type = ResultType;

    explicit RDRANDEngine(result_type = 0)
    {
        VSMC_STATIC_ASSERT_RNG_RDRAND_ENGINE;
    }

    template <typename SeedSeq>
    explicit RDRANDEngine(SeedSeq &,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, result_type,
            RDRANDEngine<ResultType, NTrialMax>>::value>::type * = nullptr)
    {
        VSMC_STATIC_ASSERT_RNG_RDRAND_ENGINE;
    }

    void seed(result_type) {}

    template <typename SeedSeq>
    void seed(SeedSeq &, typename std::enable_if<internal::is_seed_seq<SeedSeq,
                             result_type>::value>::type * = nullptr)
    {
    }

    result_type operator()()
    {
        return generate(std::integral_constant<bool, NTrialMax != 0>());
    }

    void discard(std::size_t) {}

    static constexpr result_type min VSMC_MNE()
    {
        return std::numeric_limits<result_type>::min VSMC_MNE();
    }

    static constexpr result_type max VSMC_MNE()
    {
        return std::numeric_limits<result_type>::max VSMC_MNE();
    }

    friend bool operator==(const RDRANDEngine<ResultType, NTrialMax> &,
        const RDRANDEngine<ResultType, NTrialMax> &)
    {
        return false;
    }

    friend bool operator!=(const RDRANDEngine<ResultType, NTrialMax> &,
        const RDRANDEngine<ResultType, NTrialMax> &)
    {
        return true;
    }

    template <typename CharT, typename CharTraits>
    friend std::basic_ostream<CharT, CharTraits> &operator<<(
        std::basic_ostream<CharT, CharTraits> &os,
        const RDRANDEngine<ResultType, NTrialMax> &)
    {
        return os;
    }

    template <typename CharT, typename CharTraits>
    friend std::basic_istream<CharT, CharTraits> &operator>>(
        std::basic_istream<CharT, CharTraits> &is,
        RDRANDEngine<ResultType, NTrialMax> &)
    {
        return is;
    }

    private:
    result_type generate(std::true_type)
    {
        result_type r;
        std::size_t ntrial = 0;
        while (true) {
            ++ntrial;
            bool success = rdrand<result_type>(&r,
                std::integral_constant<std::size_t, sizeof(result_type)>());
            if (success || ntrial > NTrialMax)
                break;
        }
        VSMC_RUNTIME_WARNING_RNG_RDRAND_ENGINE_NTRIAL(ntrial, NTrialMax);

        return r;
    }

    result_type generate(std::false_type)
    {
        result_type r;
        while (true) {
            bool success = rdrand<result_type>(&r,
                std::integral_constant<std::size_t, sizeof(result_type)>());
            if (success)
                break;
        }

        return r;
    }
}; // class RDRANDEngine

/// \brief C++11 Engine using 16-bits RDRAND instruction
/// \ingroup RDRAND
using RDRAND16 = RDRANDEngine<std::uint16_t>;

/// \brief C++11 Engine using 32-bits RDRAND instruction
/// \ingroup RDRAND
using RDRAND32 = RDRANDEngine<std::uint32_t>;

/// \brief C++11 Engine using 64-bits RDRAND instruction
/// \ingroup RDRAND
using RDRAND64 = RDRANDEngine<std::uint64_t>;

} // namespace vsmc

#endif //  VSMC_RNG_RDRAND_HPP
