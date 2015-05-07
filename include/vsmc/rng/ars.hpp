//============================================================================
// vSMC/include/vsmc/rng/ars.hpp
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

#ifndef VSMC_RNG_ARS_HPP
#define VSMC_RNG_ARS_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/aes_ni.hpp>

/// \brief ARSEngine default blocks
/// \ingroup Config
#ifndef VSMC_RNG_ARS_BLOCKS
#define VSMC_RNG_ARS_BLOCKS 1
#endif

/// \brief ARSEngine default rounds
/// \ingroup Config
#ifndef VSMC_RNG_ARS_ROUNDS
#define VSMC_RNG_ARS_ROUNDS 7
#endif

namespace vsmc
{

namespace traits
{

namespace internal
{

template <std::size_t>
struct ARSWeylConstantValue;

template <>
struct ARSWeylConstantValue<0> : public std::integral_constant<std::uint64_t,
                                     UINT64_C(0xBB67AE8584CAA73B)> {
};

template <>
struct ARSWeylConstantValue<1> : public std::integral_constant<std::uint64_t,
                                     UINT64_C(0x9E3779B97F4A7C15)> {
};

} // namespace vsmc::traits::internal

/// \brief ARSEngine Weyl sequence constants
/// \ingroup Traits
///
/// \details
/// The two specializaiton (N = 0, 1) corresponds to lower and upper 64-bits
/// or
/// the Weyl constant.
template <std::size_t I>
struct ARSWeylConstantTrait : public internal::ARSWeylConstantValue<I> {
};

} // namespace vsmc::traits

/// \brief Default ARSEngine key sequence generator
/// \ingroup AESNIRNG
template <typename ResultType>
class ARSKeySeq
{
    public:
    typedef std::array<ResultType, 16 / sizeof(ResultType)> key_type;

    ARSKeySeq()
        : weyl_(_mm_set_epi64x(static_cast<std::int64_t>(
                                   traits::ARSWeylConstantTrait<0>::value),
              static_cast<std::int64_t>(
                                   traits::ARSWeylConstantTrait<1>::value)))
    {
    }

    template <std::size_t Rp1>
    void generate(const key_type &key, std::array<__m128i, Rp1> &key_seq)
    {
        internal::m128i_pack<0>(key, key_seq.front());
        generate_seq<1>(key_seq, std::integral_constant<bool, 1 < Rp1>());
    }

    private:
    const __m128i weyl_;

    template <std::size_t, std::size_t Rp1>
    void generate_seq(std::array<__m128i, Rp1> &, std::false_type)
    {
    }

    template <std::size_t N, std::size_t Rp1>
    void generate_seq(std::array<__m128i, Rp1> &key_seq, std::true_type)
    {
        std::get<N>(key_seq) = _mm_add_epi64(std::get<N - 1>(key_seq), weyl_);
        generate_seq<N + 1>(
            key_seq, std::integral_constant<bool, N + 1 < Rp1>());
    }
}; // class ARSKeySeq

/// \brief ARS RNG engine
/// \ingroup AESNIRNG
///
/// \details
/// This is a reimplementation of the ARS engine as described in
/// [Parallel Random Numbers: As Easy as 1, 2, 3][r123paper] and implemented
/// in
/// [Random123][r123lib].
///
/// [r123paper]:http://sc11.supercomputing.org/schedule/event_detail.php?evid=pap274
/// [r123lib]: https://www.deshawresearch.com/resources_random123.html
///
/// \sa ARSKeySeq
/// \sa AESNIEngine
template <typename ResultType, std::size_t Rounds = VSMC_RNG_ARS_ROUNDS,
    std::size_t Blocks = VSMC_RNG_ARS_BLOCKS>
class ARSEngine : public AESNIEngine<ResultType, ARSKeySeq<ResultType>, false,
                      Rounds, Blocks>
{
    public:
    typedef AESNIEngine<ResultType, ARSKeySeq<ResultType>, false, Rounds,
        Blocks> base_eng_type;

    explicit ARSEngine(ResultType s = 0) : base_eng_type(s) {}

    template <typename SeedSeq>
    explicit ARSEngine(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq,
            typename base_eng_type::result_type,
            typename base_eng_type::key_type,
            ARSEngine<ResultType, Rounds, Blocks>>::value>::type * = nullptr)
        : base_eng_type(seq)
    {
    }

    ARSEngine(const typename base_eng_type::key_type &k) : base_eng_type(k) {}
}; // class ARSEngine

/// \brief ARS RNG engine with 32-bits integers output, default blocks and
/// default rounds
/// \ingroup AESNIRNG
typedef ARSEngine<std::uint32_t> ARS_32;

/// \brief ARS RNG engine with 32-bits integers output, 1 block and default
/// rounds
/// \ingroup AESNIRNG
typedef ARSEngine<std::uint32_t, VSMC_RNG_ARS_ROUNDS, 1> ARS_1x32;

/// \brief ARS RNG engine with 32-bits integers output, 2 blocks and default
/// rounds
/// \ingroup AESNIRNG
typedef ARSEngine<std::uint32_t, VSMC_RNG_ARS_ROUNDS, 2> ARS_2x32;

/// \brief ARS RNG engine with 32-bits integers output, 4 blocks and default
/// rounds
/// \ingroup AESNIRNG
typedef ARSEngine<std::uint32_t, VSMC_RNG_ARS_ROUNDS, 4> ARS_4x32;

/// \brief ARS RNG engine with 32-bits integers output, 8 blocks and default
/// rounds
/// \ingroup AESNIRNG
typedef ARSEngine<std::uint32_t, VSMC_RNG_ARS_ROUNDS, 8> ARS_8x32;

/// \brief ARS RNG engine with 64-bits integers output, default blocks and
/// default rounds
/// \ingroup AESNIRNG
typedef ARSEngine<std::uint64_t> ARS_64;

/// \brief ARS RNG engine with 64-bits integers output, 1 block and default
/// rounds
/// \ingroup AESNIRNG
typedef ARSEngine<std::uint64_t, VSMC_RNG_ARS_ROUNDS, 1> ARS_1x64;

/// \brief ARS RNG engine with 64-bits integers output, 2 blocks and default
/// rounds
/// \ingroup AESNIRNG
typedef ARSEngine<std::uint64_t, VSMC_RNG_ARS_ROUNDS, 2> ARS_2x64;

/// \brief ARS RNG engine with 64-bits integers output, 4 blocks and default
/// rounds
/// \ingroup AESNIRNG
typedef ARSEngine<std::uint64_t, VSMC_RNG_ARS_ROUNDS, 4> ARS_4x64;

/// \brief ARS RNG engine with 64-bits integers output, 8 blocks and default
/// rounds
/// \ingroup AESNIRNG
typedef ARSEngine<std::uint64_t, VSMC_RNG_ARS_ROUNDS, 8> ARS_8x64;

} // namespace vsmc

#endif // VSMC_RNG_ARS_HPP
