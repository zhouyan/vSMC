//============================================================================
// vSMC/include/vsmc/rng/ars.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2016, Yan Zhou
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

/// \brief ARSEngine default rounds
/// \ingroup Config
#ifndef VSMC_RNG_ARS_ROUNDS
#define VSMC_RNG_ARS_ROUNDS 5
#endif

/// \brief ARSEngine default blocks
/// \ingroup Config
#ifndef VSMC_RNG_ARS_BLOCKS
#define VSMC_RNG_ARS_BLOCKS 8
#endif

namespace vsmc
{

namespace internal
{

template <std::size_t>
class ARSWeylConstant;

template <>
class ARSWeylConstant<0> : public std::integral_constant<std::uint64_t,
                               UINT64_C(0x9E3779B97F4A7C15)>
{
}; // class ARSWeylConstant

template <>
class ARSWeylConstant<1> : public std::integral_constant<std::uint64_t,
                               UINT64_C(0xBB67AE8584CAA73B)>
{
}; // class ARSWeylConstant

} // namespace vsmc::internal

/// \brief ARSEngine Weyl sequence constants
/// \ingroup Traits
///
/// \details
/// The two specializaiton (N = 0, 1) corresponds to lower and upper 64-bit
/// or the Weyl constant.
template <std::size_t I>
class ARSWeylConstantTrait : public internal::ARSWeylConstant<I>
{
}; // class ARSWeylConstantTrait

/// \brief Default ARSEngine key sequence generator
/// \ingroup AESNIRNG
class ARSKeySeq
{
    public:
    using key_type = std::array<std::uint64_t, 2>;

    void reset(const key_type &) {}

    template <std::size_t Rp1>
    const std::array<M128I<>, Rp1> &operator()(
        const key_type &key, std::array<M128I<>, Rp1> &rk) const
    {
        M128I<std::uint64_t> weyl;
        weyl.set(
            ARSWeylConstantTrait<1>::value, ARSWeylConstantTrait<0>::value);
        std::get<0>(rk).load(key.data());
        generate<1>(rk, weyl, std::integral_constant<bool, 1 < Rp1>());

        return rk;
    }

    private:
    template <std::size_t, std::size_t Rp1>
    void generate(std::array<M128I<>, Rp1> &, const M128I<std::uint64_t> &,
        std::false_type) const
    {
    }

    template <std::size_t N, std::size_t Rp1>
    void generate(std::array<M128I<>, Rp1> &rk,
        const M128I<std::uint64_t> &weyl, std::true_type) const
    {
        std::get<N>(rk) =
            _mm_add_epi64(std::get<N - 1>(rk).value(), weyl.value());
        generate<N + 1>(rk, weyl, std::integral_constant<bool, N + 1 < Rp1>());
    }
}; // class ARSKeySeq

/// \brief ARS RNG engine
/// \ingroup AESNIRNG
template <typename ResultType, std::size_t Rounds = VSMC_RNG_ARS_ROUNDS,
    std::size_t Blocks = VSMC_RNG_ARS_BLOCKS>
using ARSEngine = AESNIEngine<ResultType, ARSKeySeq, Rounds, Blocks>;

/// \brief ARS RNG engine with 32-bit integers output, 1 block and default
/// rounds
/// \ingroup AESNIRNG
using ARSx1 = ARSEngine<std::uint32_t, VSMC_RNG_ARS_ROUNDS, 1>;

/// \brief ARS RNG engine with 32-bit integers output, 2 blocks and default
/// rounds
/// \ingroup AESNIRNG
using ARSx2 = ARSEngine<std::uint32_t, VSMC_RNG_ARS_ROUNDS, 2>;

/// \brief ARS RNG engine with 32-bit integers output, 4 blocks and default
/// rounds
/// \ingroup AESNIRNG
using ARSx4 = ARSEngine<std::uint32_t, VSMC_RNG_ARS_ROUNDS, 4>;

/// \brief ARS RNG engine with 32-bit integers output, 8 blocks and default
/// rounds
/// \ingroup AESNIRNG
using ARSx8 = ARSEngine<std::uint32_t, VSMC_RNG_ARS_ROUNDS, 8>;

/// \brief ARS RNG engine with 64-bit integers output, 1 block and default
/// rounds
/// \ingroup AESNIRNG
using ARSx1_64 = ARSEngine<std::uint64_t, VSMC_RNG_ARS_ROUNDS, 1>;

/// \brief ARS RNG engine with 64-bit integers output, 2 blocks and default
/// rounds
/// \ingroup AESNIRNG
using ARSx2_64 = ARSEngine<std::uint64_t, VSMC_RNG_ARS_ROUNDS, 2>;

/// \brief ARS RNG engine with 64-bit integers output, 4 blocks and default
/// rounds
/// \ingroup AESNIRNG
using ARSx4_64 = ARSEngine<std::uint64_t, VSMC_RNG_ARS_ROUNDS, 4>;

/// \brief ARS RNG engine with 64-bit integers output, 8 blocks and default
/// rounds
/// \ingroup AESNIRNG
using ARSx8_64 = ARSEngine<std::uint64_t, VSMC_RNG_ARS_ROUNDS, 8>;

/// \brief ARS RNG engine with 32-bit integers output, default blocks and
/// default rounds
/// \ingroup AESNIRNG
using ARS = ARSEngine<std::uint32_t>;

/// \brief ARS RNG engine with 64-bit integers output, default blocks and
/// default rounds
/// \ingroup AESNIRNG
using ARS_64 = ARSEngine<std::uint64_t>;

} // namespace vsmc

#endif // VSMC_RNG_ARS_HPP
