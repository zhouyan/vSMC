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
#define VSMC_RNG_ARS_ROUNDS 5
#endif

namespace vsmc
{

namespace internal
{

template <std::size_t>
class ARSWeylConstant;

template <>
class ARSWeylConstant<0> : public std::integral_constant<std::uint64_t,
                               UINT64_C(0xBB67AE8584CAA73B)>
{
}; // class ARSWeylConstant

template <>
class ARSWeylConstant<1> : public std::integral_constant<std::uint64_t,
                               UINT64_C(0x9E3779B97F4A7C15)>
{
}; // class ARSWeylConstant

} // namespace vsmc::internal

/// \brief ARSEngine Weyl sequence constants
/// \ingroup Traits
///
/// \details
/// The two specializaiton (N = 0, 1) corresponds to lower and upper 64-bits
/// or the Weyl constant.
template <std::size_t I>
class ARSWeylConstantTrait : public internal::ARSWeylConstant<I>
{
}; // class ARSWeylConstantTrait

/// \brief Default ARSEngine key sequence generator
/// \ingroup AESNIRNG
template <typename ResultType>
class ARSKeySeq
{
    public:
    using key_type = std::array<ResultType, M128I<ResultType>::size()>;

    ARSKeySeq()
    {
        weyl_.set(
            ARSWeylConstantTrait<0>::value, ARSWeylConstantTrait<1>::value);
    }

    void key(const key_type &k) { key_.load(k.data()); }

    key_type key() const
    {
        key_type k;
        key_.store(k.data());

        return k;
    }

    template <std::size_t Rp1>
    void generate(std::array<M128I<>, Rp1> &rk) const
    {
        std::get<0>(rk).value() = key_.value();
        generate<1>(rk, std::integral_constant<bool, 1 < Rp1>());
    }

    friend bool operator==(
        const ARSKeySeq<ResultType> &ks1, const ARSKeySeq<ResultType> &ks2)
    {
        if (ks1.round_key_ != ks2.round_key_)
            return false;
        if (ks1.weyl_ != ks2.weyl_)
            return false;
        if (ks1.key_ != ks2.key_)
            return false;
        return true;
    }

    friend bool operator!=(
        const ARSKeySeq<ResultType> &ks1, const ARSKeySeq<ResultType> &ks2)
    {
        return !(ks1 == ks2);
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os, const ARSKeySeq<ResultType> &ks)
    {
        if (!os.good())
            return os;

        os << ks.round_key_ << ' ';
        os << ks.weyl_ << ' ';
        os << ks.key_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is, ARSKeySeq<ResultType> &ks)
    {
        if (!is.good())
            return is;

        ARSKeySeq<ResultType> ks_tmp;
        is >> std::ws >> ks_tmp.round_key_;
        is >> std::ws >> ks_tmp.weyl_;
        is >> std::ws >> ks_tmp.key_;

        if (is.good())
            ks = std::move(ks_tmp);

        return is;
    }

    private:
    M128I<std::uint64_t> weyl_;
    M128I<std::uint64_t> key_;

    template <std::size_t, std::size_t Rp1>
    void generate(std::array<M128I<>, Rp1> &, std::false_type) const
    {
    }

    template <std::size_t N, std::size_t Rp1>
    void generate(std::array<M128I<>, Rp1> &rk, std::true_type) const
    {
        std::get<N>(rk) =
            _mm_add_epi64(std::get<N - 1>(rk).value(), weyl_.value());
        generate<N + 1>(rk, std::integral_constant<bool, N + 1 < Rp1>());
    }
}; // class ARSKeySeq

/// \brief ARS RNG engine
/// \ingroup AESNIRNG
template <typename ResultType, std::size_t Rounds = VSMC_RNG_ARS_ROUNDS,
    std::size_t Blocks = VSMC_RNG_ARS_BLOCKS>
using ARSEngine =
    AESNIEngine<ResultType, ARSKeySeq<ResultType>, Rounds, Blocks>;

/// \brief ARS RNG engine with 32-bits integers output, 1 block and default
/// rounds
/// \ingroup AESNIRNG
using ARS_1x32 = ARSEngine<std::uint32_t, VSMC_RNG_ARS_ROUNDS, 1>;

/// \brief ARS RNG engine with 32-bits integers output, 2 blocks and default
/// rounds
/// \ingroup AESNIRNG
using ARS_2x32 = ARSEngine<std::uint32_t, VSMC_RNG_ARS_ROUNDS, 2>;

/// \brief ARS RNG engine with 32-bits integers output, 4 blocks and default
/// rounds
/// \ingroup AESNIRNG
using ARS_4x32 = ARSEngine<std::uint32_t, VSMC_RNG_ARS_ROUNDS, 4>;

/// \brief ARS RNG engine with 32-bits integers output, 8 blocks and default
/// rounds
/// \ingroup AESNIRNG
using ARS_8x32 = ARSEngine<std::uint32_t, VSMC_RNG_ARS_ROUNDS, 8>;

/// \brief ARS RNG engine with 64-bits integers output, 1 block and default
/// rounds
/// \ingroup AESNIRNG
using ARS_1x64 = ARSEngine<std::uint64_t, VSMC_RNG_ARS_ROUNDS, 1>;

/// \brief ARS RNG engine with 64-bits integers output, 2 blocks and default
/// rounds
/// \ingroup AESNIRNG
using ARS_2x64 = ARSEngine<std::uint64_t, VSMC_RNG_ARS_ROUNDS, 2>;

/// \brief ARS RNG engine with 64-bits integers output, 4 blocks and default
/// rounds
/// \ingroup AESNIRNG
using ARS_4x64 = ARSEngine<std::uint64_t, VSMC_RNG_ARS_ROUNDS, 4>;

/// \brief ARS RNG engine with 64-bits integers output, 8 blocks and default
/// rounds
/// \ingroup AESNIRNG
using ARS_8x64 = ARSEngine<std::uint64_t, VSMC_RNG_ARS_ROUNDS, 8>;

/// \brief ARS RNG engine with 32-bits integers output, default blocks and
/// default rounds
/// \ingroup AESNIRNG
using ARS = ARS_4x32;

/// \brief ARS RNG engine with 64-bits integers output, default blocks and
/// default rounds
/// \ingroup AESNIRNG
using ARS_64 = ARS_4x64;

} // namespace vsmc

#endif // VSMC_RNG_ARS_HPP
