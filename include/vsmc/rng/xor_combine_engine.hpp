//============================================================================
// vSMC/include/vsmc/rng/xor_combine_engine.hpp
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

#ifndef VSMC_RNG_COMBINE_HPP
#define VSMC_RNG_COMBINE_HPP

#include <vsmc/rng/internal/common.hpp>

#define VSMC_STATIC_ASSERT_RNG_XOR_COMBINE_UNSIGNED(result_type)              \
    VSMC_STATIC_ASSERT((std::is_unsigned<result_type>::value),                \
        "**XorCombineEngine** USED WITH RNG ENGINES HAVNG result_type NOT "   \
        "AN UNSIGNED INTEGER TYPE")

#define VSMC_STATIC_ASSERT_RNG_XOR_COMBINE_SAME_TYPE(Eng1, Eng2)              \
    VSMC_STATIC_ASSERT((std::is_same<typename Eng1::resultType,               \
                           typename Eng2::resultType>::value),                \
        "**XorCombneEngine** USED WITH TWO RNG ENGINES WITH DIFFERENT "       \
        "result_type")

#define VSMC_STATIC_ASSERT_RNG_XOR_COMBINE                                    \
    VSMC_STATIC_ASSERT_RNG_XOR_COMBINE_UNSIGNED(result_type);                 \
    VSMC_STATIC_ASSERT_RNG_XOR_COMBINE_SAME_TYPE(Eng1, Eng2);

namespace vsmc
{

/// \brief Combine two RNG engines using XOR
/// \ingroup RNGAdapter
template <typename Eng1, typename Eng2, typename Eng1::result_type S1 = 0,
    typename Eng2::result_type S2 = 0>
class XorCombineEngine
{
    public:
    typedef typename Eng1::result_type result_type;
    typedef Eng1 engine1_type;
    typedef Eng2 engine2_type;

    explicit XorCombineEngine(result_type s = 1) : eng1_(s), eng2_(s)
    {
        VSMC_STATIC_ASSERT_RNG_XOR_COMBINE;
    }

    template <typename SeedSeq>
    explicit XorCombineEngine(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, result_type,
            XorCombineEngine<Eng1, Eng2, S1, S2>>::value>::type * = nullptr)
        : eng1_(seq), eng2_(seq)
    {
        VSMC_STATIC_ASSERT_RNG_XOR_COMBINE;
    }

    void seed(result_type s)
    {
        eng1_.seed(s);
        eng2_.seed(s);
    }

    template <typename SeedSeq>
    void seed(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, result_type,
            XorCombineEngine<Eng1, Eng2, S1, S2>>::value>::type * = nullptr)
    {
        eng1_.seed(seq);
        eng2_.seed(seq);
    }

    engine1_type &eng1() { return eng1_; }

    engine2_type &eng2() { return eng2_; }

    static constexpr result_type min VSMC_MNE()
    {
        return Eng1::min VSMC_MNE() < Eng2::min VSMC_MNE() ?
            Eng1::min VSMC_MNE :
            Eng2::min
            VSMC_MNE();
    }

    static constexpr result_type max VSMC_MNE()
    {
        return Eng1::max VSMC_MNE() > Eng2::max VSMC_MNE() ?
            Eng1::max VSMC_MNE :
            Eng2::max
            VSMC_MNE();
    }

    result_type operator()() { return (eng1_() << S1) ^ (eng2_() << S2); }

    void discard(std::size_t nskip)
    {
        for (std::size_t i = 0; i != nskip; ++i)
            operator()();
    }

    friend inline bool operator==(
        const XorCombineEngine<Eng1, Eng2, S1, S2> &eng1,
        const XorCombineEngine<Eng1, Eng2, S1, S2> &eng2)
    {
        return (eng1.eng1_ == eng2.eng1_ && eng1.eng2_ == eng2.eng2_);
    }

    friend inline bool operator!=(
        const XorCombineEngine<Eng1, Eng2, S1, S2> &eng1,
        const XorCombineEngine<Eng1, Eng2, S1, S2> &eng2)
    {
        return !(eng1 == eng2);
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os,
        const XorCombineEngine<Eng1, Eng2, S1, S2> &eng)
    {
        if (!os.good())
            return os;

        os << eng.eng1_ << ' ' << eng.eng2_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is,
        XorCombineEngine<Eng1, Eng2, S1, S2> &eng)
    {
        if (!is.good())
            return is;

        engine1_type eng1_tmp;
        engine2_type eng2_tmp;
        is >> std::ws >> eng1_tmp;
        is >> std::ws >> eng2_tmp;

        if (is.good()) {
            eng.eng1_ = std::move(eng1_tmp);
            eng.eng2_ = std::move(eng2_tmp);
        }

        return is;
    }

    private:
    engine1_type eng1_;
    engine2_type eng2_;
}; // class XorCombineEngine

} // namespace vsmc

#endif // VSMC_RNG_COMBINE_HPP
