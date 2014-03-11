#ifndef VSMC_RNG_AES_HPP
#define VSMC_RNG_AES_HPP

#include <vsmc/rng/aes_ni.hpp>

#define VSMC_DEFINE_RNG_AES_ROUND_CONSTANT(N, val) \
    template <> struct AESRoundConstantValue< N > :                          \
        public cxx11::integral_constant<int, val > {};

/// \brief AESEngine default blocks
/// \ingroup Config
#ifndef VSMC_RNG_AES_BLOCKS
#define VSMC_RNG_AES_BLOCKS 1
#endif

namespace vsmc {

namespace internal {

template <std::size_t N> struct AESRoundConstantValue;

VSMC_DEFINE_RNG_AES_ROUND_CONSTANT(0x00, 0x8D)
VSMC_DEFINE_RNG_AES_ROUND_CONSTANT(0x01, 0x01)
VSMC_DEFINE_RNG_AES_ROUND_CONSTANT(0x02, 0x02)
VSMC_DEFINE_RNG_AES_ROUND_CONSTANT(0x03, 0x04)
VSMC_DEFINE_RNG_AES_ROUND_CONSTANT(0x04, 0x08)
VSMC_DEFINE_RNG_AES_ROUND_CONSTANT(0x05, 0x10)
VSMC_DEFINE_RNG_AES_ROUND_CONSTANT(0x06, 0x20)
VSMC_DEFINE_RNG_AES_ROUND_CONSTANT(0x07, 0x40)
VSMC_DEFINE_RNG_AES_ROUND_CONSTANT(0x08, 0x80)
VSMC_DEFINE_RNG_AES_ROUND_CONSTANT(0x09, 0x1B)
VSMC_DEFINE_RNG_AES_ROUND_CONSTANT(0x0A, 0x36)

} // namespace vsmc::internal

namespace traits {

/// \brief AESEngine round constant traits
/// \ingroup Traits
///
/// The specialization for `N = 0` to `N = 9` are used as the ten round
/// constants in AESEngine
template <std::size_t N>
struct AESRoundConstantTrait :
    public ::vsmc::internal::AESRoundConstantValue<N> {};

} // namespace traits

/// \brief AES128Engine key sequence generator
/// \ingroup R123RNG
class AES128KeySeq
{
    public :

    AES128KeySeq () : tmp0_(), tmp1_(), tmp2_() {}

    template <typename T, std::size_t K, typename KeyTraits,
             std::size_t Rp1, typename Traits>
    void generate (const StaticVector<T, K, KeyTraits> &key,
            StaticVector<__m128i, Rp1, Traits> &key_seq)
    {
        m128i_pack<0>(key, tmp0_);
        key_seq.front() = tmp0_;
        generate_seq<1>(key_seq, cxx11::integral_constant<bool, 1 < Rp1>());
    }

    private :

    __m128i tmp0_;
    __m128i tmp1_;
    __m128i tmp2_;

    template <std::size_t, std::size_t Rp1, typename Traits>
    void generate_seq (StaticVector<__m128i, Rp1, Traits> &,
            cxx11::false_type) {}

    template <std::size_t N, std::size_t Rp1, typename Traits>
    void generate_seq (StaticVector<__m128i, Rp1, Traits> &key_seq,
            cxx11::true_type)
    {
        tmp1_ = _mm_aeskeygenassist_si128(tmp0_,
                traits::AESRoundConstantTrait<N>::value);
        generate_assit();
        key_seq[Position<N>()] = tmp0_;
        generate_seq<N + 1>(key_seq,
                cxx11::integral_constant<bool, N + 1 < Rp1>());
    }

    void generate_assit ()
    {
        tmp1_ = _mm_shuffle_epi32 (tmp1_ ,0xFF);
        tmp2_ = _mm_slli_si128    (tmp0_, 0x04);
        tmp0_ = _mm_xor_si128     (tmp0_, tmp2_);
        tmp2_ = _mm_slli_si128    (tmp2_, 0x04);
        tmp0_ = _mm_xor_si128     (tmp0_, tmp2_);
        tmp2_ = _mm_slli_si128    (tmp2_, 0x04);
        tmp0_ = _mm_xor_si128     (tmp0_, tmp2_);
        tmp0_ = _mm_xor_si128     (tmp0_, tmp1_);
    }
}; // class AES128KeySeq

/// \brief AES-128 RNG engine reimplemented
/// \ingroup AESNIRNG
///
/// \details
/// This is a reimplementation of the algorithm AESNI engine as described in
/// [Parallel Random Numbers: As Easy as 1, 2, 3][r123paper] and implemented in
/// [Random123][r123lib].
///
/// [r123paper]:http://sc11.supercomputing.org/schedule/event_detail.php?evid=pap274
/// [r123lib]: https://www.deshawresearch.com/resources_random123.html
///
/// \sa AES128KeySeq.
template <typename ResultType, std::size_t Blocks = VSMC_RNG_AES_BLOCKS>
class AES128Engine : public AESNIEngine<ResultType, AES128KeySeq, 10, Blocks>
{
    typedef AESNIEngine<ResultType, AES128KeySeq, 10, Blocks> base;

    public :

    explicit AES128Engine (ResultType s = 0) : base(s) {}

    template <typename SeedSeq>
    explicit AES128Engine (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_seq<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR) : base(seq) {}
}; // class AES128Engine

/// \brief AES-128 RNG engine returning 32-bits integers with default blocks
/// \ingroup R123RNG
typedef AES128Engine<uint32_t> AES128_32;

/// \brief AES-128 RNG engine returning 64-bits integers with default blocks
/// \ingroup R123RNG
typedef AES128Engine<uint64_t> AES128_64;

} // namespace vsmc

#endif // VSMC_RNG_AES_HPP
