#ifndef VSMC_RNG_AES_HPP
#define VSMC_RNG_AES_HPP

#include <vsmc/rng/ars.hpp>

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

VSMC_DEFINE_RNG_AES_ROUND_CONSTANT(0, 0x01)
VSMC_DEFINE_RNG_AES_ROUND_CONSTANT(1, 0x02)
VSMC_DEFINE_RNG_AES_ROUND_CONSTANT(2, 0x04)
VSMC_DEFINE_RNG_AES_ROUND_CONSTANT(3, 0x08)
VSMC_DEFINE_RNG_AES_ROUND_CONSTANT(4, 0x10)
VSMC_DEFINE_RNG_AES_ROUND_CONSTANT(5, 0x20)
VSMC_DEFINE_RNG_AES_ROUND_CONSTANT(6, 0x40)
VSMC_DEFINE_RNG_AES_ROUND_CONSTANT(7, 0x80)
VSMC_DEFINE_RNG_AES_ROUND_CONSTANT(8, 0x1B)
VSMC_DEFINE_RNG_AES_ROUND_CONSTANT(9, 0x36)

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

/// \brief AESEngine key sequence generator
/// \ingroup R123RNG
template <std::size_t R>
class AESKeySeq
{
    public :

    typedef StaticVector<__m128i, R + 1> key_seq_type;

    AESKeySeq () : tmp0_(), tmp1_(), tmp2_() {}

    void generate (const __m128i &ukey, key_seq_type &key_seq)
    {
        tmp0_ = ukey;
        generate<0>(key_seq, cxx11::true_type());
    }

    private :

    __m128i tmp0_;
    __m128i tmp1_;
    __m128i tmp2_;

    template <std::size_t>
    void generate (key_seq_type &key_seq, cxx11::false_type)
    {key_seq.back() = tmp0_;}

    template <std::size_t N>
    void generate (key_seq_type &key_seq, cxx11::true_type)
    {
        key_seq[Position<N>()] = tmp0_;
        tmp1_ = _mm_aeskeygenassist_si128(tmp0_,
                traits::AESRoundConstantTrait<N>::value);
        generate_assit();
        generate<N + 1>(key_seq, cxx11::integral_constant<bool, N + 1 < R>());
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
}; // class AESKeySeq

/// \brief AES RNG engine reimplemented
/// \ingroup R123RNG
///
/// \details
/// This is a reimplementation of the algorithm AES as described in [Parallel
/// Random Numbers: As Easy as 1, 2, 3][r123paper] and implemented in
/// [Random123][r123lib].
///
/// [r123paper]:http://sc11.supercomputing.org/schedule/event_detail.php?evid=pap274
/// [r123lib]: https://www.deshawresearch.com/resources_random123.html
///
/// The algorithm is almost identical to the original. Compared to
/// `r123:Engine<r123::AESNI4x32>`, when using the default constructor or the
/// one with a single seed, the output shall be exactly the same for the first
/// \f$2^{32}\f$ iterations. Further iterations may produce different results,
/// as vSMC increment the counter slightly differently, but it still cover the
/// same range and has the same period as the original.
///
/// The implementation however is much different the original. See the source
/// for how the ARSEngine is used as the base of AESEngine, and how to
/// implement similar engines.
///
/// The second template argument specify the number of blocks
///
/// \sa AESKeySeq.
/// \sa ARSEngine.
template <typename ResultType, std::size_t Blocks = VSMC_RNG_AES_BLOCKS>
class AESEngine : public ARSEngine<ResultType, Blocks, 10, AESKeySeq<10> >
{
    typedef ARSEngine<ResultType, Blocks, 10, AESKeySeq<10> > base;

    public :

    explicit AESEngine (ResultType s = 0) : base(s) {}

    template <typename SeedSeq>
    explicit AESEngine (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_seq<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR) : base(seq) {}
}; // class AESEngine

/// \brief AES RNG engine returning 32-bits integers with default blocks
/// \ingroup R123RNG
typedef AESEngine<uint32_t> AES4x32;

/// \brief AES RNG engine returning 64-bits integers with default blocks
/// \ingroup R123RNG
typedef AESEngine<uint64_t> AES2x64;

/// \brief AES RNG engine returning 128-bits integers with default blocks
/// \ingroup R123RNG
typedef AESEngine<__m128i>  AES1x128;

/// \brief AES RNG engine returning 32-bits integers with 1 block
/// \ingroup R123RNG
typedef AESEngine<uint32_t, 1> AES4x32_1;

/// \brief AES RNG engine returning 64-bits integers with 1 block
/// \ingroup R123RNG
typedef AESEngine<uint64_t, 1> AES2x64_1;

/// \brief AES RNG engine returning 128-bits integers with 1 block
/// \ingroup R123RNG
typedef AESEngine<__m128i, 1>  AES1x128_1;

/// \brief AES RNG engine returning 32-bits integers with 2 block
/// \ingroup R123RNG
typedef AESEngine<uint32_t, 2> AES4x32_2;

/// \brief AES RNG engine returning 64-bits integers with 2 block
/// \ingroup R123RNG
typedef AESEngine<uint64_t, 2> AES2x64_2;

/// \brief AES RNG engine returning 128-bits integers with 2 block
/// \ingroup R123RNG
typedef AESEngine<__m128i, 2>  AES1x128_2;

/// \brief AES RNG engine returning 32-bits integers with 4 block
/// \ingroup R123RNG
typedef AESEngine<uint32_t, 4> AES4x32_4;

/// \brief AES RNG engine returning 64-bits integers with 4 block
/// \ingroup R123RNG
typedef AESEngine<uint64_t, 4> AES2x64_4;

/// \brief AES RNG engine returning 128-bits integers with 4 block
/// \ingroup R123RNG
typedef AESEngine<__m128i, 4>  AES1x128_4;

/// \brief AES RNG engine returning 32-bits integers with 8 block
/// \ingroup R123RNG
typedef AESEngine<uint32_t, 8> AES4x32_8;

/// \brief AES RNG engine returning 64-bits integers with 8 block
/// \ingroup R123RNG
typedef AESEngine<uint64_t, 8> AES2x64_8;

/// \brief AES RNG engine returning 128-bits integers with 8 block
/// \ingroup R123RNG
typedef AESEngine<__m128i, 8>  AES1x128_8;

} // namespace vsmc

#endif // VSMC_RNG_AES_HPP
