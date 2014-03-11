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

struct AESKeyInit
{
    template <std::size_t N,
             typename T, std::size_t KeySize, typename KeyTraits,
             std::size_t K, typename Traits>
    static void key_init (const StaticVector<T, KeySize, KeyTraits> &key,
            StaticVector<__m128i, K, Traits> &key_seq, __m128i &xmm)
    {
        key_init_xmm<N>(key, key_seq, xmm,
                cxx11::integral_constant<bool, 2 * N < KeySize && N < K>());
    }

    private :

    template <std::size_t,
             typename T, std::size_t KeySize, typename KeyTraits,
             std::size_t K, typename Traits>
    static void key_init_xmm (const StaticVector<T, KeySize, KeyTraits> &,
            StaticVector<__m128i, K, Traits> &,
            __m128i &, cxx11::false_type) {}

    template <std::size_t N,
             typename T, std::size_t KeySize, typename KeyTraits,
             std::size_t K, typename Traits>
    static void key_init_xmm (const StaticVector<T, KeySize, KeyTraits> &key,
            StaticVector<__m128i, K, Traits> &key_seq,
            __m128i &xmm, cxx11::true_type)
    {
        m128i_pack<2 * N>(key, xmm);
        key_seq[Position<N>()] = xmm;
        StaticVector<char, 16> buf;
        m128i_unpack<0>(xmm, buf);
    }
}; // struct AESKeyInit

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
/// \ingroup AESNIRNG
class AES128KeySeq
{
    public :

    typedef StaticVector<uint64_t, 2> key_type;

    AES128KeySeq () : xmm1_(), xmm2_(), xmm3_() {}

    template <std::size_t K, typename Traits>
    void generate (const key_type &key,
            StaticVector<__m128i, K, Traits> &key_seq)
    {
        internal::AESKeyInit::key_init<0>(key, key_seq, xmm1_);
        generate_seq<1>(key_seq, cxx11::integral_constant<bool, 1 < K>());
    }

    private :

    __m128i xmm1_;
    __m128i xmm2_;
    __m128i xmm3_;

    template <std::size_t, std::size_t K, typename Traits>
    void generate_seq (StaticVector<__m128i, K, Traits> &,
            cxx11::false_type) {}

    template <std::size_t N, std::size_t K, typename Traits>
    void generate_seq (StaticVector<__m128i, K, Traits> &key_seq,
            cxx11::true_type)
    {
        xmm2_ = _mm_aeskeygenassist_si128(xmm1_,
                traits::AESRoundConstantTrait<N>::value);
        key_expansion();
        key_seq[Position<N>()] = xmm1_;
        generate_seq<N + 1>(key_seq,
                cxx11::integral_constant<bool, N + 1 < K>());
    }

    void key_expansion ()
    {
        xmm2_ = _mm_shuffle_epi32 (xmm2_, 0xFF);  // pshufd xmm2, xmm2, 0xFF
        xmm3_ = _mm_slli_si128    (xmm1_, 0x04);  // pshufb xmm3, xmm5
        xmm1_ = _mm_xor_si128     (xmm1_, xmm3_); // pxor   xmm1, xmm3
        xmm3_ = _mm_slli_si128    (xmm3_, 0x04);  // pshfb  xmm3, xmm5
        xmm1_ = _mm_xor_si128     (xmm1_, xmm3_); // pxor   xmm1, xmm3
        xmm3_ = _mm_slli_si128    (xmm3_, 0x04);  // pshfb  xmm3, xmm5
        xmm1_ = _mm_xor_si128     (xmm1_, xmm3_); // pxor   xmm1, xmm3
        xmm1_ = _mm_xor_si128     (xmm1_, xmm2_); // pxor   xmm1, xmm2
    }
}; // class AES128KeySeq

/// \brief AES256Engine key sequence generator
/// \ingroup AESNIRNG
class AES256KeySeq : public AES128KeySeq
{
    public :

    typedef StaticVector<uint64_t, 4> key_type;

    AES256KeySeq () : xmm1_(), xmm2_(), xmm3_(), xmm4_() {}

    template <std::size_t K, typename Traits>
    void generate (const key_type &key,
            StaticVector<__m128i, K, Traits> &key_seq)
    {
        internal::AESKeyInit::key_init<0>(key, key_seq, xmm1_);
        internal::AESKeyInit::key_init<1>(key, key_seq, xmm3_);
        generate_seq<2>(key_seq, cxx11::integral_constant<bool, 2 < K>());
    }

    private :

    __m128i xmm1_;
    __m128i xmm2_;
    __m128i xmm3_;
    __m128i xmm4_;

    template <std::size_t, std::size_t K, typename Traits>
    void generate_seq (StaticVector<__m128i, K, Traits> &,
            cxx11::false_type) {}

    template <std::size_t N, std::size_t K, typename Traits>
    void generate_seq (StaticVector<__m128i, K, Traits> &key_seq,
            cxx11::true_type)
    {
        key_gen<N>(key_seq, cxx11::integral_constant<bool, N % 2 == 0>());
        generate_seq<N + 1>(key_seq,
                cxx11::integral_constant<bool, N + 1 < K>());
    }

    template <std::size_t N, std::size_t K, typename Traits>
    void key_gen (StaticVector<__m128i, K, Traits> &key_seq, cxx11::true_type)
    {
        xmm2_ = _mm_aeskeygenassist_si128(xmm3_,
                traits::AESRoundConstantTrait<N / 2>::value);
        key_expansion(cxx11::true_type());
        key_seq[Position<N>()] = xmm1_;
    }

    template <std::size_t N, std::size_t K, typename Traits>
    void key_gen (StaticVector<__m128i, K, Traits> &key_seq, cxx11::false_type)
    {
        xmm4_ = _mm_aeskeygenassist_si128(xmm1_, 0);
        key_expansion(cxx11::false_type());
        key_seq[Position<N>()] = xmm3_;
    }

    void key_expansion (cxx11::true_type)
    {
        xmm2_ = _mm_shuffle_epi32 (xmm2_, 0xFF);  // pshufd xmm2, xmm2, 0xFF
        xmm4_ = _mm_slli_si128    (xmm1_, 0x04);  // pshufb xmm4, xmm5
        xmm1_ = _mm_xor_si128     (xmm1_, xmm4_); // pxor   xmm1, xmm4
        xmm4_ = _mm_slli_si128    (xmm4_, 0x04);  // pshfb  xmm4, xmm5
        xmm1_ = _mm_xor_si128     (xmm1_, xmm4_); // pxor   xmm1, xmm4
        xmm4_ = _mm_slli_si128    (xmm4_, 0x04);  // pshfb  xmm4, xmm5
        xmm1_ = _mm_xor_si128     (xmm1_, xmm4_); // pxor   xmm1, xmm4
        xmm1_ = _mm_xor_si128     (xmm1_, xmm2_); // pxor   xmm1, xmm2
    }

    void key_expansion (cxx11::false_type)
    {
        xmm2_ = _mm_shuffle_epi32 (xmm4_, 0xAA);  // pshufd xmm2, xmm4, 0xAA
        xmm4_ = _mm_slli_si128    (xmm3_, 0x04);  // pshufb xmm4, xmm5
        xmm3_ = _mm_xor_si128     (xmm3_, xmm4_); // pxor   xmm3, xmm4
        xmm4_ = _mm_slli_si128    (xmm4_, 0x04);  // pshfb  xmm4, xmm5
        xmm3_ = _mm_xor_si128     (xmm3_, xmm4_); // pxor   xmm3, xmm4
        xmm4_ = _mm_slli_si128    (xmm4_, 0x04);  // pshfb  xmm4, xmm5
        xmm3_ = _mm_xor_si128     (xmm3_, xmm4_); // pxor   xmm3, xmm4
        xmm3_ = _mm_xor_si128     (xmm3_, xmm2_); // pxor   xmm1, xmm2
    }
}; // class AESKey256

/// \brief AES-128 RNG engine
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
/// \sa AES128KeySeq
/// \sa AESNIEngine
template <typename ResultType, std::size_t Blocks = VSMC_RNG_AES_BLOCKS>
class AES128Engine :
    public AESNIEngine<ResultType, AES128KeySeq, true, 10, Blocks>
{

    public :

    typedef AESNIEngine<ResultType, AES128KeySeq, true, 10, Blocks>
        base_eng_type;

    explicit AES128Engine (ResultType s = 0) : base_eng_type(s) {}

    template <typename SeedSeq>
    explicit AES128Engine (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_seq<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR) : base_eng_type(seq) {}

    AES128Engine (const typename base_eng_type::ctr_type &c,
            const typename base_eng_type::key_type &k) : base_eng_type(c, k) {}
}; // class AES128Engine

/// \brief AES-128 RNG engine with 32-bits integers output and default blocks
/// \ingroup AESNIRNG
typedef AES128Engine<uint32_t> AES128_32;

/// \brief AES-128 RNG engine with 64-bits integers output and default blocks
/// \ingroup AESNIRNG
typedef AES128Engine<uint64_t> AES128_64;

/// \brief AES-256 RNG engine
/// \ingroup AESNIRNG
///
/// \sa AES256KeySeq
/// \sa AESNIEngine
template <typename ResultType, std::size_t Blocks = VSMC_RNG_AES_BLOCKS>
class AES256Engine :
    public AESNIEngine<ResultType, AES256KeySeq, true, 14, Blocks>
{

    public :

    typedef AESNIEngine<ResultType, AES256KeySeq, true, 14, Blocks>
        base_eng_type;

    explicit AES256Engine (ResultType s = 0) : base_eng_type(s) {}

    template <typename SeedSeq>
    explicit AES256Engine (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_seq<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR) : base_eng_type(seq) {}

    AES256Engine (const typename base_eng_type::ctr_type &c,
            const typename base_eng_type::key_type &k) : base_eng_type(c, k) {}
}; // class AES256Engine

/// \brief AES-128 RNG engine with 32-bits integers output and default blocks
/// \ingroup AESNIRNG
typedef AES128Engine<uint32_t> AES128_32;

/// \brief AES-128 RNG engine with 64-bits integers output and default blocks
/// \ingroup AESNIRNG
typedef AES128Engine<uint64_t> AES128_64;

} // namespace vsmc

#endif // VSMC_RNG_AES_HPP
