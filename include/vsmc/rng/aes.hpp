#ifndef VSMC_RNG_AES_HPP
#define VSMC_RNG_AES_HPP

#include <vsmc/rng/aes_ni.hpp>

#define VSMC_DEFINE_RNG_AES_ROUND_CONSTANT(N, val) \
    template <> struct AESRoundConstant< N > :                               \
        public cxx11::integral_constant<int, val > {};

/// \brief AESEngine default blocks
/// \ingroup Config
#ifndef VSMC_RNG_AES_BLOCKS
#define VSMC_RNG_AES_BLOCKS 1
#endif

namespace vsmc {

namespace internal {

template <std::size_t N> struct AESRoundConstant;

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
    template <std::size_t Offset, std::size_t N,
             typename T, std::size_t KeySize, typename KeyTraits,
             std::size_t Rp1, typename Traits>
    static void key_init (const StaticVector<T, KeySize, KeyTraits> &key,
            StaticVector<__m128i, Rp1, Traits> &key_seq, __m128i &xmm)
    {
        key_init_xmm<Offset, N>(key, key_seq, xmm,
                cxx11::integral_constant<bool, N < Rp1>());
    }

    private :

    template <std::size_t, std::size_t,
             typename T, std::size_t KeySize, typename KeyTraits,
             std::size_t Rp1, typename Traits>
    static void key_init_xmm (const StaticVector<T, KeySize, KeyTraits> &,
            StaticVector<__m128i, Rp1, Traits> &,
            __m128i &, cxx11::false_type) {}

    template <std::size_t Offset, std::size_t N,
             typename T, std::size_t KeySize, typename KeyTraits,
             std::size_t Rp1, typename Traits>
    static void key_init_xmm (const StaticVector<T, KeySize, KeyTraits> &key,
            StaticVector<__m128i, Rp1, Traits> &key_seq,
            __m128i &xmm, cxx11::true_type)
    {
        m128i_pack<Offset>(key, xmm);
        key_seq[Position<N>()] = xmm;
    }
}; // struct AESKeyInit

} // namespace vsmc::internal

/// \brief AES128Engine key sequence generator
/// \ingroup AESNIRNG
template <typename ResultType>
class AES128KeySeq
{
    public :

    typedef StaticVector<ResultType, 16 / sizeof(ResultType)> key_type;

    AES128KeySeq () : xmm1_(), xmm2_(), xmm3_() {}

    template <std::size_t Rp1, typename Traits>
    void generate (const key_type &key,
            StaticVector<__m128i, Rp1, Traits> &key_seq)
    {
        internal::AESKeyInit::key_init<0, 0>(key, key_seq, xmm1_);
        generate_seq<1>(key_seq, cxx11::integral_constant<bool, 1 < Rp1>());
    }

    private :

    __m128i xmm1_;
    __m128i xmm2_;
    __m128i xmm3_;

    template <std::size_t, std::size_t Rp1, typename Traits>
    void generate_seq (StaticVector<__m128i, Rp1, Traits> &,
            cxx11::false_type) {}

    template <std::size_t N, std::size_t Rp1, typename Traits>
    void generate_seq (StaticVector<__m128i, Rp1, Traits> &key_seq,
            cxx11::true_type)
    {
        xmm2_ = _mm_aeskeygenassist_si128(xmm1_,
                internal::AESRoundConstant<N>::value);
        expand_key();
        key_seq[Position<N>()] = xmm1_;
        generate_seq<N + 1>(key_seq,
                cxx11::integral_constant<bool, N + 1 < Rp1>());
    }

    void expand_key ()
    {
        xmm2_ = _mm_shuffle_epi32(xmm2_, 0xFF);  // pshufd xmm2, xmm2, 0xFF
        xmm3_ = _mm_slli_si128   (xmm1_, 0x04);  // pshufb xmm3, xmm5
        xmm1_ = _mm_xor_si128    (xmm1_, xmm3_); // pxor   xmm1, xmm3
        xmm3_ = _mm_slli_si128   (xmm3_, 0x04);  // pslldq xmm3, 0x04
        xmm1_ = _mm_xor_si128    (xmm1_, xmm3_); // pxor   xmm1, xmm3
        xmm3_ = _mm_slli_si128   (xmm3_, 0x04);  // pslldq xmm3, 0x04
        xmm1_ = _mm_xor_si128    (xmm1_, xmm3_); // pxor   xmm1, xmm3
        xmm1_ = _mm_xor_si128    (xmm1_, xmm2_); // pxor   xmm1, xmm2
    }
}; // class AES128KeySeq

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
    public AESNIEngine<ResultType, AES128KeySeq<ResultType>, true, 10, Blocks>
{

    public :

    typedef AESNIEngine<ResultType, AES128KeySeq<ResultType>, true, 10, Blocks>
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

/// \brief AES192Engine key sequence generator
/// \ingroup AESNIRNG
template <typename ResultType>
class AES192KeySeq
{
    public :

    typedef StaticVector<ResultType, 24 / sizeof(ResultType)> key_type;

    AES192KeySeq () :
        xmm1_(), xmm2_(), xmm3_(), xmm4_(), xmm5_(), xmm6_(), xmm7_() {}

    template <std::size_t Rp1, typename Traits>
    void generate (const key_type &key,
            StaticVector<__m128i, Rp1, Traits> &key_seq)
    {
        StaticVector<uint64_t, 3> key_tmp;
        std::memcpy(key_tmp.data(), key.data(), 24);
        internal::AESKeyInit::key_init<0, 0>(key_tmp, key_seq, xmm1_);
        key_tmp.at<0>() = key_tmp.at<2>();
        key_tmp.at<1>() = 0;
        internal::AESKeyInit::key_init<0, 1>(key_tmp, key_seq, xmm7_);

        xmm3_ = _mm_setzero_si128();
        xmm6_ = _mm_setzero_si128();
        xmm4_ = _mm_shuffle_epi32(xmm7_, 0x4F);  // pshufd xmm4, xmm7, 0x4F

        StaticVector<unsigned char, Rp1 * 16 + 16> ks_tmp;
        generate_seq<1, Rp1>(ks_tmp.data(),
                cxx11::integral_constant<bool, 24 < Rp1 * 16>());
        copy_key(key_seq, ks_tmp.data(),
                cxx11::integral_constant<bool, 24 < Rp1 * 16>());
    }

    private :

    __m128i xmm1_;
    __m128i xmm2_;
    __m128i xmm3_;
    __m128i xmm4_;
    __m128i xmm5_;
    __m128i xmm6_;
    __m128i xmm7_;

    template <std::size_t, std::size_t>
    void generate_seq (unsigned char *, cxx11::false_type) {}

    template <std::size_t N, std::size_t Rp1>
    void generate_seq (unsigned char *ks_ptr, cxx11::true_type)
    {
        generate_key<N>(ks_ptr);
        complete_key<N>(ks_ptr,
                cxx11::integral_constant<bool, N * 24 + 16 < Rp1 * 16>());
        generate_seq<N + 1, Rp1>(ks_ptr,
                cxx11::integral_constant<bool, N * 24 + 24 < Rp1 * 16>());
    }

    template <std::size_t N>
    void generate_key (unsigned char *ks_ptr)
    {
        // In entry, N * 24 < Rp1 * 16
        // Required Storage: N * 24 + 16;

        xmm2_ = _mm_aeskeygenassist_si128(xmm4_,
                internal::AESRoundConstant<N>::value);
        generate_key_expansion();
        _mm_storeu_si128(reinterpret_cast<__m128i *>(
                    ks_ptr + N * 24), xmm1_);
    }

    template <std::size_t>
    void complete_key (unsigned char *, cxx11::false_type) {}

    template <std::size_t N>
    void complete_key (unsigned char *ks_ptr, cxx11::true_type)
    {
        // In entry, N * 24 + 16 < Rp1 * 16
        // Required storage: N * 24 + 32

        complete_key_expansion();
        _mm_storeu_si128(reinterpret_cast<__m128i *>(
                    ks_ptr + N * 24 + 16), xmm7_);
    }

    void generate_key_expansion ()
    {
        xmm2_ = _mm_shuffle_epi32(xmm2_, 0xFF);        // pshufd xmm2, xmm2, 0xFF
        xmm3_ = _mm_castps_si128 (_mm_shuffle_ps(      // shufps xmm3, xmm1, 0x10
                    _mm_castsi128_ps(xmm3_),
                    _mm_castsi128_ps(xmm1_), 0x10));
        xmm1_ = _mm_xor_si128    (xmm1_, xmm3_);       // pxor   xmm1, xmm3
        xmm3_ = _mm_castps_si128(_mm_shuffle_ps(       // shufps xmm3, xmm1, 0x10
                    _mm_castsi128_ps(xmm3_),
                    _mm_castsi128_ps(xmm1_), 0x8C));
        xmm1_ = _mm_xor_si128    (xmm1_, xmm3_);       // pxor   xmm1, xmm3
        xmm1_ = _mm_xor_si128    (xmm1_, xmm2_);       // pxor   xmm1, xmm2
    }

    void complete_key_expansion ()
    {
        xmm5_ = _mm_load_si128   (&xmm4_);             // movdqa xmm5, xmm4
        xmm5_ = _mm_slli_si128   (xmm5_, 0x04);        // pslldq xmm5, 0x04
        xmm6_ = _mm_castps_si128 (_mm_shuffle_ps(      // shufps xmm6, xmm1, 0x10
                    _mm_castsi128_ps(xmm6_),
                    _mm_castsi128_ps(xmm1_), 0xF0));
        xmm6_ = _mm_xor_si128    (xmm6_, xmm5_);       // pxor   xmm6, xmm5
        xmm4_ = _mm_xor_si128    (xmm4_, xmm6_);       // pxor   xmm4, xmm6
        xmm7_ = _mm_shuffle_epi32(xmm4_, 0x0E);        // pshufd xmm7, xmm4, 0x0E
    }

    template <std::size_t Rp1, typename Traits>
    void copy_key (StaticVector<__m128i, Rp1, Traits> &,
            const unsigned char *, cxx11::false_type) {}

    template <std::size_t Rp1, typename Traits>
    void copy_key (StaticVector<__m128i, Rp1, Traits> &key_seq,
            const unsigned char *ks_ptr, cxx11::true_type)
    {
        unsigned char *dst = reinterpret_cast<unsigned char *>(key_seq.data());
        std::memcpy(dst + 24, ks_ptr + 24, Rp1 * 16 - 24);
    }
}; // class AES192KeySeq

/// \brief AES-192 RNG engine
/// \ingroup AESNIRNG
///
/// \sa AES192KeySeq
/// \sa AESNIEngine
template <typename ResultType, std::size_t Blocks = VSMC_RNG_AES_BLOCKS>
class AES192Engine :
    public AESNIEngine<ResultType, AES192KeySeq<ResultType>, true, 12, Blocks>
{

    public :

    typedef AESNIEngine<ResultType, AES192KeySeq<ResultType>, true, 12, Blocks>
        base_eng_type;

    explicit AES192Engine (ResultType s = 0) : base_eng_type(s) {}

    template <typename SeedSeq>
    explicit AES192Engine (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_seq<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR) : base_eng_type(seq) {}

    AES192Engine (const typename base_eng_type::ctr_type &c,
            const typename base_eng_type::key_type &k) : base_eng_type(c, k) {}
}; // class AES192Engine

/// \brief AES-192 RNG engine with 32-bits integers output and default blocks
/// \ingroup AESNIRNG
typedef AES192Engine<uint32_t> AES192_32;

/// \brief AES-192 RNG engine with 64-bits integers output and default blocks
/// \ingroup AESNIRNG
typedef AES192Engine<uint64_t> AES192_64;

/// \brief AES256Engine key sequence generator
/// \ingroup AESNIRNG
template <typename ResultType>
class AES256KeySeq
{
    public :

    typedef StaticVector<ResultType, 32 / sizeof(ResultType)> key_type;

    AES256KeySeq () : xmm1_(), xmm2_(), xmm3_(), xmm4_() {}

    template <std::size_t Rp1, typename Traits>
    void generate (const key_type &key,
            StaticVector<__m128i, Rp1, Traits> &key_seq)
    {
        internal::AESKeyInit::key_init<0, 0>(key, key_seq, xmm1_);
        internal::AESKeyInit::key_init<16 / sizeof(ResultType), 1>(
                key, key_seq, xmm3_);
        generate_seq<2>(key_seq, cxx11::integral_constant<bool, 2 < Rp1>());
    }

    private :

    __m128i xmm1_;
    __m128i xmm2_;
    __m128i xmm3_;
    __m128i xmm4_;

    template <std::size_t, std::size_t Rp1, typename Traits>
    void generate_seq (StaticVector<__m128i, Rp1, Traits> &,
            cxx11::false_type) {}

    template <std::size_t N, std::size_t Rp1, typename Traits>
    void generate_seq (StaticVector<__m128i, Rp1, Traits> &key_seq,
            cxx11::true_type)
    {
        generate_key<N>(key_seq, cxx11::integral_constant<bool, N % 2 == 0>());
        generate_seq<N + 1>(key_seq,
                cxx11::integral_constant<bool, N + 1 < Rp1>());
    }

    template <std::size_t N, std::size_t Rp1, typename Traits>
    void generate_key (StaticVector<__m128i, Rp1, Traits> &key_seq,
            cxx11::true_type)
    {
        xmm2_ = _mm_aeskeygenassist_si128(xmm3_,
                internal::AESRoundConstant<N / 2>::value);
        expand_key(cxx11::true_type());
        key_seq[Position<N>()] = xmm1_;
    }

    template <std::size_t N, std::size_t Rp1, typename Traits>
    void generate_key (StaticVector<__m128i, Rp1, Traits> &key_seq,
            cxx11::false_type)
    {
        xmm4_ = _mm_aeskeygenassist_si128(xmm1_, 0);
        expand_key(cxx11::false_type());
        key_seq[Position<N>()] = xmm3_;
    }

    void expand_key (cxx11::true_type)
    {
        xmm2_ = _mm_shuffle_epi32(xmm2_, 0xFF);  // pshufd xmm2, xmm2, 0xFF
        xmm4_ = _mm_slli_si128   (xmm1_, 0x04);  // pshufb xmm4, xmm5
        xmm1_ = _mm_xor_si128    (xmm1_, xmm4_); // pxor   xmm1, xmm4
        xmm4_ = _mm_slli_si128   (xmm4_, 0x04);  // pslldq xmm4, 0x04
        xmm1_ = _mm_xor_si128    (xmm1_, xmm4_); // pxor   xmm1, xmm4
        xmm4_ = _mm_slli_si128   (xmm4_, 0x04);  // pslldq xmm4, 0x04
        xmm1_ = _mm_xor_si128    (xmm1_, xmm4_); // pxor   xmm1, xmm4
        xmm1_ = _mm_xor_si128    (xmm1_, xmm2_); // pxor   xmm1, xmm2
    }

    void expand_key (cxx11::false_type)
    {
        xmm2_ = _mm_shuffle_epi32(xmm4_, 0xAA);  // pshufd xmm2, xmm4, 0xAA
        xmm4_ = _mm_slli_si128   (xmm3_, 0x04);  // pshufb xmm4, xmm5
        xmm3_ = _mm_xor_si128    (xmm3_, xmm4_); // pxor   xmm3, xmm4
        xmm4_ = _mm_slli_si128   (xmm4_, 0x04);  // pslldq xmm4, 0x04
        xmm3_ = _mm_xor_si128    (xmm3_, xmm4_); // pxor   xmm3, xmm4
        xmm4_ = _mm_slli_si128   (xmm4_, 0x04);  // pslldq xmm4, 0x04
        xmm3_ = _mm_xor_si128    (xmm3_, xmm4_); // pxor   xmm3, xmm4
        xmm3_ = _mm_xor_si128    (xmm3_, xmm2_); // pxor   xmm1, xmm2
    }
}; // class AESKey256

/// \brief AES-256 RNG engine
/// \ingroup AESNIRNG
///
/// \sa AES256KeySeq
/// \sa AESNIEngine
template <typename ResultType, std::size_t Blocks = VSMC_RNG_AES_BLOCKS>
class AES256Engine :
    public AESNIEngine<ResultType, AES256KeySeq<ResultType>, true, 14, Blocks>
{

    public :

    typedef AESNIEngine<ResultType, AES256KeySeq<ResultType>, true, 14, Blocks>
        base_eng_type;

    explicit AES256Engine (ResultType s = 0) : base_eng_type(s) {}

    template <typename SeedSeq>
    explicit AES256Engine (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_seq<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR) : base_eng_type(seq) {}

    AES256Engine (const typename base_eng_type::ctr_type &c,
            const typename base_eng_type::key_type &k) : base_eng_type(c, k) {}
}; // class AES256Engine

/// \brief AES-256 RNG engine with 32-bits integers output and default blocks
/// \ingroup AESNIRNG
typedef AES256Engine<uint32_t> AES256_32;

/// \brief AES-256 RNG engine with 64-bits integers output and default blocks
/// \ingroup AESNIRNG
typedef AES256Engine<uint64_t> AES256_64;

} // namespace vsmc

#endif // VSMC_RNG_AES_HPP
