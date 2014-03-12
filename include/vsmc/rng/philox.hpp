#ifndef VSMC_RNG_PHILOX_HPP
#define VSMC_RNG_PHILOX_HPP

#include <vsmc/rng/internal/common.hpp>

#ifdef _MSC_VER
#include <intrin.h>
#endif

#define VSMC_STATIC_ASSERT_RNG_PHILOX_RESULT_TYPE(ResultType) \
    VSMC_STATIC_ASSERT(                                                      \
            (cxx11::is_same<ResultType, uint32_t>::value ||                  \
             cxx11::is_same<ResultType, uint64_t>::value),                   \
            USE_PhiloxEngine_WITH_INTEGER_TYPE_OTHER_THAN_uint32_t_OR_uint64_t)

#define VSMC_STATIC_ASSERT_RNG_PHILOX_SIZE(K) \
    VSMC_STATIC_ASSERT((K == 2 || K == 4),                                   \
            USE_PhiloxEngine_WITH_SIZE_OTHER_THAN_2_OR_4)

#define VSMC_STATIC_ASSERT_RNG_PHILOX \
        VSMC_STATIC_ASSERT_RNG_PHILOX_RESULT_TYPE(ResultType);               \
        VSMC_STATIC_ASSERT_RNG_PHILOX_SIZE(K);

#define VSMC_DEFINE_RNG_PHILOX_BUMPK_CONSTANT(T, I, val) \
    template <> struct PhiloxBumpkConstant < T, I > :                        \
        public cxx11::integral_constant< T, val > {};

#define VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(T, K, I, val) \
    template <> struct PhiloxRoundConstant < T, K, I > :                     \
        public cxx11::integral_constant< T, val > {};

/// \brief PhiloxEngine default rounds
/// \ingroup Config
#ifndef VSMC_RNG_PHILOX_ROUNDS
#define VSMC_RNG_PHILOX_ROUNDS 10
#endif

namespace vsmc {

namespace internal {

template <typename, std::size_t> struct PhiloxBumpkConstant;

VSMC_DEFINE_RNG_PHILOX_BUMPK_CONSTANT(uint32_t, 0,
        static_cast<uint32_t>(0x9E3779B9))
VSMC_DEFINE_RNG_PHILOX_BUMPK_CONSTANT(uint32_t, 1,
        static_cast<uint32_t>(0xBB67AE85))

VSMC_DEFINE_RNG_PHILOX_BUMPK_CONSTANT(uint64_t, 0,
        UINT64_C(0x9E3779B97F4A7C15))
VSMC_DEFINE_RNG_PHILOX_BUMPK_CONSTANT(uint64_t, 1,
        UINT64_C(0xBB67AE8584CAA73B))

template <typename, std::size_t, std::size_t> struct PhiloxRoundConstant;

VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(uint32_t, 2, 0,
        static_cast<uint32_t>(0xd256d193))

VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(uint32_t, 4, 0,
        static_cast<uint32_t>(0xD2511F53))
VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(uint32_t, 4, 1,
        static_cast<uint32_t>(0xCD9E8D57))

VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(uint64_t, 2, 0,
        UINT64_C(0xD2B74407B1CE6E93))

VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(uint64_t, 4, 0,
        UINT64_C(0xD2E7470EE14C6C93))
VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(uint64_t, 4, 1,
        UINT64_C(0xCA5A826395121157))

} // namespace vsmc::internal

namespace traits {

/// \brief Traits of PhiloxEngine constants for bumping the key (Weyl
/// sequence)
/// \ingroup Traits
///
/// \details
/// The first template argument is either `uint32_t` or `uint64_t`. The second
/// is either 0 or 1. Specializing the class templates
/// `PhiloxBumpkConstantTrait<uint64_t, 0>` etc., are equivalent to define
/// macros `PHILOX_W64_0` etc., in the original implementation.
template <typename ResultType, std::size_t N>
struct PhiloxBumpkConstantTrait :
    public ::vsmc::internal::PhiloxBumpkConstant<ResultType, N> {};

/// \brief Traits of PhiloxEngine constants for rounding
/// \ingroup Traits
///
/// \details
/// The first template argument is either `uint32_t` or `uint64_t`. The second
/// is the size of the RNG, either 2 or 4. The third is either 0 or 1.
/// Specializing the class templates `PhiloxRoundConstantTrait<uint64_t, 4, 0>`
/// etc., are equivalent to define macros `PHILOX_M4x64_0` etc., in the
/// original implementation.
template <typename ResultType, std::size_t K, std::size_t N>
struct PhiloxRoundConstantTrait :
    public ::vsmc::internal::PhiloxRoundConstant<ResultType, K, N> {};

} // namespace vsmc::traits

namespace internal {

template <typename ResultType, std::size_t K, std::size_t N, bool = (N > 1)>
struct PhiloxBumpk {static void bumpk (StaticVector<ResultType, K / 2> &) {}};

template <typename ResultType, std::size_t N>
struct PhiloxBumpk<ResultType, 2, N, true>
{
    static void bumpk (StaticVector<ResultType, 1> &par)
    {
        par[Position<0>()] +=
            traits::PhiloxBumpkConstantTrait<ResultType, 0>::value;
    }
}; // struct PhiloxBumpk

template <typename ResultType, std::size_t N>
struct PhiloxBumpk<ResultType, 4, N, true>
{
    static void bumpk (StaticVector<ResultType, 2> &par)
    {
        par[Position<0>()] +=
            traits::PhiloxBumpkConstantTrait<ResultType, 0>::value;
        par[Position<1>()] +=
            traits::PhiloxBumpkConstantTrait<ResultType, 1>::value;
    }
}; // struct PhiloxBumpk

template <std::size_t K, std::size_t N>
inline void philox_hilo (uint32_t b, uint32_t &hi, uint32_t &lo)
{
    uint64_t prod =
        static_cast<uint64_t>(b) *
        static_cast<uint64_t>(
                traits::PhiloxRoundConstantTrait<uint32_t, K, N>::value);
    hi = prod >> 32;
    lo = static_cast<uint32_t>(prod);
}

#if VSMC_HAS_INT128

template <std::size_t K, std::size_t N>
inline void philox_hilo (uint64_t b, uint64_t &hi, uint64_t &lo)
{
    unsigned VSMC_INT128 prod =
        static_cast<unsigned VSMC_INT128>(b) *
        static_cast<unsigned VSMC_INT128>(
                traits::PhiloxRoundConstantTrait<uint64_t, K, N>::value);
    hi = prod >> 64;
    lo = static_cast<uint64_t>(prod);
}

#elif defined(_MSC_VER) // VSMC_HAS_INT128

template <std::size_t K, std::size_t N>
inline void philox_hilo (uint64_t b, uint64_t &hi, uint64_t &lo)
{
    lo = _umul128(traits::PhiloxRoundConstantTrait<uint64_t, K, N>::value, b,
            &hi);
}

#else // VSMC_HAS_INT128

template <std::size_t K, std::size_t I>
inline void philox_hilo (uint64_t b, uint64_t &hi, uint64_t &lo)
{
    const uint64_t a =
        traits::PhiloxRoundConstantTrait<uint64_t, K, I>::value;
    const unsigned whalf = 32;
    const uint64_t lomask = (static_cast<uint64_t>(1) << whalf) - 1;

    lo = static_cast<uint64_t>(a * b);

    uint64_t ahi = a >> whalf;
    uint64_t alo = a & lomask;
    uint64_t bhi = b >> whalf;
    uint64_t blo = b & lomask;

    uint64_t ahbl = ahi * blo;
    uint64_t albh = alo * bhi;

    uint64_t ahbl_albh = ((ahbl & lomask) + (albh & lomask));

    hi = ahi * bhi + (ahbl >> whalf) + (albh >> whalf);
    hi += ahbl_albh >> whalf;
    hi += ((lo >> whalf) < (ahbl_albh & lomask));
}

#endif // VSMC_HAS_INT128

template <typename ResultType, std::size_t K, std::size_t N, bool = (N > 0)>
struct PhiloxRound
{
    static void round (StaticVector<ResultType, K> &,
            const StaticVector<ResultType, K / 2> &) {}
}; // struct PhiloxRound

template <typename ResultType, std::size_t N>
struct PhiloxRound<ResultType, 2, N, true>
{
    static void round (StaticVector<ResultType, 2> &state,
            const StaticVector<ResultType, 1> &par)
    {
        ResultType hi = 0;
        ResultType lo = 0;
        philox_hilo<2, 0>(state[Position<0>()], hi, lo);
        state[Position<0>()] = hi^par[Position<0>()]^state[Position<1>()];
        state[Position<1>()] = lo;
    }
}; // struct PhiloxRound

template <typename ResultType, std::size_t N>
struct PhiloxRound<ResultType, 4, N, true>
{
    static void round (StaticVector<ResultType, 4> &state,
            const StaticVector<ResultType, 2> &par)
    {
        ResultType hi0 = 0;
        ResultType hi1 = 0;
        ResultType lo0 = 0;
        ResultType lo1 = 0;
        philox_hilo<4, 0>(state[Position<0>()], hi0, lo0);
        philox_hilo<4, 1>(state[Position<2>()], hi1, lo1);
        state[Position<0>()] = hi1^state[Position<1>()]^par[Position<0>()];
        state[Position<1>()] = lo1;
        state[Position<2>()] = hi0^state[Position<3>()]^par[Position<1>()];
        state[Position<3>()] = lo0;
    }
}; // struct PhiloxRound

} // namespace vsmc::internal

/// \brief Philox RNG engine reimplemented
/// \ingroup R123RNG
///
/// \details
/// This is a reimplementation of the algorithm Philox as described in
/// [Parallel Random Numbers: As Easy as 1, 2, 3][r123paper] and implemented in
/// [Random123][r123lib].
///
/// [r123paper]:http://sc11.supercomputing.org/schedule/event_detail.php?evid=pap274
/// [r123lib]: https://www.deshawresearch.com/resources_random123.html
///
/// Depending on the compilers, processors and RNG configurations, it might be
/// slightly faster or slower than the original implementation. At most
/// two-folds performace difference (both faster and slower) were observed.
///
/// Currently the 64-bits version is much slower than the original, except when
/// using recent Clang, GCC, Intel C++ or MSVC on x86-64 computers. The
/// original implementation use some platform dependent assembly or intrinsics
/// to optimize the performance. This implementation use standard C99 when used
/// on other platforms.
///
/// This implementation is slightly more flexible in the sense that it does not
/// limit the number of rounds. However, larger number of rounds can have
/// undesired effects. To say the least, currently all loops are unrolled,
/// which can slow down significantly when the number of rounds is large.
///
/// Compared to `r123:Engine<r123::Philox4x32>` etc., when using the default
/// constructor or the one with a single seed, the output shall be exactly the
/// same for the first \f$2^n\f$ iterations, where \f$n\f$ is the number of
/// bits (32 or 64).  Further iterations may produce different results, as vSMC
/// increment the counter slightly differently, but it still cover the same
/// range and has the same period as the original.
///
/// The constants of bumping the key (Weyl constants) and those used in each
/// rounds can be set through traits, `vsmc::traits::PhiloxBumpkConstantTrait`
/// and `vsmc::traits::PhiloxRoundConstantTrait`.
template <typename ResultType, std::size_t K,
         std::size_t Rounds = VSMC_RNG_PHILOX_ROUNDS>
class PhiloxEngine
{
    static VSMC_CONSTEXPR const std::size_t buffer_size_ = K;

    typedef internal::RngCounter<ResultType> counter;

    public :

    typedef ResultType result_type;
    typedef StaticVector<ResultType, buffer_size_> buffer_type;
    typedef StaticVector<ResultType, K> ctr_type;
    typedef StaticVector<ResultType, K / 2> key_type;

    explicit PhiloxEngine (result_type s = 0) : remain_(0)
    {
        VSMC_STATIC_ASSERT_RNG_PHILOX;
        seed(s);
    }

    template <typename SeedSeq>
    explicit PhiloxEngine (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_seq<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR) : remain_(0)
    {
        VSMC_STATIC_ASSERT_RNG_PHILOX;
        seed(seq);
    }

    PhiloxEngine (const ctr_type &c, const key_type &k) : key_(k), remain_(0)
    {
        VSMC_STATIC_ASSERT_RNG_PHILOX;
        counter::set(ctr_, c);
    }

    void seed (result_type s)
    {
        counter::reset(ctr_);
        key_.fill(0);
        key_.front() = s;
        remain_ = 0;
    }

    template <typename SeedSeq>
    void seed (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_seq<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR)
    {
        counter::reset(ctr_);
        seq.generate(key_.begin(), key_.end());
        remain_ = 0;
    }

    const ctr_type &ctr () const {return counter::get(ctr_);}

    const key_type &key () const {return key_;}

    void ctr (const ctr_type &c)
    {
        counter::set(ctr_, c);
        remain_ = 0;
    }

    void key (const key_type &k)
    {
        key_ = k;
        remain_ = 0;
    }

    /// \brief After reset, next call to `operator()` will always increase the
    /// counter and refresh the buffer
    void reset () {remain_ = 0;}

    static VSMC_CONSTEXPR std::size_t buffer_size () {return buffer_size_;}

    const buffer_type &buffer () {return buffer_;}

    /// \brief Generate a buffer of random bits given the counter and key
    ///
    /// \details
    /// This is much slower than calling `operator()` to generate the same
    /// amount of bits, since each call to this function requires the key to be
    /// initialized.
    static buffer_type generate (const ctr_type &c, const key_type &k)
    {
        PhiloxEngine<ResultType, K, Rounds> eng(c, k);
        eng.generate();

        return eng.buffer_;
    }

    /// \brief Generate a buffer of random bits given the counter and using the
    /// current key
    ///
    /// \details
    /// This is (hopefully not much) slower than calling `operator()` to
    /// generate the same amount of bits, since the state of the engine has to
    /// be saved.
    buffer_type generate (const ctr_type &c) const
    {
        PhiloxEngine<ResultType, K, Rounds> eng(*this);
        eng.ctr(c);
        eng.generate();

        return eng.buffer_;
    }

    result_type operator() ()
    {
        if (remain_ == 0) {
            counter::increment(ctr_);
            generate();
            remain_ = buffer_size_;
        }
        --remain_;

        return buffer_[remain_];
    }

    void discard (std::size_t nskip)
    {
        if (nskip <= remain_) {
            remain_ -= nskip;
            return;
        }

        nskip -= remain_;
        if (nskip <= buffer_size_) {
            remain_ = 0;
            operator()();
            remain_ = buffer_size_ - nskip;
            return;
        }

        remain_ = 0;
        counter::increment(ctr_, nskip / buffer_size_);
        operator()();
        remain_ = buffer_size_ - nskip % buffer_size_;
    }

    static VSMC_CONSTEXPR const result_type _Min = 0;
    static VSMC_CONSTEXPR const result_type _Max = static_cast<result_type>(
            ~(static_cast<result_type>(0)));

    static VSMC_CONSTEXPR result_type min VSMC_MNE () {return _Min;}
    static VSMC_CONSTEXPR result_type max VSMC_MNE () {return _Max;}

    friend inline bool operator== (
            const PhiloxEngine<ResultType, K, Rounds> &eng1,
            const PhiloxEngine<ResultType, K, Rounds> &eng2)
    {
        return
            eng1.ctr_ == eng2.ctr_ &&
            eng1.key_ == eng2.key_ &&
            eng1.buffer_ == eng2.buffer_ &&
            eng1.remain_ == eng2.remain_;
    }

    friend inline bool operator!= (
            const PhiloxEngine<ResultType, K, Rounds> &eng1,
            const PhiloxEngine<ResultType, K, Rounds> &eng2)
    {return !(eng1 == eng2);}

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &operator<< (
            std::basic_ostream<CharT, Traits> &os,
            const PhiloxEngine<ResultType, K, Rounds> &eng)
    {
        if (os) os << eng.ctr_;    if (os) os << ' ';
        if (os) os << eng.key_;    if (os) os << ' ';
        if (os) os << eng.par_;    if (os) os << ' ';
        if (os) os << eng.buffer_; if (os) os << ' ';
        if (os) os << eng.remain_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &operator>> (
            std::basic_istream<CharT, Traits> &is,
            PhiloxEngine<ResultType, K, Rounds> &eng)
    {
        PhiloxEngine<ResultType, K, Rounds> eng_tmp;
        if (is) is >> std::ws >> eng_tmp.ctr_;
        if (is) is >> std::ws >> eng_tmp.key_;
        if (is) is >> std::ws >> eng_tmp.par_;
        if (is) is >> std::ws >> eng_tmp.buffer_;
        if (is) is >> std::ws >> eng_tmp.remain_;
        if (is) eng = eng_tmp;

        return is;
    }

    private :

    ctr_type ctr_;
    key_type key_;
    key_type par_;
    buffer_type buffer_;
    std::size_t remain_;

    void generate ()
    {
        buffer_ = ctr_;
        par_ = key_;
        generate<0>(cxx11::true_type());
    }

    template <std::size_t> void generate (cxx11::false_type) {}

    template <std::size_t N>
    void generate (cxx11::true_type)
    {
        internal::PhiloxBumpk<ResultType, K, N>::bumpk(par_);
        internal::PhiloxRound<ResultType, K, N>::round(buffer_, par_);
        generate<N + 1>(cxx11::integral_constant<bool, N < Rounds>());
    }
}; // class PhiloxEngine

/// \brief Philox2x32 RNG engine reimplemented
/// \ingroup R123RNG
typedef PhiloxEngine<uint32_t, 2> Philox2x32;

/// \brief Philox4x32 RNG engine reimplemented
/// \ingroup R123RNG
typedef PhiloxEngine<uint32_t, 4> Philox4x32;

/// \brief Philox2x64 RNG engine reimplemented
/// \ingroup R123RNG
typedef PhiloxEngine<uint64_t, 2> Philox2x64;

/// \brief Philox4x64 RNG engine reimplemented
/// \ingroup R123RNG
typedef PhiloxEngine<uint64_t, 4> Philox4x64;

} // namespace vsmc

#endif // VSMC_RNG_PHILOX_HPP
