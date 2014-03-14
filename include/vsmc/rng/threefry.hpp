#ifndef VSMC_RNG_THREEFRY_HPP
#define VSMC_RNG_THREEFRY_HPP

#include <vsmc/rng/internal/common.hpp>

#define VSMC_STATIC_ASSERT_RNG_THREEFRY_RESULT_TYPE(ResultType) \
    VSMC_STATIC_ASSERT(                                                      \
            (cxx11::is_same<ResultType, uint32_t>::value ||                  \
             cxx11::is_same<ResultType, uint64_t>::value),                   \
            USE_ThreefryEngine_WITH_INTEGER_TYPE_OTHER_THAN_uint32_t_OR_uint64_t)

#define VSMC_STATIC_ASSERT_RNG_THREEFRY_SIZE(K) \
    VSMC_STATIC_ASSERT((K == 2 || K == 4),                                   \
            USE_ThreefryEngine_WITH_SIZE_OTHER_THAN_2_OR_4)

#define VSMC_STATIC_ASSERT_RNG_THREEFRY \
        VSMC_STATIC_ASSERT_RNG_THREEFRY_RESULT_TYPE(ResultType);             \
        VSMC_STATIC_ASSERT_RNG_THREEFRY_SIZE(K);

#define VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(T, K, N, I, val) \
    template <> struct ThreefryRotateConstant < T, K, N, I > :               \
        public cxx11::integral_constant< unsigned, val > {};

/// \brief ThreefryEngine default rounds
/// \ingroup Config
#ifndef VSMC_RNG_THREEFRY_ROUNDS
#define VSMC_RNG_THREEFRY_ROUNDS 20
#endif

namespace vsmc {

namespace internal {

template <typename> struct ThreefryPar;

template <> struct ThreefryPar<uint32_t> :
    public cxx11::integral_constant<uint32_t, 0x1BD11BDA> {};

template <> struct ThreefryPar<uint64_t> :
    public cxx11::integral_constant<uint64_t,
           (static_cast<uint64_t>(0xA9FC1A22) +
            (static_cast<uint64_t>(0x1BD11BDA) << 32))> {};

template <typename, std::size_t, std::size_t, std::size_t>
struct ThreefryRotateConstant;

VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint32_t, 2, 0, 0, 13)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint32_t, 2, 1, 0, 15)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint32_t, 2, 2, 0, 26)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint32_t, 2, 3, 0,  6)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint32_t, 2, 4, 0, 17)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint32_t, 2, 5, 0, 29)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint32_t, 2, 6, 0, 16)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint32_t, 2, 7, 0, 24)

VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint32_t, 4, 0, 0, 10)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint32_t, 4, 1, 0, 11)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint32_t, 4, 2, 0, 13)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint32_t, 4, 3, 0, 23)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint32_t, 4, 4, 0,  6)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint32_t, 4, 5, 0, 17)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint32_t, 4, 6, 0, 25)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint32_t, 4, 7, 0, 18)

VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint32_t, 4, 0, 1, 26)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint32_t, 4, 1, 1, 21)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint32_t, 4, 2, 1, 27)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint32_t, 4, 3, 1,  5)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint32_t, 4, 4, 1, 20)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint32_t, 4, 5, 1, 11)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint32_t, 4, 6, 1, 10)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint32_t, 4, 7, 1, 20)

VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint64_t, 2, 0, 0, 16)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint64_t, 2, 1, 0, 42)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint64_t, 2, 2, 0, 12)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint64_t, 2, 3, 0, 31)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint64_t, 2, 4, 0, 16)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint64_t, 2, 5, 0, 32)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint64_t, 2, 6, 0, 24)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint64_t, 2, 7, 0, 21)

VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint64_t, 4, 0, 0, 14)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint64_t, 4, 1, 0, 52)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint64_t, 4, 2, 0, 23)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint64_t, 4, 3, 0,  5)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint64_t, 4, 4, 0, 25)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint64_t, 4, 5, 0, 46)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint64_t, 4, 6, 0, 58)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint64_t, 4, 7, 0, 32)

VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint64_t, 4, 0, 1, 16)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint64_t, 4, 1, 1, 57)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint64_t, 4, 2, 1, 40)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint64_t, 4, 3, 1, 37)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint64_t, 4, 4, 1, 33)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint64_t, 4, 5, 1, 12)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint64_t, 4, 6, 1, 22)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(uint64_t, 4, 7, 1, 32)

template <typename ResultType, unsigned N> struct ThreefryRotateImpl;

template <unsigned N>
struct ThreefryRotateImpl<uint32_t, N>
{
    static uint32_t rotate (uint32_t x)
    {return (x << (N & 31) | x >> ((32 - N) & 31));}
}; // struct ThreefryRotateImpl

template <unsigned N>
struct ThreefryRotateImpl<uint64_t, N>
{
    static uint64_t rotate (uint64_t x)
    {return (x << (N & 63) | x >> ((64 - N) & 63));}
}; // struct ThreefryRotateImpl

template <typename ResultType, std::size_t K, std::size_t N, bool = (N > 0)>
struct ThreefryRotate {static void rotate (StaticVector<ResultType, K> &) {}};

template <typename ResultType, std::size_t N>
struct ThreefryRotate<ResultType, 2, N, true>
{
    static void rotate (StaticVector<ResultType, 2> &state)
    {
        state[Position<0>()] += state[Position<1>()];
        state[Position<1>()] =
            ThreefryRotateImpl<ResultType, ThreefryRotateConstant<
            ResultType, 2, r_, 0>::value>::rotate(state[Position<1>()]);
        state[Position<1>()] ^= state[Position<0>()];
    }

    private :

    static VSMC_CONSTEXPR const unsigned r_ = (N - 1) % 8;
}; // struct ThreefryRotate

template <typename ResultType, std::size_t N>
struct ThreefryRotate<ResultType, 4, N, true>
{
    static void rotate (StaticVector<ResultType, 4> &state)
    {
        state[Position<0>()] += state[Position<i0_>()];
        state[Position<i0_>()] =
            ThreefryRotateImpl<ResultType, ThreefryRotateConstant<
            ResultType, 4, r_, 0>::value>::rotate(state[Position<i0_>()]);
        state[Position<i0_>()] ^= state[Position<0>()];

        state[Position<2>()] += state[Position<i2_>()];
        state[Position<i2_>()] =
            ThreefryRotateImpl<ResultType, ThreefryRotateConstant<
            ResultType, 4, r_, 1>::value>::rotate(state[Position<i2_>()]);
        state[Position<i2_>()] ^= state[Position<2>()];
    }

    private :

    static VSMC_CONSTEXPR const std::size_t i0_ = N % 2 ? 1 : 3;
    static VSMC_CONSTEXPR const std::size_t i2_ = N % 2 ? 3 : 1;
    static VSMC_CONSTEXPR const unsigned r_ = (N - 1) % 8;
}; // struct ThreefryRotate

template <typename ResultType, std::size_t K, std::size_t N,
         bool = (N % 4 == 0)>
struct ThreefryInsert
{
    static void insert (StaticVector<ResultType, K> &,
            const StaticVector<ResultType, K + 1> &) {}
}; // struct ThreefryInsert

template <typename ResultType, std::size_t N>
struct ThreefryInsert<ResultType, 2, N, true>
{
    static void insert (StaticVector<ResultType, 2> &state,
            const StaticVector<ResultType, 3> &par)
    {
        state[Position<0>()] += par[Position<i0_>()];
        state[Position<1>()] += par[Position<i1_>()];
        state[Position<1>()] += inc_;
    }

    private :

    static VSMC_CONSTEXPR const std::size_t inc_ = N / 4;
    static VSMC_CONSTEXPR const std::size_t i0_ = (inc_ + 0) % 3;
    static VSMC_CONSTEXPR const std::size_t i1_ = (inc_ + 1) % 3;
}; // struct ThreefryInsert

template <typename ResultType, std::size_t N>
struct ThreefryInsert<ResultType, 4, N, true>
{
    static void insert (StaticVector<ResultType, 4> &state,
            const StaticVector<ResultType, 5> &par)
    {
        state[Position<0>()] += par[Position<i0_>()];
        state[Position<1>()] += par[Position<i1_>()];
        state[Position<2>()] += par[Position<i2_>()];
        state[Position<3>()] += par[Position<i3_>()];
        state[Position<3>()] += inc_;
    }

    private :

    static VSMC_CONSTEXPR const std::size_t inc_ = N / 4;
    static VSMC_CONSTEXPR const std::size_t i0_ = (inc_ + 0) % 5;
    static VSMC_CONSTEXPR const std::size_t i1_ = (inc_ + 1) % 5;
    static VSMC_CONSTEXPR const std::size_t i2_ = (inc_ + 2) % 5;
    static VSMC_CONSTEXPR const std::size_t i3_ = (inc_ + 3) % 5;
}; // struct ThreefryInsert

} // namespace vsmc::internal

/// \brief Threefry RNG engine reimplemented
/// \ingroup R123RNG
///
/// \details
/// This is a reimplementation of the algorithm Threefry as described in
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
/// This implementation is slightly more flexible in the sense that it does not
/// limit the number of rounds. However, larger number of rounds can have
/// undesired effects. To say the least, currently all loops are unrolled,
/// which can slow down significantly when the number of rounds is large.
///
/// Compared to `r123:Engine<r123::Threefry4x32>` etc., when using the default
/// constructor or the one with a single seed, the output shall be exactly the
/// same for the first \f$2^n\f$ iterations, where \f$n\f$ is the number of
/// bits (32 or 64).  Further iterations may produce different results, as vSMC
/// increment the counter slightly differently, but it still cover the same
/// range and has the same period as the original.
template <typename ResultType, std::size_t K,
         std::size_t Rounds = VSMC_RNG_THREEFRY_ROUNDS>
class ThreefryEngine
{
    static VSMC_CONSTEXPR const std::size_t buffer_size_ = K;

    public :

    typedef ResultType result_type;
    typedef StaticVector<ResultType, buffer_size_> buffer_type;
    typedef StaticVector<ResultType, K> ctr_type;
    typedef StaticVector<ResultType, K> key_type;

    private :

    typedef StaticCounter<ctr_type> counter;

    public :

    explicit ThreefryEngine (result_type s = 0) : remain_(0)
    {
        VSMC_STATIC_ASSERT_RNG_THREEFRY;
        seed(s);
    }

    template <typename SeedSeq>
    explicit ThreefryEngine (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_seq<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR) : remain_(0)
    {
        VSMC_STATIC_ASSERT_RNG_THREEFRY;
        seed(seq);
    }

    ThreefryEngine (const ctr_type &c, const key_type &k) : remain_(0)
    {
        VSMC_STATIC_ASSERT_RNG_THREEFRY;
        counter::set(ctr_, c);
        init_par(k);
    }

    void seed (result_type s)
    {
        counter::reset(ctr_);
        key_type k;
        k.fill(0);
        k.front() = s;
        init_par(k);
        remain_ = 0;
    }

    template <typename SeedSeq>
    void seed (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_seq<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR)
    {
        counter::reset(ctr_);
        key_type k;
        seq.generate(k.begin(), k.end());
        init_par(k);
        remain_ = 0;
    }

    ctr_type ctr () const {return ctr_;}

    key_type key () const {return par_.template slice<0, K>();}

    void ctr (const ctr_type &c)
    {
        counter::set(ctr_, c);
        remain_ = 0;
    }

    void key (const key_type &k)
    {
        init_par(k);
        remain_ = 0;
    }

    /// \brief After reset, next call to `operator()` will always increase the
    /// counter and refresh the buffer
    void reset ()
    {
        counter::reset(ctr_);
        remain_ = 0;
    }

    result_type operator() ()
    {
        if (remain_ == 0) {
            counter::increment(ctr_);
            generate_buffer(ctr_, buffer_);
            remain_ = buffer_size_;
        }
        --remain_;

        return buffer_[remain_];
    }

    /// \brief Generate a buffer of random bits given a counter using the
    /// current key
    buffer_type operator() (const ctr_type &c) const
    {
        buffer_type buf;
        generate_buffer(c, buf);

        return buf;
    }

    /// \brief Generate random bits in a pre-allocated buffer given a counter
    /// using the current key
    void operator() (const ctr_type &c, buffer_type &buf) const
    {generate_buffer(c, buf);}

    void discard (result_type nskip)
    {
        std::size_t n = static_cast<std::size_t>(nskip);
        if (n <= remain_) {
            remain_ -= n;
            return;
        }

        n -= remain_;
        if (n <= buffer_size_ ) {
            remain_ = 0;
            operator()();
            remain_ = buffer_size_ - n;
            return;
        }

        remain_ = 0;
        counter::increment(ctr_, static_cast<result_type>(n / buffer_size_));
        operator()();
        remain_ = buffer_size_ - n % buffer_size_;
    }

    static VSMC_CONSTEXPR const result_type _Min = 0;
    static VSMC_CONSTEXPR const result_type _Max = static_cast<result_type>(
            ~(static_cast<result_type>(0)));

    static VSMC_CONSTEXPR result_type min VSMC_MNE () {return _Min;}
    static VSMC_CONSTEXPR result_type max VSMC_MNE () {return _Max;}

    friend inline bool operator== (
            const ThreefryEngine<ResultType, K, Rounds> &eng1,
            const ThreefryEngine<ResultType, K, Rounds> &eng2)
    {
        return
            eng1.ctr_ == eng2.ctr_ &&
            eng1.par_ == eng2.par_ &&
            eng1.buffer_ == eng2.buffer_ &&
            eng1.remain_ == eng2.remain_;
    }

    friend inline bool operator!= (
            const ThreefryEngine<ResultType, K, Rounds> &eng1,
            const ThreefryEngine<ResultType, K, Rounds> &eng2)
    {return !(eng1 == eng2);}

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &operator<< (
            std::basic_ostream<CharT, Traits> &os,
            const ThreefryEngine<ResultType, K, Rounds> &eng)
    {
        if (os) os << eng.ctr_;    if (os) os << ' ';
        if (os) os << eng.par_;    if (os) os << ' ';
        if (os) os << eng.buffer_; if (os) os << ' ';
        if (os) os << eng.remain_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &operator>> (
            std::basic_istream<CharT, Traits> &is,
            ThreefryEngine<ResultType, K, Rounds> &eng)
    {
        ThreefryEngine<ResultType, K, Rounds> eng_tmp;
        if (is) is >> std::ws >> eng_tmp.ctr_;
        if (is) is >> std::ws >> eng_tmp.par_;
        if (is) is >> std::ws >> eng_tmp.buffer_;
        if (is) is >> std::ws >> eng_tmp.remain_;
        if (is) eng = eng_tmp;

        return is;
    }

    private :

    ctr_type ctr_;
    StaticVector<ResultType, K + 1> par_;
    buffer_type buffer_;
    std::size_t remain_;

    void generate_buffer (const ctr_type &c, buffer_type &buf) const
    {
        buf = c;
        generate_buffer<0>(buf, cxx11::true_type());
    }

    template <std::size_t>
    void generate_buffer (buffer_type &, cxx11::false_type) const {}

    template <std::size_t N>
    void generate_buffer (buffer_type &buf, cxx11::true_type) const
    {
        internal::ThreefryRotate<ResultType, K, N>::rotate(buf);
        internal::ThreefryInsert<ResultType, K, N>::insert(buf, par_);
        generate_buffer<N + 1>(buf,
                cxx11::integral_constant<bool, N < Rounds>());
    }

    void init_par (const key_type &key)
    {
        par_.back() = internal::ThreefryPar<ResultType>::value;
        par_xor<0>(key, cxx11::integral_constant<bool, 0 < K>());
    }

    template <std::size_t>
    void par_xor (const key_type &, cxx11::false_type) {}

    template <std::size_t N>
    void par_xor (const key_type &key, cxx11::true_type)
    {
        par_[Position<N>()] = key[Position<N>()];
        par_.back() ^= key[Position<N>()];
        par_xor<N + 1>(key, cxx11::integral_constant<bool, N + 1 < K>());
    }
}; // class ThreefryEngine

/// \brief Threefry2x32 RNG engine reimplemented
/// \ingroup R123RNG
typedef ThreefryEngine<uint32_t, 2> Threefry2x32;

/// \brief Threefry4x32 RNG engine reimplemented
/// \ingroup R123RNG
typedef ThreefryEngine<uint32_t, 4> Threefry4x32;

/// \brief Threefry2x64 RNG engine reimplemented
/// \ingroup R123RNG
typedef ThreefryEngine<uint64_t, 2> Threefry2x64;

/// \brief Threefry4x64 RNG engine reimplemented
/// \ingroup R123RNG
typedef ThreefryEngine<uint64_t, 4> Threefry4x64;

} // namespace vsmc

#endif // VSMC_RNG_THREEFRY_HPP
