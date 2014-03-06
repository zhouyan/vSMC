#ifndef VSMC_RNG_THREEFRY_HPP
#define VSMC_RNG_THREEFRY_HPP

#include <vsmc/rng/common.hpp>

#define VSMC_STATIC_ASSERT_RNG_THREEFRY

#define VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(T, K, N, R, val) \
    template <> struct ThreefryRotateConstant < T, K, N, R > :               \
        public cxx11::integral_constant< unsigned, val > {};

namespace vsmc {

namespace internal {

template <typename, std::size_t> struct ThreefryIncrement;

template <typename ResultType>
struct ThreefryIncrement<ResultType, 2>
{
    static void increment (ResultType *ctr)
    {
        if (ctr[0] < max_) {++ctr[0]; return;}
        if (ctr[1] < max_) {++ctr[0]; return;}
        ctr[0] = 0;
    }

    private :

    static VSMC_CONSTEXPR const ResultType max_ = static_cast<ResultType>(
            ~(static_cast<ResultType>(0)));
};

template <typename ResultType>
struct ThreefryIncrement<ResultType, 4>
{
    static void increment (ResultType *ctr)
    {
        if (ctr[0] < max_) {++ctr[0]; return;}
        if (ctr[1] < max_) {++ctr[0]; return;}
        if (ctr[2] < max_) {++ctr[0]; return;}
        if (ctr[3] < max_) {++ctr[0]; return;}
        ctr[0] = 0;
    }

    private :

    static VSMC_CONSTEXPR const ResultType max_ = static_cast<ResultType>(
            ~(static_cast<ResultType>(0)));
};

template <typename> struct ThreefryParity;

template <> struct ThreefryParity<uint32_t> :
    public cxx11::integral_constant<uint32_t, 0x1BD11BDA> {};

template <> struct ThreefryParity<uint64_t> :
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

template <typename ResultType, std::size_t, std::size_t N, bool = (N != 0)>
struct ThreefryRotate {static void rotate (ResultType *) {}};

template <typename ResultType, std::size_t N>
struct ThreefryRotate<ResultType, 2, N, true>
{
    static void rotate (ResultType *state)
    {
        state[0] += state[1];
        state[1] = RngRotate<ResultType, ThreefryRotateConstant<
            ResultType, 2, r_, 0>::value>::rotate(state[1]);
        state[1] ^= state[0];
    }

    private :

    static VSMC_CONSTEXPR const unsigned r_ = (N - 1) % 8;
};

template <typename ResultType, std::size_t N>
struct ThreefryRotate<ResultType, 4, N, true>
{
    static void rotate (ResultType *state)
    {
        state[0] += state[i0_];
        state[i0_] = RngRotate<ResultType, ThreefryRotateConstant<
            ResultType, 4, r_, 0>::value>::rotate(state[i0_]);
        state[i0_] ^= state[0];

        state[2] += state[i2_];
        state[i2_] = RngRotate<ResultType, ThreefryRotateConstant<
            ResultType, 4, r_, 1>::value>::rotate(state[i2_]);
        state[i2_] ^= state[2];
    }

    private :

    static VSMC_CONSTEXPR const std::size_t i0_ = N % 2 ? 1 : 3;
    static VSMC_CONSTEXPR const std::size_t i2_ = N % 2 ? 3 : 1;
    static VSMC_CONSTEXPR const unsigned r_ = (N - 1) % 8;
};

template <typename ResultType, std::size_t, std::size_t N, bool = (N % 4 == 0)>
struct ThreefryInsert
{static void insert (ResultType *, const ResultType *) {}};

template <typename ResultType, std::size_t N>
struct ThreefryInsert<ResultType, 2, N, true>
{
    static void insert (ResultType *state, const ResultType *ks)
    {
        state[0] += ks[i0_];
        state[1] += ks[i1_];
        state[1] += inc_;
    }

    private :

    static VSMC_CONSTEXPR const std::size_t inc_ = N / 4;
    static VSMC_CONSTEXPR const std::size_t i0_ = (inc_ + 0) % 3;
    static VSMC_CONSTEXPR const std::size_t i1_ = (inc_ + 1) % 3;
};

template <typename ResultType, std::size_t N>
struct ThreefryInsert<ResultType, 4, N, true>
{
    static void insert (ResultType *state, const ResultType *ks)
    {
        state[0] += ks[i0_];
        state[1] += ks[i1_];
        state[2] += ks[i2_];
        state[3] += ks[i3_];
        state[3] += inc_;
    }

    private :

    static VSMC_CONSTEXPR const std::size_t inc_ = N / 4;
    static VSMC_CONSTEXPR const std::size_t i0_ = (inc_ + 0) % 5;
    static VSMC_CONSTEXPR const std::size_t i1_ = (inc_ + 1) % 5;
    static VSMC_CONSTEXPR const std::size_t i2_ = (inc_ + 2) % 5;
    static VSMC_CONSTEXPR const std::size_t i3_ = (inc_ + 3) % 5;
};

} // namespace vsmc::internal

/// \brief Threefry RNG engine reimplemented
/// \ingroup R123RNG
///
/// \details
/// This is a reimplementation of the algorithm Threefry as described in
/// [Parallel Random Numbers: As Easy as 1, 2, 3][r123paper] and implemented in
/// [Random123][r123lib]. Depending on the compiler, it might be slightly
/// faster than the original implementation. I have observed a speedup at most
/// of two-folds when using Clang on a Haswell CPU. In some cases it is
/// slightly slower (but more close than the faster case).
///
/// [r123paper]:http://sc11.supercomputing.org/schedule/event_detail.php?evid=pap274
/// [r123lib]: https://www.deshawresearch.com/resources_random123.html
///
/// The implementation is almost identical to the original. Compared to
/// `r123:Engine<Threefry2x32>` etc., when using the default constructor of the
/// one with a single seed, the output shall be exactly the same for the first
/// \f$2^n\f$ iterations, where \f$n\f$ is the number of bits (32 or 64).
/// Further iterations may produce different results, as vSMC increment the
/// counter slightly differently, but it still cover the same range and has the
/// same period as the original.
template <typename ResultType, std::size_t K, std::size_t R = 20>
class ThreefryEngine
{
    public :

    typedef ResultType result_type;

    explicit ThreefryEngine (result_type s = 0) : remain_(0)
    {
        VSMC_STATIC_ASSERT_RNG_THREEFRY;
        seed(s);
    }

    template <typename SeedSeq>
    explicit ThreefryEngine (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_sequence<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR) : remain_(0)
    {
        VSMC_STATIC_ASSERT_RNG_THREEFRY;
        seed(seq);
    }

    ThreefryEngine (const ThreefryEngine<ResultType, K, R> &other) :
        remain_(other.remain_),
        key_(other.key_), ctr_(other.ctr_), res_(other.res_)
    {VSMC_STATIC_ASSERT_RNG_THREEFRY;}

    ThreefryEngine<ResultType, K, R> &operator= (
            const ThreefryEngine<ResultType, K, R> &other)
    {
        if (this != &other) {
            remain_ = other.remain_;
            key_ = other.key_;
            ctr_ = other.ctr_;
            res_ = other.res_;
        }

        return *this;
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    ThreefryEngine (ThreefryEngine<ResultType, K, R> &&other) :
        remain_(other.remain_),
        key_(cxx11::move(other.key_)), ctr_(cxx11::move(other.ctr_)),
        res_(cxx11::move(other.res_))
    {VSMC_STATIC_ASSERT_RNG_THREEFRY;}

    ThreefryEngine<ResultType, K, R> &operator= (
            ThreefryEngine<ResultType, K, R> &&other)
    {
        if (this != &other) {
            remain_ = other.remain_;
            key_ = cxx11::move(other.key_);
            ctr_ = cxx11::move(other.ctr_);
            res_ = cxx11::move(other.res_);
        }

        return *this;
    }
#endif

    void seed (result_type s)
    {
        remain_ = 0;
        key_.fill(0);
        ctr_.fill(0);
        res_.fill(0);
        key_[0] = s;
        init_ks();
    }

    template <typename SeedSeq>
    void seed (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_sequence<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR)
    {
        remain_ = 0;
        key_.fill(0);
        ctr_.fill(0);
        res_.fill(0);
        seq.generate(key_.begin(), key_.end());
        init_ks();
    }

    result_type operator() ()
    {
        if (remain_ > 0)
            return res_[--remain_];

        internal::ThreefryIncrement<ResultType, K>::increment(ctr_.data());
        res_ = ctr_;
        generate<0>(res_.data(), ks_.data(), cxx11::true_type());
        remain_ = K - 1;

        return res_[K - 1];
    }

    void discard (std::size_t nskip)
    {
        for (std::size_t i = 0; i != nskip; ++i)
            operator()();
    }

    static VSMC_CONSTEXPR const result_type _Min = 0;
    static VSMC_CONSTEXPR const result_type _Max = static_cast<result_type>(
            ~(static_cast<result_type>(0)));

    static VSMC_CONSTEXPR result_type min VSMC_MNE () {return _Min;}
    static VSMC_CONSTEXPR result_type max VSMC_MNE () {return _Max;}

    friend inline bool operator== (
            const ThreefryEngine<ResultType, K, R> &eng1,
            const ThreefryEngine<ResultType, K, R> &eng2)
    {
        return eng1.remain_ == eng2.remain_ &&
            eng1.key_ == eng2.key_ &&
            eng1.ctr_ == eng2.ctr_ &&
            eng1.res_ == eng2.res_;
    }

    friend inline bool operator!= (
            const ThreefryEngine<ResultType, K, R> &eng1,
            const ThreefryEngine<ResultType, K, R> &eng2)
    {return !(eng1 == eng2);}

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &operator<< (
            std::basic_ostream<CharT, Traits> &os,
            const ThreefryEngine<ResultType, K, R> &eng)
    {
        os << eng.remain_ << ' ';
        os << eng.key_ << ' ' << eng.ctr_ << ' ' << eng.res_ << ' ' << eng.ks_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &operator>> (
            std::basic_istream<CharT, Traits> &is,
            ThreefryEngine<ResultType, K, R> &eng)
    {
        ThreefryEngine eng_tmp;
        if (is) is >> eng_tmp.remain_;
        if (is) is >> eng_tmp.key_;
        if (is) is >> eng_tmp.ctr_;
        if (is) is >> eng_tmp.res_;
        if (is) is >> eng_tmp.ks_;
        if (is) {
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
            eng = cxx11::move(eng_tmp);
#else
            eng = eng_tmp;
#endif
        }

        return is;
    }

    private :

    std::size_t remain_;
    StaticVector<ResultType, K> key_;
    StaticVector<ResultType, K> ctr_;
    StaticVector<ResultType, K> res_;
    StaticVector<ResultType, K + 1> ks_;

    void init_ks ()
    {
        ks_[K] = internal::ThreefryParity<ResultType>::value;
        for (std::size_t i = 0; i != K; ++i) {
            ks_[i] = key_[i];
            ks_[K] ^= key_[i];
        }
    }

    template <std::size_t N>
    void generate (result_type *, result_type *, cxx11::false_type) {}

    template <std::size_t N>
    void generate (result_type *state, result_type *ks, cxx11::true_type)
    {
        internal::ThreefryRotate<ResultType, K, N>::rotate(state);
        internal::ThreefryInsert<ResultType, K, N>::insert(state, ks);
        generate<N + 1>(state, ks, cxx11::integral_constant<bool, N < R>());
    }
}; // class ThreefryEngine

/// \brief Threefry RNG engine re-implemented
/// \ingroup R123RNG
typedef ThreefryEngine<uint32_t, 2> Threefry2x32;
typedef ThreefryEngine<uint32_t, 4> Threefry4x32;
typedef ThreefryEngine<uint64_t, 2> Threefry2x64;
typedef ThreefryEngine<uint64_t, 4> Threefry4x64;

} // namespace vsmc

#endif // VSMC_RNG_THREEFRY_HPP
