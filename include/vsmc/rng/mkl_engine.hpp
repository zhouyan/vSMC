#ifndef VSMC_RNG_MKL_ENGINE_HPP
#define VSMC_RNG_MKL_ENGINE_HPP

#include <vsmc/internal/common.hpp>
#include <mkl_vsl.h>

#if VSMC_HAS_CXX11_THREAD_LOCAL && VSMC_HAS_CXX11LIB_MUTEX
#include <vsmc/rng/rng_set.hpp>
#endif

#define VSMC_RUNTIME_ASSERT_RNG_MKL_OFFSET(offset) \
    VSMC_RUNTIME_ASSERT((offset < max VSMC_MACRO_NO_EXPANSION ()),           \
            ("**vsmc::mkl::OffsetDynamic**"                                  \
             "EXCESS MAXIMUM NUMBER OF INDEPDENT RNG STREAMS"))

#ifndef VSMC_RNG_MKL_BUFFER_SIZE
#define VSMC_RNG_MKL_BUFFER_SIZE 1000
#endif

#ifndef VSMC_RNG_MKL_SEED
#define VSMC_RNG_MKL_SEED 861014
#endif

namespace vsmc {

namespace mkl {

/// \cond HIDDEN_SYMBOLS

struct ThreadLocalRng;
template <MKL_INT> class Stream;
template <typename, typename> class Distribution;
template <MKL_INT, typename, MKL_UINT S = VSMC_RNG_MKL_SEED> class Engine;

template <typename> class UniformBits;

template <MKL_INT M = VSL_RNG_METHOD_UNIFORM_STD>         class UniformInteger;
template <MKL_INT M = VSL_RNG_METHOD_BERNOULLI_ICDF>      class Bernoulli;
template <MKL_INT M = VSL_RNG_METHOD_GEOMETRIC_ICDF>      class Geometric;
template <MKL_INT M = VSL_RNG_METHOD_BINOMIAL_BTPE>       class Binomial;
template <MKL_INT M = VSL_RNG_METHOD_HYPERGEOMETRIC_H2PE> class HyperGeometric;
template <MKL_INT M = VSL_RNG_METHOD_POISSON_PTPE>        class Poisson;
template <MKL_INT M = VSL_RNG_METHOD_NEGBINOMIAL_NBAR>    class NegBinomial;

template <typename F = double,
         MKL_INT M = VSL_RNG_METHOD_UNIFORM_STD>           class UniformReal;
template <typename F = double,
         MKL_INT M = VSL_RNG_METHOD_GAUSSIAN_BOXMULLER2>   class Gaussian;
template <typename F = double,
         MKL_INT M = VSL_RNG_METHOD_GAUSSIANMV_BOXMULLER2> class GaussianMV;
template <typename F = double,
         MKL_INT M = VSL_RNG_METHOD_EXPONENTIAL_ICDF>      class Exponential;
template <typename F = double,
         MKL_INT M = VSL_RNG_METHOD_LAPLACE_ICDF>          class Laplace;
template <typename F = double,
         MKL_INT M = VSL_RNG_METHOD_WEIBULL_ICDF>          class WeiBull;
template <typename F = double,
         MKL_INT M = VSL_RNG_METHOD_CAUCHY_ICDF>           class Cauchy;
template <typename F = double,
         MKL_INT M = VSL_RNG_METHOD_RAYLEIGH_ICDF>         class Rayleigh;
template <typename F = double,
         MKL_INT M = VSL_RNG_METHOD_LOGNORMAL_BOXMULLER2>  class LogNormal;
template <typename F = double,
         MKL_INT M = VSL_RNG_METHOD_GUMBEL_ICDF>           class Gumbel;
template <typename F = double,
         MKL_INT M = VSL_RNG_METHOD_GAMMA_GNORM>           class Gamma;
template <typename F = double,
         MKL_INT M = VSL_RNG_METHOD_BETA_CJA>              class Beta;

/// \endcond HIDDEN_SYMBOLS

#ifndef NDEBUG
/// \brief Check MKL RNG error status
/// \ingroup MKLRNG
template <MKL_INT BRNG>
inline void rng_error_check (int status, const char *func, const char *vslf)
{
    if (status == VSL_ERROR_OK)
        return;

    std::string msg("**vsmc::mkl::");
    msg += func;
    msg += " failure";
    msg += "; MKL function: ";
    msg += vslf;

    msg += "; BRNG: ";
    switch (BRNG) {
        case VSL_BRNG_MCG59 :
            msg += "VSL_BRNG_MCG59";
            break;
        case VSL_BRNG_MT19937 :
            msg += "VSL_BRNG_MT19937";
            break;
        case VSL_BRNG_SFMT19937 :
            msg += "VSL_BRNG_SFMT19937";
            break;
        case VSL_BRNG_MT2203 :
            msg += "VSL_BRNG_MT2203";
            break;
        case VSL_BRNG_NONDETERM :
            msg += "VSL_BRNG_NONDETERM";
            break;
        default :
            msg += "UNKNOWN";
            break;
    } // switch (BRNG)

    msg += "; Error code: ";
    switch (status) {
        case VSL_ERROR_BADARGS :
            msg += "VSL_ERROR_BADARGS";
            break;
        case VSL_ERROR_CPU_NOT_SUPPORTED :
            msg += "VSL_ERROR_CPU_NOT_SUPPORTED";
            break;
        case VSL_ERROR_FEATURE_NOT_IMPLEMENTED :
            msg += "VSL_ERROR_FEATURE_NOT_IMPLEMENTED";
            break;
        case VSL_ERROR_MEM_FAILURE :
            msg += "VSL_ERROR_MEM_FAILURE";
            break;
        case VSL_ERROR_NULL_PTR :
            msg += "VSL_ERROR_NULL_PTR";
            break;
        case VSL_ERROR_UNKNOWN :
            msg += "VSL_ERROR_UNKNOWN";
            break;
        case VSL_RNG_ERROR_BAD_FILE_FORMAT :
            msg += "VSL_RNG_ERROR_BAD_FILE_FORMAT";
            break;
        case VSL_RNG_ERROR_BAD_MEM_FORMAT :
            msg += "VSL_RNG_ERROR_BAD_MEM_FORMAT";
            break;
        case VSL_RNG_ERROR_BAD_NBITS :
            msg += "VSL_RNG_ERROR_BAD_NBITS";
            break;
        case VSL_RNG_ERROR_BAD_NSEEDS :
            msg += "VSL_RNG_ERROR_BAD_NSEEDS";
            break;
        case VSL_RNG_ERROR_BAD_STREAM :
            msg += "VSL_RNG_ERROR_BAD_STREAM";
            break;
        case VSL_RNG_ERROR_BAD_STREAM_STATE_SIZE :
            msg += "VSL_RNG_ERROR_BAD_STREAM_STATE_SIZE";
            break;
        case VSL_RNG_ERROR_BAD_UPDATE :
            msg += "VSL_RNG_ERROR_BAD_UPDATE";
            break;
        case VSL_RNG_ERROR_BAD_WORD_SIZE :
            msg += "VSL_RNG_ERROR_BAD_WORD_SIZE";
            break;
        case VSL_RNG_ERROR_BRNG_NOT_SUPPORTED :
            msg += "VSL_RNG_ERROR_BRNG_NOT_SUPPORTED";
            break;
        case VSL_RNG_ERROR_BRNG_TABLE_FULL :
            msg += "VSL_RNG_ERROR_BRNG_TABLE_FULL";
            break;
        case VSL_RNG_ERROR_BRNGS_INCOMPATIBLE :
            msg += "VSL_RNG_ERROR_BRNGS_INCOMPATIBLE";
            break;
        case VSL_RNG_ERROR_FILE_CLOSE :
            msg += "VSL_RNG_ERROR_FILE_CLOSE";
            break;
        case VSL_RNG_ERROR_FILE_OPEN :
            msg += "VSL_RNG_ERROR_FILE_OPEN";
            break;
        case VSL_RNG_ERROR_FILE_READ :
            msg += "VSL_RNG_ERROR_FILE_READ";
            break;
        case VSL_RNG_ERROR_FILE_WRITE :
            msg += "VSL_RNG_ERROR_FILE_WRITE";
            break;
        case VSL_RNG_ERROR_INVALID_ABSTRACT_STREAM :
            msg += "VSL_RNG_ERROR_INVALID_ABSTRACT_STREAM";
            break;
        case VSL_RNG_ERROR_INVALID_BRNG_INDEX :
            msg += "VSL_RNG_ERROR_INVALID_BRNG_INDEX";
            break;
        case VSL_RNG_ERROR_LEAPFROG_UNSUPPORTED :
            msg += "VSL_RNG_ERROR_LEAPFROG_UNSUPPORTED";
            break;
        case VSL_RNG_ERROR_NO_NUMBERS :
            msg += "VSL_RNG_ERROR_NO_NUMBERS";
            break;
        case VSL_RNG_ERROR_QRNG_PERIOD_ELAPSED :
            msg += "VSL_RNG_ERROR_QRNG_PERIOD_ELAPSED";
            break;
        case VSL_RNG_ERROR_SKIPAHEAD_UNSUPPORTED :
            msg += "VSL_RNG_ERROR_SKIPAHEAD_UNSUPPORTED";
            break;
        case VSL_RNG_ERROR_UNSUPPORTED_FILE_VER :
            msg += "VSL_RNG_ERROR_UNSUPPORTED_FILE_VER";
            break;
        case VSL_RNG_ERROR_NONDETERM_NOT_SUPPORTED :
            msg += "VSL_RNG_ERROR_NONDETERM_NOT_SUPPORTED";
            break;
        case VSL_RNG_ERROR_NONDETERM_NRETRIES_EXCEEDED :
            msg += "VSL_RNG_ERROR_NONDETERM_ NRETRIES_EXCEEDED";
            break;
        default :
            msg += "UNKNOWN";
            break;
    } // switch (status)

    VSMC_RUNTIME_ASSERT((status == VSL_ERROR_OK), msg.c_str());
} // error_check
#else
template <MKL_INT BRNG>
inline void rng_error_check (int, const char *, const char *) {}
#endif

namespace traits {

struct SkipAheadVSL
{
    typedef long long size_type;

    template <MKL_INT BRNG>
    void operator() (const Stream<BRNG> &stream, size_type nskip)
    {
        int status = ::vslSkipAheadStream(stream.ptr(), nskip);
        rng_error_check<BRNG>(
                status, "SkipAheadVSL::skip", "vslSkipAheadStream");
    }

    static void buffer_size (MKL_INT) {}
    static MKL_INT buffer_size () {return 0;}
}; // struct SkipAheadVSL

template <typename ResultType>
struct SkipAheadForce
{
    typedef MKL_INT size_type;

    SkipAheadForce () : buffer_size_(VSMC_RNG_MKL_BUFFER_SIZE) {}

    template <MKL_INT BRNG>
    void operator() (const Stream<BRNG> &stream, size_type nskip)
    {
        if (nskip == 0)
            return;

        if (nskip < buffer_size_) {
            if (ruint_.size() < nskip)
                ruint_.resize(nskip);
            runif_(stream, nskip, &ruint_[0]);
        } else {
            if (ruint_.size() < buffer_size_)
                ruint_.resize(buffer_size_);
            size_type repeat = nskip / buffer_size_;
            size_type remain = nskip - repeat * buffer_size_;
            for (size_type r = 1; r != repeat + 1; ++r) {
                size_type n = r * buffer_size_;
                runif_(stream, n, &ruint_[0]);
            }
            runif_(stream, remain, &ruint_[0]);
        }
    }

    void buffer_size (MKL_INT size) {buffer_size_ = size;}
    MKL_INT buffer_size () {return buffer_size_;}

    private :

    MKL_INT buffer_size_ = VSMC_RNG_MKL_BUFFER_SIZE;
    std::vector<ResultType> ruint_;
    UniformBits<ResultType> runif_;
}; // strut SkipAheadForce

struct OffsetZero
{
    static VSMC_CONSTEXPR MKL_INT min VSMC_MACRO_NO_EXPANSION () {return 0;}
    static VSMC_CONSTEXPR MKL_INT max VSMC_MACRO_NO_EXPANSION () {return 0;}
    static void offset (MKL_INT) {}
    static VSMC_CONSTEXPR MKL_INT offset () {return 0;}
}; // struct OffsetZero

template <MKL_INT MaxOffset>
struct OffsetDynamic
{
    OffsetDynamic () : offset_(0) {}

    static VSMC_CONSTEXPR MKL_INT min VSMC_MACRO_NO_EXPANSION () {return 0;}
    static VSMC_CONSTEXPR MKL_INT max VSMC_MACRO_NO_EXPANSION ()
    {return MaxOffset;}

    void offset (MKL_INT n)
    {
        VSMC_RUNTIME_ASSERT_RNG_MKL_OFFSET(n);
        offset_ = n;
    }

    MKL_INT offset () const {return offset_;}

    private :

    MKL_INT offset_;
}; // struct OffsetDynamic

template <MKL_INT BRNG, typename ResultType>
struct SkipAheadTrait
{typedef SkipAheadForce<ResultType> type;};

template <typename ResultType>
struct SkipAheadTrait<VSL_BRNG_MCG31, ResultType>
{typedef SkipAheadVSL type;};

template <typename ResultType>
struct SkipAheadTrait<VSL_BRNG_MCG59, ResultType>
{typedef SkipAheadVSL type;};

template <typename ResultType>
struct SkipAheadTrait<VSL_BRNG_MRG32K3A, ResultType>
{typedef SkipAheadVSL type;};

template <typename ResultType>
struct SkipAheadTrait<VSL_BRNG_SOBOL, ResultType>
{typedef SkipAheadVSL type;};

template <typename ResultType>
struct SkipAheadTrait<VSL_BRNG_NIEDERR, ResultType>
{typedef SkipAheadVSL type;};

template <MKL_INT> struct OffsetTrait {typedef OffsetZero type;};

template <>
struct OffsetTrait<VSL_BRNG_MT2203> {typedef OffsetDynamic<6024> type;};

template <>
struct OffsetTrait<VSL_BRNG_WH> {typedef OffsetDynamic<273> type;};

} // namespace vsmc::mkl::traits

/// \brief MKL RNG C++11 engine stream
/// \ingroup MKLRNG
template <MKL_INT BRNG>
class Stream : public traits::OffsetTrait<BRNG>::type
{
    public :

    explicit Stream (MKL_UINT seed)
    {
        int status = ::vslNewStream(&stream_, BRNG + this->offset(), seed);
        rng_error_check<BRNG>(status, "Stream::Stream", "vslNewStream");
    }

    template <typename SeedSeq>
    explicit Stream (SeedSeq &seq)
    {
        MKL_UINT seed = 0;
        seq.generate(&seed, &seed + 1);
        int status = ::vslNewStream(&stream_, BRNG + this->offset(), seed);
        rng_error_check<BRNG>(status, "Stream::Stream", "vslNewStream");
    }

    Stream (const Stream<BRNG> &other)
    {
        int status = ::vslCopyStream(&stream_, other.stream_);
        rng_error_check<BRNG>(status, "Stream::Stream", "vslCopyStream");
    }

    Stream<BRNG> &operator= (const Stream<BRNG> &other)
    {
        int status = ::vslCopyStreamState(stream_, other.stream_);
        rng_error_check<BRNG>(
                status, "Stream::operator=", "vslCopyStreamState");
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    Stream (Stream<BRNG> &&other) : stream_(other.stream_) {}

    Stream<BRNG> &operator= (Stream<BRNG> &&other)
    {stream_ = other.stream_;}
#endif

    ~Stream () {::vslDeleteStream(&stream_);}

    void seed (MKL_UINT s)
    {
        int status = VSL_ERROR_OK;
        VSLStreamStatePtr new_stream;
        status = ::vslNewStream(&new_stream, BRNG + this->offset(), s);
        rng_error_check<BRNG>(status, "Stream::seed", "vslNewStream");
        status = ::vslCopyStreamState(stream_, new_stream);
        rng_error_check<BRNG>(status, "Stream::seed", "vslCopyStreamState");
        status = ::vslDeleteStream(&new_stream);
        rng_error_check<BRNG>(status, "Stream::seed", "vslDeleteStream");
    }

    template <typename SeedSeq>
    void seed (SeedSeq &seq)
    {
        MKL_UINT s = 0;
        seq.generate(&s, &s + 1);
        seed(s);
    }

    VSLStreamStatePtr ptr () const {return stream_;}

    private :

    VSLStreamStatePtr stream_;
}; // class Stream

/// \brief Base class of MKL distributions
/// \ingroup MKLRNG
template <typename ResultType, typename Derived>
class Distribution
{
    public :

    typedef ResultType result_type;

    Distribution () : remain_(0), buffer_size_(VSMC_RNG_MKL_BUFFER_SIZE) {}

    template <MKL_INT BRNG>
    result_type operator() (const Stream<BRNG> &stream)
    {
        result_.resize(buffer_size_);
        result_type *const rptr = &result_[0];
        if (remain_ > 0) {
            --remain_;
        } else {
            static_cast<Derived *>(this)->generate(stream, buffer_size_, rptr);
            remain_ = buffer_size_ - 1;
        }

        return rptr[remain_];
    }

    template <MKL_INT BRNG>
    void operator() (const Stream<BRNG> &stream, MKL_INT n, result_type *r)
    {static_cast<Derived *>(this)->generate(stream, n, r);}

    void reset () {remain_ = 0;}
    void buffer_size (MKL_INT size) {buffer_size_ = size;}
    MKL_INT buffer_size () const {return buffer_size_;}

    private :

    MKL_INT remain_;
    MKL_INT buffer_size_;
    std::vector<result_type> result_;
}; // class Distribution

/// \brief MKL RNG C++11 engine
/// \ingroup MKLRNG
template <MKL_INT BRNG, typename ResultType, MKL_UINT Seed>
class Engine
{
    public :

    typedef ResultType result_type;
    typedef Stream<BRNG> stream_type;
    typedef typename traits::SkipAheadTrait<BRNG, ResultType>::type
        skip_ahead_type;
    typedef UniformBits<result_type> runif_type;

    explicit Engine (MKL_UINT seed = Seed) : stream_(seed) {}

    template <typename SeedSeq>
    explicit Engine (SeedSeq &seq) : stream_(seq) {}

    void seed (MKL_UINT s = Seed)
    {
        stream_.seed(s);
        runif_.reset();
    }

    template <typename SeedSeq>
    void seed (SeedSeq &seq)
    {
        stream_.seed(seq);
        runif_.reset();
    }

    result_type operator() () {return runif_(stream_);}

    void discard (std::size_t nskip)
    {
        skip_ahead_(stream_,
                static_cast<typename skip_ahead_type::size_type>(nskip));
        runif_.reset();
    }

    static const result_type _Min = 0;
    static const result_type _Max = ~((result_type)0);

    static VSMC_CONSTEXPR result_type min VSMC_MACRO_NO_EXPANSION ()
    {return _Min;}

    static VSMC_CONSTEXPR result_type max VSMC_MACRO_NO_EXPANSION ()
    {return _Max;}

    stream_type &stream () {return stream_;}
    const stream_type &stream () const {return stream_;}

    skip_ahead_type &skip_ahead () {return skip_ahead_;}
    const skip_ahead_type &skip_ahead () const {return skip_ahead_;}

    runif_type &runif () {return runif_;}
    const runif_type &runif () const {return runif_;}

    private :

    stream_type stream_;
    skip_ahead_type skip_ahead_;
    UniformBits<result_type> runif_;
}; // class Engine

/// \brief A 59-bit multiplicative congruential generator
/// \ingroup MKLRNG
typedef Engine<VSL_BRNG_MCG59, unsigned> MCG59;

/// \brief A Mersenne-Twister pseudoranom number genertor
/// \ingroup MKLRNG
typedef Engine<VSL_BRNG_MT19937, unsigned> MT19937;

/// \brief A Mersenne-Twister pseudoranom number genertor (64-bit)
/// \ingroup MKLRNG
typedef Engine<VSL_BRNG_MT19937, unsigned MKL_INT64> MT19937_64;

/// \brief A SIMD-oriented fast Mersenne-Twister pseudoranom number genertor
/// \ingroup MKLRNG
typedef Engine<VSL_BRNG_SFMT19937, unsigned> SFMT19937;

/// \brief A SIMD-oriented fast Mersenne-Twister pseudoranom number genertor
/// (64-bit)
/// \ingroup MKLRNG
typedef Engine<VSL_BRNG_SFMT19937, unsigned MKL_INT64> SFMT19937_64;

/// \brief A set of 6024 Mersenne-Twister pseudoranom number genertor
/// \ingroup MKLRNG
typedef Engine<VSL_BRNG_MT2203, unsigned> MT2203;

/// \brief A set of 6024 Mersenne-Twister pseudoranom number genertor (64-bit)
/// \ingroup MKLRNG
typedef Engine<VSL_BRNG_MT2203, unsigned MKL_INT64>MT2203_64;

/// \brief A non-determinstic random number generator
/// \ingroup MKLRNG
typedef Engine<VSL_BRNG_NONDETERM, unsigned> NONDETERM;

/// \brief A non-determinstic random number generator (64-bit)
/// \ingroup MKLRNG
typedef Engine<VSL_BRNG_NONDETERM, unsigned MKL_INT64> NONDETERM_64;

} // namespace vsmc::mkl

#if VSMC_HAS_CXX11_THREAD_LOCAL && VSMC_HAS_CXX11LIB_MUTEX

/// \brief Vector RNG set using MKL BRNG with thread-local storage
/// \ingroup MKLRNG
template <MKL_INT BRNG, typename ResultType, MKL_UINT Seed>
class RngSet<mkl::Engine<BRNG, ResultType, Seed>, mkl::ThreadLocalRng>
{
    public :

    typedef mkl::Engine<BRNG, ResultType, Seed> rng_type;
    typedef std::size_t size_type;

    explicit RngSet (size_type N = 1) : size_(N) {}

    size_type size () const {return size_;}

    rng_type &rng (size_type id = 0)
    {
        static thread_local rng_type tl_rng;
        static thread_local bool tl_flag = 0;
        if (!tl_flag)
            init_rng(tl_rng, tl_flag);

        return tl_rng;
    }

    private :

    std::size_t size_;
    std::mutex mtx_;

    void init_rng (rng_type &tl_rng, bool &tl_flag)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        unsigned offset = SeedGenerator<rng_type>::instance().get();
        tl_rng.stream().offset(static_cast<MKL_INT>(offset));
        tl_rng.seed(static_cast<MKL_UINT>(Seed::instance().get()));
        tl_flag = true;
    }
}; // class RngSet

#endif // VSMC_HAS_CXX11_THREAD_LOCAL && VSMC_HAS_CXX11LIB_MUTEX

namespace mkl {

/// \brief MKL uniform bits (32-bits)
/// \ingroup MKLRNG
template <>
class UniformBits<unsigned> :
    public Distribution<unsigned, UniformBits<unsigned> >
{
    public :

    typedef unsigned result_type;

    template <MKL_INT BRNG>
    void generate (const Stream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = ::viRngUniformBits32(VSL_RNG_METHOD_UNIFORMBITS32_STD,
                str.ptr(), n, r);
        rng_error_check<BRNG>(
                status, "UniformBits::generate", "viRngUniformBits32");
    }
}; // class UniformBits

/// \brief MKL uniform bits (64-bits)
/// \ingroup MKLRNG
template <>
class UniformBits<unsigned MKL_INT64> :
    public Distribution<unsigned MKL_INT64, UniformBits<unsigned MKL_INT64> >
{
    public :

    typedef unsigned MKL_INT64 result_type;

    template <MKL_INT BRNG>
    void generate (const Stream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = ::viRngUniformBits64(VSL_RNG_METHOD_UNIFORMBITS64_STD,
                str.ptr(), n, r);
        rng_error_check<BRNG>(
                status, "UniformBits::generate", "viRngUniformBits64");
    }
}; // class UniformBits

/// \brief MKL uniform integer distribution
/// \ingroup MKLRNG
template <MKL_INT Method>
class UniformInteger : public Distribution<MKL_INT, UniformInteger<Method> >
{
    public :

    typedef MKL_INT result_type;

    UniformInteger (result_type a, result_type b) : a_(a), b_(b) {}

    template <MKL_INT BRNG>
    void generate (const Stream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = ::viRngUniform(Method, str.ptr(), n, r, a_, b_);
        rng_error_check<BRNG>(
                status, "UniformIntger::generate", "viRngUniform");
    }

    public :

    result_type a_;
    result_type b_;
}; // class UniformInteger

/// \brief MKL Bernoulli distribution
/// \ingroup MKLRNG
template <MKL_INT Method>
class Bernoulli : public Distribution<MKL_INT, Bernoulli<Method> >
{
    public :

    typedef MKL_INT result_type;

    Bernoulli (double p = 0.5) : p_(p) {}

    template <MKL_INT BRNG>
    void generate (const Stream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = ::viRngBernoulli(Method, str.ptr(), n, r, p_);
        rng_error_check<BRNG>(
                status, "Bernoulli::generate", "viRngBernoulli");
    }

    public :

    double p_;
}; // class Bernoulli

/// \brief MKL Geometric distribution
/// \ingroup MKLRNG
template <MKL_INT Method>
class Geometric : public Distribution<MKL_INT, Geometric<Method> >
{
    public :

    typedef MKL_INT result_type;

    Geometric (double p = 0.5) : p_(p) {}

    template <MKL_INT BRNG>
    void generate (const Stream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = ::viRngGeometric(Method, str.ptr(), n, r, p_);
        rng_error_check<BRNG>(
                status, "Geometric::generate", "viRngGeometric");
    }

    public :

    double p_;
}; // class Geometric

/// \brief MKL Binomial distribution
/// \ingroup MKLRNG
template <MKL_INT Method>
class Binomial : public Distribution<MKL_INT, Binomial<Method> >
{
    public :

    typedef MKL_INT result_type;

    Binomial (result_type ntrial, double p = 0.5) : ntrial_(ntrial), p_(p) {}

    template <MKL_INT BRNG>
    void generate (const Stream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = ::viRngBinomial(Method, str.ptr(), n, r, ntrial_, p_);
        rng_error_check<BRNG>(
                status, "Binomial::generate", "viRngBinomial");
    }

    public :

    result_type ntrial_;
    double p_;
}; // class Binomial

/// \brief MKL HyperGeometric distribution
/// \ingroup MKLRNG
template <MKL_INT Method>
class HyperGeometric : public Distribution<MKL_INT, HyperGeometric<Method> >
{
    public :

    typedef MKL_INT result_type;

    HyperGeometric (result_type l, result_type s, result_type m) :
        l_(l), s_(s), m_(m) {}

    template <MKL_INT BRNG>
    void generate (const Stream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = ::viRngHypergeometric(Method, str.ptr(), n, r, l_, s_, m_);
        rng_error_check<BRNG>(
                status, "HyperGeometric::generate", "viRngHypergeometric");
    }

    public :

    result_type l_;
    result_type s_;
    result_type m_;
}; // class HyperGeometric

/// \brief MKL Poisson distribution
/// \ingroup MKLRNG
template <MKL_INT Method>
class Poisson : public Distribution<MKL_INT, Poisson<Method> >
{
    public :

    typedef MKL_INT result_type;

    Poisson (double lambda = 1) : lambda_(lambda) {}

    template <MKL_INT BRNG>
    void generate (const Stream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = ::viRngPoisson(Method, str.ptr(), n, r, lambda_);
        rng_error_check<BRNG>(
                status, "Poisson::generate", "viRngPoisson");
    }

    public :

    double lambda_;
}; // class Poisson

/// \brief MKL NegBinomial distribution
/// \ingroup MKLRNG
template <MKL_INT Method>
class NegBinomial : public Distribution<MKL_INT, NegBinomial<Method> >
{
    public :

    typedef MKL_INT result_type;

    NegBinomial (double first, double p = 0.5) : first_(first), p_(p) {}

    template <MKL_INT BRNG>
    void generate (const Stream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = ::viRngNegBinomial(Method, str.ptr(), n, r, first_, p_);
        rng_error_check<BRNG>(
                status, "NegBinomial::generate", "viRngNegBinomial");
    }

    public :

    double first_;
    double p_;
}; // class NegBinomial

} // namespace vsmc::mkl

} // namespace vsmc

#endif // VSMC_RNG_MKL_ENGINE_HPP
