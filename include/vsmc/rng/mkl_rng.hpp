#ifndef VSMC_RNG_MKL_RNG_HPP
#define VSMC_RNG_MKL_RNG_HPP

#include <vsmc/rng/rng_set.hpp>
#include <mkl_vsl.h>

#define VSMC_RUNTIME_ASSERT_RNG_MKL_OFFSET(offset) \
    VSMC_RUNTIME_ASSERT((offset < max VSMC_MACRO_NO_EXPANSION ()),           \
            ("**vsmc::mkl::OffsetDynamic** "                                 \
             "EXCESS MAXIMUM NUMBER OF INDEPDENT RNG STREAMS"))

#ifndef VSMC_RNG_MKL_BUFFER_SIZE
#define VSMC_RNG_MKL_BUFFER_SIZE 1000
#endif

namespace vsmc {

namespace mkl {

/// \cond HIDDEN_SYMBOLS

template <MKL_INT> class Stream;
template <typename, typename> class Distribution;
template <MKL_INT, typename> class Engine;

template <typename> class UniformBits;

template <MKL_INT M = VSL_RNG_METHOD_BERNOULLI_ICDF>      class Bernoulli;
template <MKL_INT M = VSL_RNG_METHOD_GEOMETRIC_ICDF>      class Geometric;
template <MKL_INT M = VSL_RNG_METHOD_BINOMIAL_BTPE>       class Binomial;
template <MKL_INT M = VSL_RNG_METHOD_HYPERGEOMETRIC_H2PE> class Hypergeometric;
template <MKL_INT M = VSL_RNG_METHOD_POISSON_PTPE>        class Poisson;
template <MKL_INT M = VSL_RNG_METHOD_NEGBINOMIAL_NBAR>    class NegBinomial;

template <typename, MKL_INT M = VSL_RNG_METHOD_UNIFORM_STD>  class Uniform;

template <typename F = double,
         MKL_INT M = VSL_RNG_METHOD_GAUSSIAN_BOXMULLER2>   class Gaussian;
template <typename F = double,
         MKL_INT M = VSL_RNG_METHOD_EXPONENTIAL_ICDF>      class Exponential;
template <typename F = double,
         MKL_INT M = VSL_RNG_METHOD_LAPLACE_ICDF>          class Laplace;
template <typename F = double,
         MKL_INT M = VSL_RNG_METHOD_WEIBULL_ICDF>          class Weibull;
template <typename F = double,
         MKL_INT M = VSL_RNG_METHOD_CAUCHY_ICDF>           class Cauchy;
template <typename F = double,
         MKL_INT M = VSL_RNG_METHOD_RAYLEIGH_ICDF>         class Rayleigh;
template <typename F = double,
         MKL_INT M = VSL_RNG_METHOD_LOGNORMAL_BOXMULLER2>  class Lognormal;
template <typename F = double,
         MKL_INT M = VSL_RNG_METHOD_GUMBEL_ICDF>           class Gumbel;
template <typename F = double,
         MKL_INT M = VSL_RNG_METHOD_GAMMA_GNORM>           class Gamma;
template <typename F = double,
         MKL_INT M = VSL_RNG_METHOD_BETA_CJA>              class Beta;

/// \endcond HIDDEN_SYMBOLS

#define VSMC_DEFINE_VSL_BRNG_CASE(BRNG) \
    case BRNG : return #BRNG ;

/// \brief Transfer MKL BRNG index to string
/// \ingroup MKLRNG
inline std::string rng_brng_str (MKL_INT BRNG)
{
    switch (BRNG) {
        VSMC_DEFINE_VSL_BRNG_CASE(VSL_BRNG_MCG31)
        VSMC_DEFINE_VSL_BRNG_CASE(VSL_BRNG_MCG59)
        VSMC_DEFINE_VSL_BRNG_CASE(VSL_BRNG_MRG32K3A)
        VSMC_DEFINE_VSL_BRNG_CASE(VSL_BRNG_R250)
        VSMC_DEFINE_VSL_BRNG_CASE(VSL_BRNG_MT19937)
        VSMC_DEFINE_VSL_BRNG_CASE(VSL_BRNG_SFMT19937)
        VSMC_DEFINE_VSL_BRNG_CASE(VSL_BRNG_MT2203)
        VSMC_DEFINE_VSL_BRNG_CASE(VSL_BRNG_WH)
        VSMC_DEFINE_VSL_BRNG_CASE(VSL_BRNG_SOBOL)
        VSMC_DEFINE_VSL_BRNG_CASE(VSL_BRNG_NIEDERR)
        VSMC_DEFINE_VSL_BRNG_CASE(VSL_BRNG_IABSTRACT)
        VSMC_DEFINE_VSL_BRNG_CASE(VSL_BRNG_DABSTRACT)
        VSMC_DEFINE_VSL_BRNG_CASE(VSL_BRNG_SABSTRACT)
        VSMC_DEFINE_VSL_BRNG_CASE(VSL_BRNG_NONDETERM)
        default : return "Unknown";
    }
}

#define VSMC_DEFINE_VSL_ERROR_CASE(STATUS) \
    case STATUS : return #STATUS ;

/// \brief Transfer MKL VSL error code to string
/// \ingroup MKLRNG
inline std::string rng_error_str (int status)
{
    switch (status) {
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_ERROR_OK)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_ERROR_BADARGS)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_ERROR_CPU_NOT_SUPPORTED)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_ERROR_FEATURE_NOT_IMPLEMENTED)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_ERROR_MEM_FAILURE)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_ERROR_NULL_PTR)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_ERROR_UNKNOWN)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_RNG_ERROR_BAD_FILE_FORMAT)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_RNG_ERROR_BAD_MEM_FORMAT)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_RNG_ERROR_BAD_NBITS)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_RNG_ERROR_BAD_NSEEDS)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_RNG_ERROR_BAD_STREAM)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_RNG_ERROR_BAD_STREAM_STATE_SIZE)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_RNG_ERROR_BAD_UPDATE)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_RNG_ERROR_BAD_WORD_SIZE)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_RNG_ERROR_BRNG_NOT_SUPPORTED)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_RNG_ERROR_BRNG_TABLE_FULL)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_RNG_ERROR_BRNGS_INCOMPATIBLE)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_RNG_ERROR_FILE_CLOSE)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_RNG_ERROR_FILE_OPEN)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_RNG_ERROR_FILE_READ)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_RNG_ERROR_FILE_WRITE)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_RNG_ERROR_INVALID_ABSTRACT_STREAM)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_RNG_ERROR_INVALID_BRNG_INDEX)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_RNG_ERROR_LEAPFROG_UNSUPPORTED)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_RNG_ERROR_NO_NUMBERS)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_RNG_ERROR_QRNG_PERIOD_ELAPSED)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_RNG_ERROR_SKIPAHEAD_UNSUPPORTED)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_RNG_ERROR_UNSUPPORTED_FILE_VER)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_RNG_ERROR_NONDETERM_NOT_SUPPORTED)
        VSMC_DEFINE_VSL_ERROR_CASE(VSL_RNG_ERROR_NONDETERM_NRETRIES_EXCEEDED)
        default :
            return "UNKNOWN";
    }
}

#ifndef NDEBUG
/// \brief Check MKL RNG error status
/// \ingroup MKLRNG
inline void rng_error_check (MKL_INT BRNG, int status,
        const char *func, const char *vslf)
{
    if (status == VSL_ERROR_OK)
        return;

    std::string msg("**vsmc::mkl::");
    msg += func;
    msg += " failure";
    msg += "; MKL function: ";
    msg += vslf;

    msg += "; BRNG: ";
    msg += rng_brng_str(BRNG);
    msg += "; Error code: ";
    msg += rng_error_str(status);

    VSMC_RUNTIME_ASSERT((status == VSL_ERROR_OK), msg.c_str());
} // error_check
#else
inline void rng_error_check (MKL_INT int, const char *, const char *) {}
#endif

namespace traits {

template<MKL_INT> struct SeedTrait
{static const MKL_UINT value = 101;};

template <> struct SeedTrait<VSL_BRNG_SOBOL>
{static const MKL_UINT value = 10;};

template <> struct SeedTrait<VSL_BRNG_NIEDERR>
{static const MKL_UINT value = 10;};

struct SkipAheadVSL
{
    typedef long long size_type;

    template <MKL_INT BRNG>
    void operator() (const Stream<BRNG> &stream, size_type nskip)
    {
        int status = ::vslSkipAheadStream(stream.ptr(), nskip);
        rng_error_check(BRNG, status,
                "SkipAheadVSL::skip", "vslSkipAheadStream");
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

    MKL_INT buffer_size_;
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

    explicit Stream (MKL_UINT s = traits::SeedTrait<BRNG>::value) : seed_(s)
    {
        int status = ::vslNewStream(&str_ptr_, BRNG + this->offset(), seed_);
        rng_error_check(BRNG, status, "Stream::Stream", "vslNewStream");
    }

    template <typename SeedSeq>
    explicit Stream (SeedSeq &seq)
    {
        seq.generate(&seed_, &seed_ + 1);
        int status = ::vslNewStream(&str_ptr_, BRNG + this->offset(), seed_);
        rng_error_check(BRNG, status, "Stream::Stream", "vslNewStream");
    }

    Stream (const Stream<BRNG> &other) : traits::OffsetTrait<BRNG>::type(other)
    {
        int status = ::vslCopyStream(&str_ptr_, other.str_ptr_);
        rng_error_check(BRNG, status, "Stream::Stream", "vslCopyStream");
    }

    Stream (Stream<BRNG> &other) : traits::OffsetTrait<BRNG>::type(other)
    {
        int status = ::vslCopyStream(&str_ptr_, other.str_ptr_);
        rng_error_check(BRNG, status, "Stream::Stream", "vslCopyStream");
    }

    Stream<BRNG> &operator= (const Stream<BRNG> &other)
    {
        traits::OffsetTrait<BRNG>::type::operator=(other);
        int status = ::vslCopyStreamState(str_ptr_, other.str_ptr_);
        rng_error_check(BRNG, status,
                "Stream::operator=", "vslCopyStreamState");
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    Stream (Stream<BRNG> &&other) :
        traits::OffsetTrait<BRNG>::type(std::move(other)),
        seed_(other.seed_), str_ptr_(other.str_ptr_) {}

    Stream<BRNG> &operator= (Stream<BRNG> &&other)
    {
        traits::OffsetTrait<BRNG>::type::operator=(std::move(other));
        seed_ = other.seed_;
        str_ptr_ = other.str_ptr_;
    }
#endif

    ~Stream () {::vslDeleteStream(&str_ptr_);}

    void seed (MKL_UINT s)
    {
        seed_ = s;
        int status = VSL_ERROR_OK;
        VSLStreamStatePtr new_str_ptr;

        status = ::vslNewStream(&new_str_ptr, BRNG + this->offset(), s);
        rng_error_check(BRNG, status, "Stream::seed", "vslNewStream");

        status = ::vslCopyStreamState(str_ptr_, new_str_ptr);
        rng_error_check(BRNG, status, "Stream::seed", "vslCopyStreamState");

        status = ::vslDeleteStream(&new_str_ptr);
        rng_error_check(BRNG, status, "Stream::seed", "vslDeleteStream");
    }

    template <typename SeedSeq>
    void seed (SeedSeq &seq)
    {
        seq.generate(&seed_, &seed_ + 1);
        seed(seed_);
    }

    VSLStreamStatePtr ptr () const {return str_ptr_;}

    void shift ()
    {
        unsigned offset = SeedGenerator<Stream<BRNG> >::instance().get();
        this->offset(static_cast<MKL_INT>(offset));
        seed(seed_);
    }

    private :

    MKL_UINT seed_;
    VSLStreamStatePtr str_ptr_;
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

    protected :

    template <MKL_INT BRNG>
    void generate_error_check (int status, const char *name)
    {
        if (status == VSL_ERROR_OK)
            return;

        std::string dist_name(name);
        dist_name += "::generate";
        std::string vsl_name(vsl_name_prefix(static_cast<result_type>(0)));
        vsl_name += "Rng";
        vsl_name += name;
        rng_error_check(BRNG, status, dist_name.c_str(), vsl_name.c_str());
    }

    private :

    MKL_INT remain_;
    MKL_INT buffer_size_;
    std::vector<result_type> result_;

    std::string vsl_name_prefix (MKL_INT)            {return "vi";}
    std::string vsl_name_prefix (unsigned)           {return "vi";}
    std::string vsl_name_prefix (unsigned MKL_INT64) {return "vi";}
    std::string vsl_name_prefix (float)              {return "vs";}
    std::string vsl_name_prefix (double)             {return "vd";}
}; // class Distribution

/// \brief MKL RNG C++11 engine
/// \ingroup MKLRNG
template <MKL_INT BRNG, typename ResultType>
class Engine
{
    public :

    typedef ResultType result_type;
    typedef Stream<BRNG> stream_type;
    typedef typename traits::SkipAheadTrait<BRNG, ResultType>::type
        skip_ahead_type;
    typedef UniformBits<result_type> runif_type;

    explicit Engine (MKL_UINT s = traits::SeedTrait<BRNG>::value) :
        stream_(s) {}

    template <typename SeedSeq>
    explicit Engine (SeedSeq &seq) : stream_(seq) {}

    Engine (const Engine<BRNG, ResultType> &other) :
        stream_(other.stream_), skip_ahead_(other.skip_ahead_) {}

    Engine (Engine<BRNG, ResultType> &other) :
        stream_(other.stream_), skip_ahead_(other.skip_ahead_) {}

    Engine<BRNG, ResultType> &operator= (const Engine<BRNG, ResultType> &other)
    {
        stream_ = other.stream_;
        skip_ahead_ = other.skip_ahead_;
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    Engine (Engine<BRNG, ResultType> &&other) :
        stream_(std::move(other.stream_)),
        skip_ahead_(std::move(other.skip_ahead_)) {}

    Engine<BRNG, ResultType> &operator= (Engine<BRNG, ResultType> &&other)
    {
        stream_ = std::move(other.stream_);
        skip_ahead_ = std::move(other.skip_ahead_);
    }
#endif

    void seed (MKL_UINT s)
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
    runif_type runif_;
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
        rng_error_check(BRNG, status,
                "UniformBits::generate", "viRngUniformBits32");
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
        rng_error_check(BRNG, status,
                "UniformBits::generate", "viRngUniformBits64");
    }
}; // class UniformBits

/// \brief MKL Bernoulli distribution
/// \ingroup MKLRNG
template <MKL_INT Method>
class Bernoulli : public Distribution<MKL_INT, Bernoulli<Method> >
{
    public :

    typedef MKL_INT result_type;

    explicit Bernoulli (double p = 0.5) : p_(p) {}

    template <MKL_INT BRNG>
    void generate (const Stream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = ::viRngBernoulli(Method, str.ptr(), n, r, p_);
        this->template generate_error_check<BRNG>(status, "Bernoulli");
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

    explicit Geometric (double p = 0.5) : p_(p) {}

    template <MKL_INT BRNG>
    void generate (const Stream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = ::viRngGeometric(Method, str.ptr(), n, r, p_);
        this->template generate_error_check<BRNG>(status, "Geometric");
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

    explicit Binomial (result_type ntrial = 1, double p = 0.5) :
        ntrial_(ntrial), p_(p) {}

    template <MKL_INT BRNG>
    void generate (const Stream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = ::viRngBinomial(Method, str.ptr(), n, r, ntrial_, p_);
        this->template generate_error_check<BRNG>(status, "Binomial");
    }

    public :

    result_type ntrial_;
    double p_;
}; // class Binomial

/// \brief MKL Hypergeometric distribution
/// \ingroup MKLRNG
template <MKL_INT Method>
class Hypergeometric : public Distribution<MKL_INT, Hypergeometric<Method> >
{
    public :

    typedef MKL_INT result_type;

    Hypergeometric (result_type population, result_type sample,
            result_type mask) :
        l_(population), s_(sample), m_(mask) {}

    template <MKL_INT BRNG>
    void generate (const Stream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = ::viRngHypergeometric(Method, str.ptr(), n, r, l_, s_, m_);
        this->template generate_error_check<BRNG>(status, "Hypergeometric");
    }

    public :

    result_type l_;
    result_type s_;
    result_type m_;
}; // class Hypergeometric

/// \brief MKL Poisson distribution
/// \ingroup MKLRNG
template <MKL_INT Method>
class Poisson : public Distribution<MKL_INT, Poisson<Method> >
{
    public :

    typedef MKL_INT result_type;

    explicit Poisson (double lambda = 1) : lambda_(lambda) {}

    template <MKL_INT BRNG>
    void generate (const Stream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = ::viRngPoisson(Method, str.ptr(), n, r, lambda_);
        this->template generate_error_check<BRNG>(status, "Poisson");
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

    explicit NegBinomial (double ntrial = 1, double p = 0.5) :
        ntrial_(ntrial), p_(p) {}

    template <MKL_INT BRNG>
    void generate (const Stream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = ::viRngNegBinomial(Method, str.ptr(), n, r, ntrial_, p_);
        this->template generate_error_check<BRNG>(status, "NegBinomial");
    }

    public :

    double ntrial_;
    double p_;
}; // class NegBinomial

/// \brief MKL Uniform distributions
/// \ingroup MKLRNG
template <typename ResultType, MKL_INT Method>
class Uniform : public Distribution<ResultType, Uniform<ResultType, Method> >
{
    public :

    typedef ResultType result_type;

    explicit Uniform (result_type a = 0, result_type b = 1) : a_(a), b_(b) {}

    template <MKL_INT BRNG>
    void generate (const Stream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = generate(str.ptr(), n, r);
        this->template generate_error_check<BRNG>(status, "Uniform");
    }

    private :

    result_type a_;
    result_type b_;

    int generate (VSLStreamStatePtr ptr, MKL_INT n, MKL_INT *r)
    {return ::viRngUniform(Method, ptr, n, r, a_, b_);}

    int generate (VSLStreamStatePtr ptr, MKL_INT n, float *r)
    {return ::vsRngUniform(Method, ptr, n, r, a_, b_);}

    int generate (VSLStreamStatePtr ptr, MKL_INT n, double *r)
    {return ::vdRngUniform(Method, ptr, n, r, a_, b_);}
}; // class Uniform

/// \brief MKL Gaussian distributions
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class Gaussian : public Distribution<FPType, Gaussian<FPType, Method> >
{
    public :

    typedef FPType result_type;

    explicit Gaussian (result_type mean = 0, result_type sd = 1) :
        mean_(mean), sd_(sd) {}

    template <MKL_INT BRNG>
    void generate (const Stream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = generate(str.ptr(), n, r);
        this->template generate_error_check<BRNG>(status, "Gaussian");
    }

    private :

    result_type mean_;
    result_type sd_;

    int generate (VSLStreamStatePtr ptr, MKL_INT n, float *r)
    {return ::vsRngGaussian(Method, ptr, n, r, mean_, sd_);}

    int generate (VSLStreamStatePtr ptr, MKL_INT n, double *r)
    {return ::vdRngGaussian(Method, ptr, n, r, mean_, sd_);}
}; // class Gaussian

/// \brief MKL Exponential distributions
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class Exponential : public Distribution<FPType, Exponential<FPType, Method> >
{
    public :

    typedef FPType result_type;

    explicit Exponential (
            result_type displacement = 0, result_type scale = 1) :
        disp_(displacement), scale_(scale) {}

    template <MKL_INT BRNG>
    void generate (const Stream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = generate(str.ptr(), n, r);
        this->template generate_error_check<BRNG>(status, "Exponential");
    }

    private :

    result_type disp_;
    result_type scale_;

    int generate (VSLStreamStatePtr ptr, MKL_INT n, float *r)
    {return ::vsRngExponential(Method, ptr, n, r, disp_, scale_);}

    int generate (VSLStreamStatePtr ptr, MKL_INT n, double *r)
    {return ::vdRngExponential(Method, ptr, n, r, disp_, scale_);}
}; // class Exponential

/// \brief MKL Laplace distributions
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class Laplace : public Distribution<FPType, Laplace<FPType, Method> >
{
    public :

    typedef FPType result_type;

    explicit Laplace (result_type mean = 0, result_type scale = 1) :
        mean_(mean), scale_(scale) {}

    template <MKL_INT BRNG>
    void generate (const Stream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = generate(str.ptr(), n, r);
        this->template generate_error_check<BRNG>(status, "Laplace");
    }

    private :

    result_type mean_;
    result_type scale_;

    int generate (VSLStreamStatePtr ptr, MKL_INT n, float *r)
    {return ::vsRngLaplace(Method, ptr, n, r, mean_, scale_);}

    int generate (VSLStreamStatePtr ptr, MKL_INT n, double *r)
    {return ::vdRngLaplace(Method, ptr, n, r, mean_, scale_);}
}; // class Laplace

/// \brief MKL Weibull distributions
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class Weibull : public Distribution<FPType, Weibull<FPType, Method> >
{
    public :

    typedef FPType result_type;

    explicit Weibull (result_type shape = 1,
            result_type displacement = 0, result_type scale = 1) :
        shape_(shape), disp_(displacement), scale_(scale) {}

    template <MKL_INT BRNG>
    void generate (const Stream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = generate(str.ptr(), n, r);
        this->template generate_error_check<BRNG>(status, "Weibull");
    }

    private :

    result_type shape_;
    result_type disp_;
    result_type scale_;

    int generate (VSLStreamStatePtr ptr, MKL_INT n, float *r)
    {return ::vsRngWeibull(Method, ptr, n, r, shape_, disp_, scale_);}

    int generate (VSLStreamStatePtr ptr, MKL_INT n, double *r)
    {return ::vdRngWeibull(Method, ptr, n, r, shape_, disp_, scale_);}
}; // class Weibull

/// \brief MKL Cauchy distributions
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class Cauchy : public Distribution<FPType, Cauchy<FPType, Method> >
{
    public :

    typedef FPType result_type;

    explicit Cauchy (result_type displacement = 0, result_type scale = 1) :
        disp_(displacement), scale_(scale) {}

    template <MKL_INT BRNG>
    void generate (const Stream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = generate(str.ptr(), n, r);
        this->template generate_error_check<BRNG>(status, "Cauchy");
    }

    private :

    result_type disp_;
    result_type scale_;

    int generate (VSLStreamStatePtr ptr, MKL_INT n, float *r)
    {return ::vsRngCauchy(Method, ptr, n, r, disp_, scale_);}

    int generate (VSLStreamStatePtr ptr, MKL_INT n, double *r)
    {return ::vdRngCauchy(Method, ptr, n, r, disp_, scale_);}
}; // class Cauchy

/// \brief MKL Rayleigh distributions
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class Rayleigh : public Distribution<FPType, Rayleigh<FPType, Method> >
{
    public :

    typedef FPType result_type;

    explicit Rayleigh (result_type displacement = 0, result_type scale = 1) :
        disp_(displacement), scale_(scale) {}

    template <MKL_INT BRNG>
    void generate (const Stream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = generate(str.ptr(), n, r);
        this->template generate_error_check<BRNG>(status, "Rayleigh");
    }

    private :

    result_type disp_;
    result_type scale_;

    int generate (VSLStreamStatePtr ptr, MKL_INT n, float *r)
    {return ::vsRngRayleigh(Method, ptr, n, r, disp_, scale_);}

    int generate (VSLStreamStatePtr ptr, MKL_INT n, double *r)
    {return ::vdRngRayleigh(Method, ptr, n, r, disp_, scale_);}
}; // class Rayleigh

/// \brief MKL Lognormal distributions
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class Lognormal : public Distribution<FPType, Lognormal<FPType, Method> >
{
    public :

    typedef FPType result_type;

    explicit Lognormal (result_type mean = 0, result_type sd = 1,
            result_type displacement = 0, result_type scale = 1) :
        mean_(mean), sd_(sd), disp_(displacement), scale_(scale) {}

    template <MKL_INT BRNG>
    void generate (const Stream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = generate(str.ptr(), n, r);
        this->template generate_error_check<BRNG>(status, "Lognormal");
    }

    private :

    result_type mean_;
    result_type sd_;
    result_type disp_;
    result_type scale_;

    int generate (VSLStreamStatePtr ptr, MKL_INT n, float *r)
    {return ::vsRngLognormal(Method, ptr, n, r, mean_, sd_, disp_, scale_);}

    int generate (VSLStreamStatePtr ptr, MKL_INT n, double *r)
    {return ::vdRngLognormal(Method, ptr, n, r, mean_, sd_, disp_, scale_);}
}; // class Lognormal

/// \brief MKL Gumbel distributions
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class Gumbel : public Distribution<FPType, Gumbel<FPType, Method> >
{
    public :

    typedef FPType result_type;

    explicit Gumbel (result_type displacement = 0, result_type scale = 1) :
        disp_(displacement), scale_(scale) {}

    template <MKL_INT BRNG>
    void generate (const Stream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = generate(str.ptr(), n, r);
        this->template generate_error_check<BRNG>(status, "Gumbel");
    }

    private :

    result_type disp_;
    result_type scale_;

    int generate (VSLStreamStatePtr ptr, MKL_INT n, float *r)
    {return ::vsRngGumbel(Method, ptr, n, r, disp_, scale_);}

    int generate (VSLStreamStatePtr ptr, MKL_INT n, double *r)
    {return ::vdRngGumbel(Method, ptr, n, r, disp_, scale_);}
}; // class Gumbel

/// \brief MKL Gamma distributions
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class Gamma : public Distribution<FPType, Gamma<FPType, Method> >
{
    public :

    typedef FPType result_type;

    explicit Gamma (result_type shape = 1,
            result_type displacement = 0, result_type scale = 1) :
        shape_(shape), disp_(displacement), scale_(scale) {}

    template <MKL_INT BRNG>
    void generate (const Stream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = generate(str.ptr(), n, r);
        this->template generate_error_check<BRNG>(status, "Gamma");
    }

    private :

    result_type shape_;
    result_type disp_;
    result_type scale_;

    int generate (VSLStreamStatePtr ptr, MKL_INT n, float *r)
    {return ::vsRngGamma(Method, ptr, n, r, shape_, disp_, scale_);}

    int generate (VSLStreamStatePtr ptr, MKL_INT n, double *r)
    {return ::vdRngGamma(Method, ptr, n, r, shape_, disp_, scale_);}
}; // class Gamma

/// \brief MKL Beta distributions
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class Beta : public Distribution<FPType, Beta<FPType, Method> >
{
    public :

    typedef FPType result_type;

    explicit Beta (result_type shape1 = 1, result_type shape2 = 1,
            result_type displacement = 0, result_type scale = 1) :
        shape1_(shape1), shape2_(shape2), disp_(displacement), scale_(scale) {}

    template <MKL_INT BRNG>
    void generate (const Stream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = generate(str.ptr(), n, r);
        this->template generate_error_check<BRNG>(status, "Beta");
    }

    private :

    result_type shape1_;
    result_type shape2_;
    result_type disp_;
    result_type scale_;

    int generate (VSLStreamStatePtr ptr, MKL_INT n, float *r)
    {return ::vsRngBeta(Method, ptr, n, r, shape1_, shape2_, disp_, scale_);}

    int generate (VSLStreamStatePtr ptr, MKL_INT n, double *r)
    {return ::vdRngBeta(Method, ptr, n, r, shape1_, shape2_, disp_, scale_);}
}; // class Beta

} // namespace vsmc::mkl

namespace traits {

template <MKL_INT BRNG, typename ResultType>
struct RngShift<mkl::Engine<BRNG, ResultType> >
{
    void operator() (mkl::Engine<BRNG, ResultType> &rng) const
    {rng.stream().shift();}
};

} // namespace vsmc::traits

} // namespace vsmc

#endif // VSMC_RNG_MKL_RNG_HPP
