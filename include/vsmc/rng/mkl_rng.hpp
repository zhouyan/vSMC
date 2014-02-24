#ifndef VSMC_RNG_MKL_RNG_HPP
#define VSMC_RNG_MKL_RNG_HPP

#include <vsmc/rng/seed.hpp>
#include <mkl_vsl.h>

#define VSMC_STATIC_ASSERT_RNG_MKL_RNG_DISTRIBUTION_FPTYPE(FPType, Dist) \
    VSMC_STATIC_ASSERT(                                                      \
            (::vsmc::cxx11::is_same<FPType, float>::value ||                 \
             ::vsmc::cxx11::is_same<FPType, double>::value),                 \
            USE_MKL##Dist##Distribution_##WITH_A_RESULT_TYPE_OTHER_THAN_float_OR_double)

#define VSMC_RUNTIME_ASSERT_RNG_MKL_RNG_OFFSET(offset) \
    VSMC_RUNTIME_ASSERT((offset < max VSMC_MNE ()),                          \
            ("**MKLOffsetDynamic** "                                         \
             "EXCESS MAXIMUM NUMBER OF INDEPDENT RNG STREAMS"))

#ifndef VSMC_RNG_MKL_BUFFER_SIZE
#define VSMC_RNG_MKL_BUFFER_SIZE 1000
#endif

namespace vsmc {

template <MKL_INT>            class MKLStream;
template <typename, typename> class MKLDistribution;
template <MKL_INT, typename>  class MKLEngine;
template <typename>           class MKLUniformBitsDistribution;

template <MKL_INT = VSL_RNG_METHOD_BERNOULLI_ICDF>
class MKLBernoulliDistribution;
template <MKL_INT = VSL_RNG_METHOD_GEOMETRIC_ICDF>
class MKLGeometricDistribution;
template <MKL_INT = VSL_RNG_METHOD_BINOMIAL_BTPE>
class MKLBinomialDistribution;
template <MKL_INT = VSL_RNG_METHOD_HYPERGEOMETRIC_H2PE>
class MKLHypergeometricDistribution;
template <MKL_INT = VSL_RNG_METHOD_POISSON_PTPE>
class MKLPoissonDistribution;
template <MKL_INT = VSL_RNG_METHOD_NEGBINOMIAL_NBAR>
class MKLNegBinomialDistribution;
template <typename, MKL_INT = VSL_RNG_METHOD_UNIFORM_STD>
class MKLUniformDistribution;
template <typename = double, MKL_INT = VSL_RNG_METHOD_GAUSSIAN_BOXMULLER2>
class MKLGaussianDistribution;
template <typename = double, MKL_INT = VSL_RNG_METHOD_EXPONENTIAL_ICDF>
class MKLExponentialDistribution;
template <typename = double, MKL_INT = VSL_RNG_METHOD_LAPLACE_ICDF>
class MKLLaplaceDistribution;
template <typename = double, MKL_INT = VSL_RNG_METHOD_WEIBULL_ICDF>
class MKLWeibullDistribution;
template <typename = double, MKL_INT = VSL_RNG_METHOD_CAUCHY_ICDF>
class MKLCauchyDistribution;
template <typename = double, MKL_INT = VSL_RNG_METHOD_RAYLEIGH_ICDF>
class MKLRayleighDistribution;
template <typename = double, MKL_INT = VSL_RNG_METHOD_LOGNORMAL_BOXMULLER2>
class MKLLognormalDistribution;
template <typename = double, MKL_INT = VSL_RNG_METHOD_GUMBEL_ICDF>
class MKLGumbelDistribution;
template <typename = double, MKL_INT = VSL_RNG_METHOD_GAMMA_GNORM>
class MKLGammaDistribution;
template <typename = double, MKL_INT = VSL_RNG_METHOD_BETA_CJA>
class MKLBetaDistribution;

#define VSMC_DEFINE_VSL_BRNG_CASE(BRNG) \
    case BRNG : return #BRNG ;

/// \brief Transfer MKL BRNG index to string
/// \ingroup MKLRNG
inline std::string mkl_rng_brng_str (MKL_INT BRNG)
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

/// \brief Transfer MKL error code to string
/// \ingroup MKLRNG
inline std::string mkl_rng_error_str (int status)
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
inline void mkl_rng_error_check (MKL_INT BRNG, int status,
        const char *func, const char *vslf)
{
    if (status == VSL_ERROR_OK)
        return;

    std::string msg("**vsmc::");
    msg += func;
    msg += " failure";
    msg += "; MKL function: ";
    msg += vslf;

    msg += "; BRNG: ";
    msg += mkl_rng_brng_str(BRNG);
    msg += "; Error code: ";
    msg += mkl_rng_error_str(status);

    VSMC_RUNTIME_ASSERT((status == VSL_ERROR_OK), msg.c_str());
} // error_check
#else
inline void mkl_rng_error_check (MKL_INT, int, const char *, const char *) {}
#endif

namespace traits {

/// \brief Default Seed for MKL RNG
/// \ingroup Traits
template<MKL_INT> struct MKLSeedTrait
{static VSMC_CONSTEXPR const MKL_UINT value = 101;};

template <> struct MKLSeedTrait<VSL_BRNG_SOBOL>
{static VSMC_CONSTEXPR const MKL_UINT value = 10;};

template <> struct MKLSeedTrait<VSL_BRNG_NIEDERR>
{static VSMC_CONSTEXPR const MKL_UINT value = 10;};

/// \brief Skip ahead algorithm for MKL RNG using VSL function
/// \ingroup Traits
struct MKLSkipAheadVSL
{
    typedef long long size_type;

    template <MKL_INT BRNG>
    void operator() (const MKLStream<BRNG> &stream, size_type nskip)
    {
        int status = ::vslSkipAheadStream(stream.ptr(), nskip);
        mkl_rng_error_check(BRNG, status,
                "MKLSkipAheadVSL::skip", "vslSkipAheadStream");
    }

    static void buffer_size (MKL_INT) {}
    static MKL_INT buffer_size () {return 0;}
}; // struct SkipAheadVSL

/// \brief Skip ahead algorithm for MKL RNG using brute-force
/// \ingroup Traits
template <typename ResultType>
struct MKLSkipAheadForce
{
    typedef MKL_INT size_type;

    MKLSkipAheadForce () : buffer_size_(VSMC_RNG_MKL_BUFFER_SIZE) {}

    MKLSkipAheadForce (const MKLSkipAheadForce &other) :
        buffer_size_(other.buffer_size_) {}

    MKLSkipAheadForce &operator= (const MKLSkipAheadForce &other)
    {buffer_size_ = other.buffer_size_;}

    template <MKL_INT BRNG>
    void operator() (const MKLStream<BRNG> &stream, size_type nskip)
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
    MKLUniformBitsDistribution<ResultType> runif_;
}; // strut SkipAheadForce

/// \brief Skip ahead algorithm for MKL RNG
/// \ingroup Traits
template <MKL_INT BRNG, typename ResultType>
struct MKLSkipAheadTrait
{typedef MKLSkipAheadForce<ResultType> type;};

template <typename ResultType>
struct MKLSkipAheadTrait<VSL_BRNG_MCG31, ResultType>
{typedef MKLSkipAheadVSL type;};

template <typename ResultType>
struct MKLSkipAheadTrait<VSL_BRNG_MCG59, ResultType>
{typedef MKLSkipAheadVSL type;};

template <typename ResultType>
struct MKLSkipAheadTrait<VSL_BRNG_MRG32K3A, ResultType>
{typedef MKLSkipAheadVSL type;};

template <typename ResultType>
struct MKLSkipAheadTrait<VSL_BRNG_SOBOL, ResultType>
{typedef MKLSkipAheadVSL type;};

template <typename ResultType>
struct MKLSkipAheadTrait<VSL_BRNG_NIEDERR, ResultType>
{typedef MKLSkipAheadVSL type;};

} // namespace vsmc::traits

namespace internal {

struct MKLOffsetZero
{
    static VSMC_CONSTEXPR MKL_INT min VSMC_MNE () {return 0;}
    static VSMC_CONSTEXPR MKL_INT max VSMC_MNE () {return 0;}
    static void offset (MKL_INT) {}
    static VSMC_CONSTEXPR MKL_INT offset () {return 0;}
}; // struct OffsetZero

template <MKL_INT MaxOffset>
struct MKLOffsetDynamic
{
    MKLOffsetDynamic () : offset_(0) {}

    static VSMC_CONSTEXPR MKL_INT min VSMC_MNE () {return 0;}
    static VSMC_CONSTEXPR MKL_INT max VSMC_MNE () {return MaxOffset;}

    void offset (MKL_INT n)
    {
        VSMC_RUNTIME_ASSERT_RNG_MKL_RNG_OFFSET(n);
        offset_ = n;
    }

    MKL_INT offset () const {return offset_;}

    private :

    MKL_INT offset_;
}; // struct OffsetDynamic

template <MKL_INT> struct MKLOffset {typedef MKLOffsetZero type;};

template <>
struct MKLOffset<VSL_BRNG_MT2203> {typedef MKLOffsetDynamic<6024> type;};

template <>
struct MKLOffset<VSL_BRNG_WH> {typedef MKLOffsetDynamic<273> type;};
} // namespace vsmc::internal

/// \brief MKL RNG C++11 engine stream
/// \ingroup MKLRNG
template <MKL_INT BRNG>
class MKLStream : public internal::MKLOffset<BRNG>::type
{
    public :

    explicit MKLStream (MKL_UINT s = traits::MKLSeedTrait<BRNG>::value) :
        seed_(s)
    {
        int status = ::vslNewStream(&str_ptr_, BRNG + this->offset(), seed_);
        mkl_rng_error_check(BRNG, status,
                "MKLStream::Stream", "vslNewStream");
    }

    template <typename SeedSeq>
    explicit MKLStream (SeedSeq &seq)
    {
        seq.generate(&seed_, &seed_ + 1);
        int status = ::vslNewStream(&str_ptr_, BRNG + this->offset(), seed_);
        mkl_rng_error_check(BRNG, status,
                "MKLStream::Stream", "vslNewStream");
    }

    MKLStream (const MKLStream<BRNG> &other) :
        internal::MKLOffset<BRNG>::type(other)
    {
        int status = ::vslCopyStream(&str_ptr_, other.str_ptr_);
        mkl_rng_error_check(BRNG, status,
                "MKLStream::Stream", "vslCopyStream");
    }

    MKLStream (MKLStream<BRNG> &other) :
        internal::MKLOffset<BRNG>::type(other)
    {
        int status = ::vslCopyStream(&str_ptr_, other.str_ptr_);
        mkl_rng_error_check(BRNG, status,
                "MKLStream::Stream", "vslCopyStream");
    }

    MKLStream<BRNG> &operator= (const MKLStream<BRNG> &other)
    {
        if (this != &other) {
            internal::MKLOffset<BRNG>::type::operator=(other);
            int status = ::vslCopyStreamState(str_ptr_, other.str_ptr_);
            mkl_rng_error_check(BRNG, status,
                    "MKLStream::operator=", "vslCopyStreamState");
        }

        return *this;
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    MKLStream (MKLStream<BRNG> &&other) :
        internal::MKLOffset<BRNG>::type(cxx11::move(other)),
        seed_(other.seed_), str_ptr_(other.str_ptr_)
    {other.str_ptr_ = VSMC_NULLPTR;}

    MKLStream<BRNG> &operator= (MKLStream<BRNG> &&other)
    {
        if (this != other) {
            internal::MKLOffset<BRNG>::type::operator=(cxx11::move(other));
            seed_ = other.seed_;
            str_ptr_ = other.str_ptr_;
            other.str_ptr_ = VSMC_NULLPTR;
        }

        return *this;
    }
#endif

    ~MKLStream () {if (!empty()) ::vslDeleteStream(&str_ptr_);}

    bool empty () const
    {
        if (str_ptr_)
            return false;
        else
            return true;
    }

    void seed (MKL_UINT s)
    {
        seed_ = s;
        int status = VSL_ERROR_OK;
        VSLStreamStatePtr new_str_ptr;

        status = ::vslNewStream(&new_str_ptr, BRNG + this->offset(), s);
        mkl_rng_error_check(BRNG, status,
                "MKLStream::seed", "vslNewStream");

        status = ::vslCopyStreamState(str_ptr_, new_str_ptr);
        mkl_rng_error_check(BRNG, status,
                "MKLStream::seed", "vslCopyStreamState");

        status = ::vslDeleteStream(&new_str_ptr);
        mkl_rng_error_check(BRNG, status,
                "MKLStream::seed", "vslDeleteStream");
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
        unsigned offset = SeedGenerator<MKLStream<BRNG> >::instance().get();
        this->offset(static_cast<MKL_INT>(offset));
        seed(seed_);
    }

    private :

    MKL_UINT seed_;
    VSLStreamStatePtr str_ptr_;
}; // class Stream

/// \brief Base class of MKL distribution
/// \ingroup MKLRNG
template <typename ResultType, typename Derived>
class MKLDistribution
{
    public :

    typedef ResultType result_type;

    MKLDistribution () : remain_(0), buffer_size_(VSMC_RNG_MKL_BUFFER_SIZE) {}

    MKLDistribution (const MKLDistribution &other) :
        remain_(other.remain_), buffer_size_(other.buffer_size_),
        result_(other.result_) {}

    MKLDistribution &operator= (const MKLDistribution &other)
    {
        if (this != &other) {
            remain_ = other.remain_;
            buffer_size_ = other.buffer_size_;
            result_ = other.result_;
        }

        return *this;
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    MKLDistribution (MKLDistribution &&other) :
        remain_(other.remain_), buffer_size_(other.buffer_size_),
        result_(cxx11::move(other.result_)) {}

    MKLDistribution &operator= (MKLDistribution &&other)
    {
        if (this != &other) {
            remain_ = other.remain_;
            buffer_size_ = other.buffer_size_;
            result_ = cxx11::move(other.result_);
        }

        return *this;
    }
#endif

    template <MKL_INT BRNG>
    result_type operator() (const MKLStream<BRNG> &stream)
    {
        result_.resize(static_cast<std::size_t>(buffer_size_));
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
    void operator() (const MKLStream<BRNG> &stream, MKL_INT n, result_type *r)
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
        dist_name = "MKL" + dist_name + "Distribution::generate";
        std::string vsl_name(vsl_name_prefix(static_cast<result_type>(0)));
        vsl_name += "Rng";
        vsl_name += name;
        mkl_rng_error_check(BRNG, status, dist_name.c_str(), vsl_name.c_str());
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
class MKLEngine
{
    public :

    typedef ResultType result_type;
    typedef MKLStream<BRNG> stream_type;
    typedef typename traits::MKLSkipAheadTrait<BRNG, ResultType>::type
        skip_ahead_type;
    typedef MKLUniformBitsDistribution<result_type> runif_type;

    explicit MKLEngine (MKL_UINT s = traits::MKLSeedTrait<BRNG>::value) :
        stream_(s) {}

    template <typename SeedSeq>
    explicit MKLEngine (SeedSeq &seq) : stream_(seq) {}

    MKLEngine (const MKLEngine<BRNG, ResultType> &other) :
        stream_(other.stream_), skip_ahead_(other.skip_ahead_) {}

    MKLEngine (MKLEngine<BRNG, ResultType> &other) :
        stream_(other.stream_), skip_ahead_(other.skip_ahead_) {}

    MKLEngine<BRNG, ResultType> &operator= (
            const MKLEngine<BRNG, ResultType> &other)
    {
        if (this != &other) {
            stream_ = other.stream_;
            skip_ahead_ = other.skip_ahead_;
        }

        return *this;
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    MKLEngine (MKLEngine<BRNG, ResultType> &&other) :
        stream_(static_cast<MKLStream<BRNG> &&>(other.stream_)),
        skip_ahead_(static_cast<skip_ahead_type &&>(other.skip_ahead_)) {}

    MKLEngine<BRNG, ResultType> &operator= (
            MKLEngine<BRNG, ResultType> &&other)
    {
        if (this != &other) {
            stream_ = cxx11::move(other.stream_);
            skip_ahead_ = cxx11::move(other.skip_ahead_);
        }

        return *this;
    }
#endif

    bool empty () const {return stream_.empty();}

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

    static VSMC_CONSTEXPR const result_type _Min = 0;
    static VSMC_CONSTEXPR const result_type _Max =
        ~(static_cast<result_type>(0));

    static VSMC_CONSTEXPR result_type min VSMC_MNE () {return _Min;}
    static VSMC_CONSTEXPR result_type max VSMC_MNE () {return _Max;}

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
typedef MKLEngine<VSL_BRNG_MCG59, unsigned> MKL_MCG59;

/// \brief A Mersenne-Twister pseudoranom number genertor
/// \ingroup MKLRNG
typedef MKLEngine<VSL_BRNG_MT19937, unsigned> MKL_MT19937;

/// \brief A Mersenne-Twister pseudoranom number genertor (64-bit)
/// \ingroup MKLRNG
typedef MKLEngine<VSL_BRNG_MT19937, unsigned MKL_INT64> MKL_MT19937_64;

/// \brief A SIMD-oriented fast Mersenne-Twister pseudoranom number genertor
/// \ingroup MKLRNG
typedef MKLEngine<VSL_BRNG_SFMT19937, unsigned> MKL_SFMT19937;

/// \brief A SIMD-oriented fast Mersenne-Twister pseudoranom number genertor
/// (64-bit)
/// \ingroup MKLRNG
typedef MKLEngine<VSL_BRNG_SFMT19937, unsigned MKL_INT64> MKL_SFMT19937_64;

/// \brief A set of 6024 Mersenne-Twister pseudoranom number genertor
/// \ingroup MKLRNG
typedef MKLEngine<VSL_BRNG_MT2203, unsigned> MKL_MT2203;

/// \brief A set of 6024 Mersenne-Twister pseudoranom number genertor (64-bit)
/// \ingroup MKLRNG
typedef MKLEngine<VSL_BRNG_MT2203, unsigned MKL_INT64>MKL_MT2203_64;

/// \brief A non-determinstic random number generator
/// \ingroup MKLRNG
typedef MKLEngine<VSL_BRNG_NONDETERM, unsigned> MKL_NONDETERM;

/// \brief A non-determinstic random number generator (64-bit)
/// \ingroup MKLRNG
typedef MKLEngine<VSL_BRNG_NONDETERM, unsigned MKL_INT64> MKL_NONDETERM_64;

/// \brief MKL uniform bits (32-bits)
/// \ingroup MKLRNG
template <>
class MKLUniformBitsDistribution<unsigned> :
    public MKLDistribution<unsigned, MKLUniformBitsDistribution<unsigned> >
{
    public :

    typedef unsigned result_type;

    template <MKL_INT BRNG>
    void generate (const MKLStream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = ::viRngUniformBits32(VSL_RNG_METHOD_UNIFORMBITS32_STD,
                str.ptr(), n, r);
        mkl_rng_error_check(BRNG, status,
                "MKLUniformBitsDistribution::generate", "viRngUniformBits32");
    }
}; // class UniformBits

/// \brief MKL uniform bits (64-bits)
/// \ingroup MKLRNG
template <>
class MKLUniformBitsDistribution<unsigned MKL_INT64> :
    public MKLDistribution<unsigned MKL_INT64,
    MKLUniformBitsDistribution<unsigned MKL_INT64> >
{
    public :

    typedef unsigned MKL_INT64 result_type;

    template <MKL_INT BRNG>
    void generate (const MKLStream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = ::viRngUniformBits64(VSL_RNG_METHOD_UNIFORMBITS64_STD,
                str.ptr(), n, r);
        mkl_rng_error_check(BRNG, status,
                "MKLUniformBitsDistribution::generate", "viRngUniformBits64");
    }
}; // class UniformBits

/// \brief MKL Bernoulli distribution
/// \ingroup MKLRNG
template <MKL_INT Method>
class MKLBernoulliDistribution :
    public MKLDistribution<MKL_INT, MKLBernoulliDistribution<Method> >
{
    public :

    typedef MKL_INT result_type;

    explicit MKLBernoulliDistribution (double p = 0.5) : p_(p) {}

    template <MKL_INT BRNG>
    void generate (const MKLStream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = ::viRngBernoulli(Method, str.ptr(), n, r, p_);
        this->template generate_error_check<BRNG>(status, "Bernoulli");
    }

    private :

    double p_;
}; // class Bernoulli

/// \brief MKL Geometric distribution
/// \ingroup MKLRNG
template <MKL_INT Method>
class MKLGeometricDistribution :
    public MKLDistribution<MKL_INT, MKLGeometricDistribution<Method> >
{
    public :

    typedef MKL_INT result_type;

    explicit MKLGeometricDistribution (double p = 0.5) : p_(p) {}

    template <MKL_INT BRNG>
    void generate (const MKLStream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = ::viRngGeometric(Method, str.ptr(), n, r, p_);
        this->template generate_error_check<BRNG>(status, "Geometric");
    }

    private :

    double p_;
}; // class Geometric

/// \brief MKL Binomial distribution
/// \ingroup MKLRNG
template <MKL_INT Method>
class MKLBinomialDistribution :
    public MKLDistribution<MKL_INT, MKLBinomialDistribution<Method> >
{
    public :

    typedef MKL_INT result_type;

    explicit MKLBinomialDistribution (result_type ntrial = 1, double p = 0.5) :
        ntrial_(ntrial), p_(p) {}

    template <MKL_INT BRNG>
    void generate (const MKLStream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = ::viRngBinomial(Method, str.ptr(), n, r, ntrial_, p_);
        this->template generate_error_check<BRNG>(status, "Binomial");
    }

    private :

    result_type ntrial_;
    double p_;
}; // class Binomial

/// \brief MKL Hypergeometric distribution
/// \ingroup MKLRNG
template <MKL_INT Method>
class MKLHypergeometricDistribution :
    public MKLDistribution<MKL_INT, MKLHypergeometricDistribution<Method> >
{
    public :

    typedef MKL_INT result_type;

    MKLHypergeometricDistribution (result_type population, result_type sample,
            result_type mask) :
        l_(population), s_(sample), m_(mask) {}

    template <MKL_INT BRNG>
    void generate (const MKLStream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = ::viRngHypergeometric(Method, str.ptr(),
                n, r, l_, s_, m_);
        this->template generate_error_check<BRNG>(status, "Hypergeometric");
    }

    private :

    result_type l_;
    result_type s_;
    result_type m_;
}; // class Hypergeometric

/// \brief MKL Poisson distribution
/// \ingroup MKLRNG
template <MKL_INT Method>
class MKLPoissonDistribution :
    public MKLDistribution<MKL_INT, MKLPoissonDistribution<Method> >
{
    public :

    typedef MKL_INT result_type;

    explicit MKLPoissonDistribution (double lambda = 1) : lambda_(lambda) {}

    template <MKL_INT BRNG>
    void generate (const MKLStream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = ::viRngPoisson(Method, str.ptr(), n, r, lambda_);
        this->template generate_error_check<BRNG>(status, "Poisson");
    }

    private :

    double lambda_;
}; // class Poisson

/// \brief MKL NegBinomial distribution
/// \ingroup MKLRNG
template <MKL_INT Method>
class MKLNegBinomialDistribution :
    public MKLDistribution<MKL_INT, MKLNegBinomialDistribution<Method> >
{
    public :

    typedef MKL_INT result_type;

    explicit MKLNegBinomialDistribution (double ntrial = 1, double p = 0.5) :
        ntrial_(ntrial), p_(p) {}

    template <MKL_INT BRNG>
    void generate (const MKLStream<BRNG> &str, MKL_INT n, result_type *r)
    {
        int status = ::viRngNegBinomial(Method, str.ptr(), n, r, ntrial_, p_);
        this->template generate_error_check<BRNG>(status, "NegBinomial");
    }

    private :

    double ntrial_;
    double p_;
}; // class NegBinomial

/// \brief MKL Uniform distribution
/// \ingroup MKLRNG
template <typename ResultType, MKL_INT Method>
class MKLUniformDistribution :
    public MKLDistribution<ResultType,
    MKLUniformDistribution<ResultType, Method> >
{
    public :

    typedef ResultType result_type;

    explicit MKLUniformDistribution (result_type a = 0, result_type b = 1) :
        a_(a), b_(b) {}

    template <MKL_INT BRNG>
    void generate (const MKLStream<BRNG> &str, MKL_INT n, result_type *r)
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

/// \brief MKL Gaussian distribution
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class MKLGaussianDistribution :
    public MKLDistribution<FPType, MKLGaussianDistribution<FPType, Method> >
{
    public :

    typedef FPType result_type;

    explicit MKLGaussianDistribution (result_type mean = 0,
            result_type sd = 1) : mean_(mean), sd_(sd)
    {VSMC_STATIC_ASSERT_RNG_MKL_RNG_DISTRIBUTION_FPTYPE(FPType, Gaussian);}

    template <MKL_INT BRNG>
    void generate (const MKLStream<BRNG> &str, MKL_INT n, result_type *r)
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

/// \brief MKL Exponential distribution
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class MKLExponentialDistribution :
    public MKLDistribution<FPType, MKLExponentialDistribution<FPType, Method> >
{
    public :

    typedef FPType result_type;

    explicit MKLExponentialDistribution (result_type displacement = 0,
            result_type scale = 1) : disp_(displacement), scale_(scale)
    {VSMC_STATIC_ASSERT_RNG_MKL_RNG_DISTRIBUTION_FPTYPE(FPType, Exponential);}

    template <MKL_INT BRNG>
    void generate (const MKLStream<BRNG> &str, MKL_INT n, result_type *r)
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

/// \brief MKL Laplace distribution
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class MKLLaplaceDistribution :
    public MKLDistribution<FPType, MKLLaplaceDistribution<FPType, Method> >
{
    public :

    typedef FPType result_type;

    explicit MKLLaplaceDistribution (result_type mean = 0,
            result_type scale = 1) : mean_(mean), scale_(scale)
    {VSMC_STATIC_ASSERT_RNG_MKL_RNG_DISTRIBUTION_FPTYPE(FPType, Laplace);}

    template <MKL_INT BRNG>
    void generate (const MKLStream<BRNG> &str, MKL_INT n, result_type *r)
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

/// \brief MKL Weibull distribution
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class MKLWeibullDistribution :
    public MKLDistribution<FPType, MKLWeibullDistribution<FPType, Method> >
{
    public :

    typedef FPType result_type;

    explicit MKLWeibullDistribution (result_type shape = 1,
            result_type displacement = 0, result_type scale = 1) :
        shape_(shape), disp_(displacement), scale_(scale)
    {VSMC_STATIC_ASSERT_RNG_MKL_RNG_DISTRIBUTION_FPTYPE(FPType, Weibull);}

    template <MKL_INT BRNG>
    void generate (const MKLStream<BRNG> &str, MKL_INT n, result_type *r)
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

/// \brief MKL Cauchy distribution
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class MKLCauchyDistribution :
    public MKLDistribution<FPType, MKLCauchyDistribution<FPType, Method> >
{
    public :

    typedef FPType result_type;

    explicit MKLCauchyDistribution (result_type displacement = 0,
            result_type scale = 1) : disp_(displacement), scale_(scale)
    {VSMC_STATIC_ASSERT_RNG_MKL_RNG_DISTRIBUTION_FPTYPE(FPType, Cauchy);}

    template <MKL_INT BRNG>
    void generate (const MKLStream<BRNG> &str, MKL_INT n, result_type *r)
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

/// \brief MKL Rayleigh distribution
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class MKLRayleighDistribution :
    public MKLDistribution<FPType, MKLRayleighDistribution<FPType, Method> >
{
    public :

    typedef FPType result_type;

    explicit MKLRayleighDistribution (result_type displacement = 0,
            result_type scale = 1) : disp_(displacement), scale_(scale)
    {VSMC_STATIC_ASSERT_RNG_MKL_RNG_DISTRIBUTION_FPTYPE(FPType, Rayleigh);}

    template <MKL_INT BRNG>
    void generate (const MKLStream<BRNG> &str, MKL_INT n, result_type *r)
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

/// \brief MKL Lognormal distribution
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class MKLLognormalDistribution :
    public MKLDistribution<FPType, MKLLognormalDistribution<FPType, Method> >
{
    public :

    typedef FPType result_type;

    explicit MKLLognormalDistribution (
            result_type mean = 0, result_type sd = 1,
            result_type displacement = 0, result_type scale = 1) :
        mean_(mean), sd_(sd), disp_(displacement), scale_(scale)
    {VSMC_STATIC_ASSERT_RNG_MKL_RNG_DISTRIBUTION_FPTYPE(FPType, Lognormal);}

    template <MKL_INT BRNG>
    void generate (const MKLStream<BRNG> &str, MKL_INT n, result_type *r)
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

/// \brief MKL Gumbel distribution
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class MKLGumbelDistribution :
    public MKLDistribution<FPType, MKLGumbelDistribution<FPType, Method> >
{
    public :

    typedef FPType result_type;

    explicit MKLGumbelDistribution (result_type displacement = 0,
            result_type scale = 1) : disp_(displacement), scale_(scale)
    {VSMC_STATIC_ASSERT_RNG_MKL_RNG_DISTRIBUTION_FPTYPE(FPType, Gumbel);}

    template <MKL_INT BRNG>
    void generate (const MKLStream<BRNG> &str, MKL_INT n, result_type *r)
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

/// \brief MKL Gamma distribution
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class MKLGammaDistribution :
    public MKLDistribution<FPType, MKLGammaDistribution<FPType, Method> >
{
    public :

    typedef FPType result_type;

    explicit MKLGammaDistribution (result_type shape = 1,
            result_type displacement = 0, result_type scale = 1) :
        shape_(shape), disp_(displacement), scale_(scale)
    {VSMC_STATIC_ASSERT_RNG_MKL_RNG_DISTRIBUTION_FPTYPE(FPType, Gamma);}

    template <MKL_INT BRNG>
    void generate (const MKLStream<BRNG> &str, MKL_INT n, result_type *r)
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

/// \brief MKL Beta distribution
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class MKLBetaDistribution :
    public MKLDistribution<FPType, MKLBetaDistribution<FPType, Method> >
{
    public :

    typedef FPType result_type;

    explicit MKLBetaDistribution (
            result_type shape1 = 1, result_type shape2 = 1,
            result_type displacement = 0, result_type scale = 1) :
        shape1_(shape1), shape2_(shape2), disp_(displacement), scale_(scale)
    {VSMC_STATIC_ASSERT_RNG_MKL_RNG_DISTRIBUTION_FPTYPE(FPType, Beta);}

    template <MKL_INT BRNG>
    void generate (const MKLStream<BRNG> &str, MKL_INT n, result_type *r)
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

namespace traits {

template <MKL_INT BRNG, typename ResultType>
struct RngShift<MKLEngine<BRNG, ResultType> >
{
    void operator() (MKLEngine<BRNG, ResultType> &rng) const
    {rng.stream().shift();}
};

} // namespace vsmc::traits

} // namespace vsmc

#endif // VSMC_RNG_MKL_RNG_HPP
