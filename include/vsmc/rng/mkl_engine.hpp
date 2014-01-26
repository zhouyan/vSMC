#ifndef VSMC_RNG_MKL_ENGINE_HPP
#define VSMC_RNG_MKL_ENGINE_HPP

#include <vsmc/internal/common.hpp>
#include <mkl_vsl.h>

#if VSMC_HAS_CXX11_THREAD_LOCAL && VSMC_HAS_CXX11LIB_MUTEX
#include <vsmc/rng/rng_set.hpp>
#endif

#define VSMC_RUNTIME_ASSEET_RNG_MKL_ENGINE_OFFSET(offset) \
    VSMC_RUNTIME_ASSERT((offset < max VSMC_MACRO_NO_EXPANSION ()),           \
            ("**vsmc::RngSet<vsmc::mkl::MT2203, vsmc::VectorRng> "           \
             "EXCESS MAXIMUM NUMBER OF INDEPDENT RNG STREAMS"))

namespace vsmc {

namespace mkl {

#ifndef NDEBUG
/// \brief Check MKL RNG error status
/// \ingroup RNG
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

template <MKL_INT, typename> struct EngRandomBits;

/// \brief MKL RNG C++11 engine generating random bits (32-bits)
/// \ingroup RNG
template <MKL_INT BRNG>
struct EngRandomBits<BRNG, unsigned>
{
    typedef unsigned result_type;

    static void generate (VSLStreamStatePtr str, MKL_INT n, result_type *r)
    {
        int status = viRngUniformBits32(VSL_RNG_METHOD_UNIFORMBITS32_STD,
                str, n, r);
        rng_error_check<BRNG>(
                status, "EngRandomBits::generate", "viRngUniformBits32");
    }
}; // struct EngRandomBits

/// \brief MKL RNG C++11 engine generating random bits (64-bits)
/// \ingroup RNG
template <MKL_INT BRNG>
struct EngRandomBits<BRNG, unsigned MKL_INT64>
{
    typedef unsigned MKL_INT64 result_type;

    static void generate (VSLStreamStatePtr str, MKL_INT n, result_type *r)
    {
        int status = viRngUniformBits64(VSL_RNG_METHOD_UNIFORMBITS64_STD,
                str, n, r);
        rng_error_check<BRNG>(
                status, "EngRandomBits::generate", "viRngUniformBits64");
    }
}; // struct EngRandomBits

/// \brief MKL RNG C++11 engine skip ahead using `vslSkipAheadStream`
/// \ingroup RNG
template <MKL_INT BRNG>
struct EngSkipVSL
{
    typedef long long skip_size_type;

    static void skip (VSLStreamStatePtr stream, skip_size_type nskip)
    {
        int status = vslSkipAheadStream(stream, nskip);
        rng_error_check<BRNG>(
                status, "EngSkipVSL::skip", "vslSkipAheadStream");
    }

    static void skip_buffer_size (skip_size_type) {}
    static VSMC_CONSTEXPR skip_size_type skip_buffer_size () {return 0;}
}; // struct EngSkipVSL

/// \brief MKL RNG C++11 engine skip ahead by generating random numbers
/// \ingroup RNG
template <MKL_INT BRNG, typename RT>
struct EngSkipForce
{
    typedef MKL_INT skip_size_type;

    EngSkipForce () : buffer_size_(1000) {}

    void skip (VSLStreamStatePtr stream, skip_size_type nskip)
    {
        if (nskip == 0)
            return;

        if (nskip < buffer_size()) {
            if (ruint_.size() < nskip)
                ruint_.resize(nskip);
            EngRandomBits<BRNG, RT>::generate(stream, nskip, &ruint_[0]);
        } else {
            if (ruint_.size() < buffer_size_)
                ruint_.resize(buffer_size_);
            skip_size_type repeat = nskip / buffer_size_;
            skip_size_type remain = nskip - repeat * buffer_size_;
            for (skip_size_type r = 1; r != repeat + 1; ++r) {
                skip_size_type n = r * buffer_size_;
                EngRandomBits<BRNG, RT>::generate(stream, n, &ruint_[0]);
            }
            EngRandomBits<BRNG, RT>::generate(stream, remain, &ruint_[0]);
        }
    }

    void buffer_size (skip_size_type size) {buffer_size_ = size;}
    skip_size_type buffer_size () const {return buffer_size_;}

    private :

    skip_size_type buffer_size_;
    std::vector<RT> ruint_;
}; // strut EngSkipForce

/// \brief MKL RNG index offest (constant zero)
/// \ingroup RNG
struct EngOffsetZero
{
    static VSMC_CONSTEXPR MKL_INT min VSMC_MACRO_NO_EXPANSION () {return 0;}
    static VSMC_CONSTEXPR MKL_INT max VSMC_MACRO_NO_EXPANSION () {return 0;}
    static void offset (MKL_INT) {}
    static VSMC_CONSTEXPR MKL_INT offset () {return 0;}
}; // struct EngOffsetZero

/// \brief MKL RNG index offest (set dynamically)
/// \ingroup RNG
template <MKL_INT MaxOffset>
struct EngOffsetDynamic
{
    EngOffsetDynamic () : offset_(0) {}

    static VSMC_CONSTEXPR MKL_INT min VSMC_MACRO_NO_EXPANSION () {return 0;}
    static VSMC_CONSTEXPR MKL_INT max VSMC_MACRO_NO_EXPANSION ()
    {return MaxOffset;}

    void offset (MKL_INT n)
    {
        VSMC_RUNTIME_ASSEET_RNG_MKL_ENGINE_OFFSET(n);
        offset_ = n;
    }

    MKL_INT offset () const {return offset_;}

    private :

    MKL_INT offset_;
}; // struct EngOffsetDynamic

/// \brief MKL RNG C++11 engine
/// \ingroup RNG
template <MKL_INT BRNG, typename RT,
         typename EngSkip, typename EngOffset,
         std::size_t Buffer = 1000, MKL_UINT Seed = 101>
class Engine : public EngSkip, public EngOffset
{
    public :

    typedef RT result_type;

    explicit Engine (MKL_UINT seed = Seed) : ruint_(Buffer), remain_(0)
    {
        int status = vslNewStream(&stream_, BRNG + this->offset(), seed);
        rng_error_check<BRNG>(status, "Engine::Engine", "vslNewStream");
    }

    template <typename SeedSeq>
    explicit Engine (SeedSeq &seq) : ruint_(Buffer), remain_(0)
    {
        MKL_UINT seed = 0;
        seq.generate(&seed, &seed + 1);
        int status = vslNewStream(&stream_, BRNG + this->offset(), seed);
        rng_error_check<BRNG>(status, "Engine::Engine", "vslNewStream");
    }

    Engine (const Engine<BRNG, RT, EngSkip, EngOffset, Buffer, Seed> &other) :
        ruint_(other.ruint_), remain_(other.remain_)
    {
        int status = vslCopyStream(&stream_, other.stream_);
        rng_error_check<BRNG>(status, "Engine::Engine", "vslCopyStream");
    }

    Engine<BRNG, RT, EngSkip, EngOffset, Buffer, Seed> &operator= (
            const Engine<BRNG, RT, EngSkip, EngOffset, Buffer, Seed> &other)
    {
        ruint_ = other.ruint_;
        remain_ = other.remain_;
        int status = vslCopyStreamState(stream_, other.stream_);
        rng_error_check<BRNG>(
                status, "Engine::operator=", "vslCopyStreamState");
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    Engine (Engine<BRNG, RT, EngSkip, EngOffset, Buffer, Seed> &&other) :
        ruint_(other.ruint_), remain_(other.remain_), stream_(other.stream_) {}

    Engine<BRNG, RT, EngSkip, EngOffset, Buffer, Seed> &operator= (
            Engine<BRNG, RT, EngSkip, EngOffset, Buffer, Seed> &&other)
    {
        ruint_ = other.ruint_;
        remain_ = other.remain_;;
        stream_ = other.stream_;
    }
#endif

    ~Engine () {vslDeleteStream(&stream_);}

    void seed (MKL_UINT s = Seed)
    {
        int status = VSL_ERROR_OK;
        VSLStreamStatePtr new_stream;
        status = vslNewStream(&new_stream, BRNG + this->offset(), s);
        rng_error_check<BRNG>(status, "Engine::seed", "vslNewStream");
        status = vslCopyStreamState(stream_, new_stream);
        rng_error_check<BRNG>(status, "Engine::seed", "vslCopyStreamState");
        status = vslDeleteStream(&new_stream);
        rng_error_check<BRNG>(status, "Engine::seed", "vslDeleteStream");
        remain_ = 0;
    }

    template <typename SeedSeq>
    void seed (SeedSeq &seq)
    {
        MKL_UINT s = 0;
        seq.generate(&s, &s + 1);
        seed(s);
    }

    result_type operator() ()
    {
        result_type *ruint_ptr = &ruint_[0];
        if (remain_ > 0) {
            --remain_;
            return ruint_ptr[remain_];
        }

        EngRandomBits<BRNG, RT>::generate(stream_, Buffer, ruint_ptr);
        remain_ = Buffer - 1;
        return ruint_ptr[remain_];
    }

    void discard (std::size_t nskip)
    {
        this->skip(stream_,
                static_cast<typename EngSkip::skip_size_type>(nskip));
        remain_ = 0;
    }

    VSLStreamStatePtr stream () const {return stream_;}

    static const result_type _Min = 0;
    static const result_type _Max = ~((result_type)0);

    static VSMC_CONSTEXPR result_type min VSMC_MACRO_NO_EXPANSION ()
    {return _Min;}

    static VSMC_CONSTEXPR result_type max VSMC_MACRO_NO_EXPANSION ()
    {return _Max;}

    private :

    std::vector<result_type> ruint_;
    std::size_t remain_;
    VSLStreamStatePtr stream_;
}; // class Engine

/// \brief A 59-bit multiplicative congruential generator
/// \ingroup RNG
typedef Engine<VSL_BRNG_MCG59, unsigned,
        EngSkipVSL<VSL_BRNG_MCG59>, EngOffsetZero> MCG59;

/// \brief A Mersenne-Twister pseudoranom number genertor
/// \ingroup RNG
typedef Engine<VSL_BRNG_MT19937, unsigned,
        EngSkipForce<VSL_BRNG_MT19937, unsigned>, EngOffsetZero>
        MT19937;

/// \brief A Mersenne-Twister pseudoranom number genertor (64-bit)
/// \ingroup RNG
typedef Engine<VSL_BRNG_MT19937, unsigned MKL_INT64,
        EngSkipForce<VSL_BRNG_MT19937, unsigned MKL_INT64>, EngOffsetZero>
        MT19937_64;

/// \brief A SIMD-oriented fast Mersenne-Twister pseudoranom number genertor
/// \ingroup RNG
typedef Engine<VSL_BRNG_SFMT19937, unsigned,
        EngSkipForce<VSL_BRNG_SFMT19937, unsigned>, EngOffsetZero>
        SFMT19937;

/// \brief A SIMD-oriented fast Mersenne-Twister pseudoranom number genertor
/// (64-bit)
/// \ingroup RNG
typedef Engine<VSL_BRNG_SFMT19937, unsigned MKL_INT64,
        EngSkipForce<VSL_BRNG_SFMT19937, unsigned MKL_INT64>, EngOffsetZero>
        SFMT19937_64;

/// \brief A set of 6024 Mersenne-Twister pseudoranom number genertor
/// \ingroup RNG
typedef Engine<VSL_BRNG_MT2203, unsigned,
        EngSkipForce<VSL_BRNG_MT2203, unsigned>, EngOffsetDynamic<6024> >
        MT2203;

/// \brief A set of 6024 Mersenne-Twister pseudoranom number genertor (64-bit)
/// \ingroup RNG
typedef Engine<VSL_BRNG_MT2203, unsigned MKL_INT64,
        EngSkipForce<VSL_BRNG_MT2203, unsigned MKL_INT64>, EngOffsetDynamic<6024> >
        MT2203_64;

/// \brief A non-determinstic random number generator
/// \ingroup RNG
typedef Engine<VSL_BRNG_NONDETERM, unsigned,
        EngSkipForce<VSL_BRNG_NONDETERM, unsigned>, EngOffsetZero>
        NONDETERM;

/// \brief A non-determinstic random number generator (64-bit)
/// \ingroup RNG
typedef Engine<VSL_BRNG_NONDETERM, unsigned MKL_INT64,
        EngSkipForce<VSL_BRNG_NONDETERM, unsigned MKL_INT64>, EngOffsetZero>
        NONDETERM_64;

} // namespace vsmc::mkl

#if VSMC_HAS_CXX11_THREAD_LOCAL && VSMC_HAS_CXX11LIB_MUTEX

namespace mkl {

struct ThreadLocalRng;

} // namespace vsmc::mkl

/// \brief Vector RNG set using MKL BRNG with thread-local storage
/// \ingroup RNG
template <MKL_INT BRNG, typename RT, typename EngSkip, typename EngOffset,
         std::size_t Buffer, MKL_UINT Seed>
class RngSet<mkl::Engine<BRNG, RT, EngSkip, EngOffset, Buffer, Seed>,
      mkl::ThreadLocalRng>
{
    public :

    typedef mkl::Engine<BRNG, RT, EngOffset, EngSkip, Buffer, Seed> rng_type;
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
        tl_rng.offset(static_cast<MKL_INT>(offset));
        tl_rng.seed(static_cast<MKL_UINT>(Seed::instance().get()));
        tl_flag = true;
    }
}; // class RngSet

#endif // VSMC_HAS_CXX11_THREAD_LOCAL && VSMC_HAS_CXX11LIB_MUTEX

} // namespace vsmc

#endif // VSMC_RNG_MKL_ENGINE_HPP
