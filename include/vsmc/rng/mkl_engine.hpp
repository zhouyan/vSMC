#ifndef VSMC_RNG_MKL_ENGINE_HPP
#define VSMC_RNG_MKL_ENGINE_HPP

#include <vsmc/internal/common.hpp>
#include <mkl_vsl.h>

namespace vsmc { namespace mkl {

template <MKL_INT BRNG>
struct EngSkipVSL
{
    static int skip (VSLStreamStatePtr stream, std::size_t nskip)
    {
        VSLBRngProperties prop;
        vslGetBrngProperties(BRNG, &prop);
        std::size_t skip_mul = prop.WordSize / sizeof(uint32_t);
        if (skip_mul < 1) skip_mul = 1;

        return vslSkipAheadStream(stream, nskip / skip_mul);
    }
}; // struct EngSkipVSL

struct EngSkipForce
{
    int skip (VSLStreamStatePtr stream, std::size_t nskip)
    {
        ruint_.resize(nskip);

        return viRngUniformBits32(VSL_RNG_METHOD_UNIFORMBITS32_STD,
                stream, nskip, &ruint_[0]);
    }

    private :

    std::vector<uint32_t> ruint_;
}; // strut EngSkipForce

struct EngOffsetZero
{
    static void set_offset (MKL_INT) {}
    static VSMC_CONSTEXPR MKL_INT offset () {return 0;}
}; // struct EngOffsetZero

struct EngOffsetDynamic
{
    EngOffsetDynamic () : offset_(0) {}

    void set_offset (MKL_INT n) {offset_ = n;}
    MKL_INT offset () const {return offset_;}

    private :

    MKL_INT offset_;
}; // struct EngOffsetDynamic

template <MKL_INT BRNG, typename EngOffset, typename EngSkip,
         std::size_t Buffer = 1000, MKL_UINT Seed = 101>
class Engine : public EngOffset, public EngSkip
{
    public :

    typedef uint32_t result_type;

    explicit Engine (MKL_UINT seed = Seed) : ruint_(Buffer), remain_(0)
    {
        int status = vslNewStream(&stream_, BRNG + this->offset(), seed);
        error_check(status);
    }

    template <typename SeedSeq>
    explicit Engine (SeedSeq &seq) : ruint_(Buffer), remain_(0)
    {
        MKL_UINT seed = 0;
        seq.generate(&seed, &seed + 1);
        int status = vslNewStream(&stream_, BRNG + this->offset(), seed);
        error_check(status);
    }

    Engine (const Engine<BRNG, EngOffset, EngSkip, Buffer, Seed> &other) :
        ruint_(other.ruint_), remain_(other.remain_)
    {
        int status = vslCopyStream(&stream_, other.stream_);
        error_check(status);
    }

    Engine<BRNG, EngOffset, EngSkip, Buffer, Seed> &operator= (
            const Engine<BRNG, EngOffset, EngSkip, Buffer, Seed> &other)
    {
        ruint_ = other.ruint_;
        remain_ = other.remain_;
        int status = vslCopyStreamState(stream_, other.stream_);
        error_check(status);
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    Engine (Engine<BRNG, EngOffset, EngSkip, Buffer, Seed> &&other) :
        ruint_(other.ruint_), remain_(other.remain_), stream_(other.stream_) {}

    Engine<BRNG, EngOffset, EngSkip, Buffer, Seed> &operator= (
            Engine<BRNG, EngOffset, EngSkip, Buffer, Seed> &&other)
    {
        ruint_ = other.ruint_;
        remain_ = other.remain_;;
        stream_ = other.stream_;
    }
#endif

    ~Engine () {vslDeleteStream(&stream_);}

    void seed (MKL_UINT s = Seed)
    {
        int status = VSL_STATUS_OK;
        VSLStreamStatePtr new_stream;
        status = vslNewStream(&new_stream, BRNG + this->offset(), s);
        error_check(status);
        status = vslCopyStreamState(stream_, new_stream);
        error_check(status);
        status = vslDeleteStream(&new_stream);
        error_check(status);
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

        int status = viRngUniformBits32(VSL_RNG_METHOD_UNIFORMBITS32_STD,
                stream_, Buffer, ruint_ptr);
        error_check(status);
        remain_ = Buffer - 1;

        return ruint_ptr[remain_];
    }

    void discard (std::size_t nskip)
    {
        int status = this->skip(stream_, nskip);
        error_check(status);
        remain_ = 0;
    }

    VSLStreamStatePtr stream () const {return stream_;}

    static VSMC_CONSTEXPR result_type min VSMC_MACRO_NO_EXPANSION () {return _Min;}
    static VSMC_CONSTEXPR result_type max VSMC_MACRO_NO_EXPANSION () {return _Max;}

    static const result_type _Min = 0;
    static const result_type _Max = ~((result_type)0);

    private :

    std::vector<result_type> ruint_;
    std::size_t remain_;
    VSLStreamStatePtr stream_;

    static void error_check (int status)
    {
        if (status != VSL_ERROR_OK) {
            std::string msg("Using MKL BRNG: ");
            switch (BRNG) {
                case VSL_BRNG_MCG31 :
                    msg += "VSL_BRNG_MCG31";
                    break;
                case VSL_BRNG_MCG59 :
                    msg += "VSL_BRNG_MCG59";
                    break;
                case VSL_BRNG_MRG32K3A :
                    msg += "VSL_BRNG_MRG32K3A";
                    break;
                case VSL_BRNG_R250 :
                    msg += "VSL_BRNG_R250";
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
                case VSL_BRNG_WH :
                    msg += "VSL_BRNG_WH";
                    break;
                case VSL_BRNG_NONDETERM :
                    msg += "VSL_BRNG_NONDETERM";
                    break;
            }
            msg += "\n";
            std::fprintf(stderr, "%s", msg.c_str());
        }
        switch (status) {
            case VSL_ERROR_BADARGS :
                VSMC_RUNTIME_ASSERT(false, "VSL_ERROR_BADARGS");
                return;
            case VSL_ERROR_CPU_NOT_SUPPORTED :
                VSMC_RUNTIME_ASSERT(false, "VSL_ERROR_CPU_NOT_SUPPORTED");
                return;
            case VSL_ERROR_FEATURE_NOT_IMPLEMENTED :
                VSMC_RUNTIME_ASSERT(false, "VSL_ERROR_FEATURE_NOT_IMPLEMENTED");
                return;
            case VSL_ERROR_MEM_FAILURE :
                VSMC_RUNTIME_ASSERT(false, "VSL_ERROR_MEM_FAILURE");
                return;
            case VSL_ERROR_NULL_PTR :
                VSMC_RUNTIME_ASSERT(false, "VSL_ERROR_NULL_PTR");
                return;
            case VSL_ERROR_UNKNOWN :
                VSMC_RUNTIME_ASSERT(false, "VSL_ERROR_UNKNOWN");
                return;
            case VSL_RNG_ERROR_BAD_FILE_FORMAT :
                VSMC_RUNTIME_ASSERT(false, "VSL_RNG_ERROR_BAD_FILE_FORMAT");
                return;
            case VSL_RNG_ERROR_BAD_MEM_FORMAT :
                VSMC_RUNTIME_ASSERT(false, "VSL_RNG_ERROR_BAD_MEM_FORMAT");
                return;
            case VSL_RNG_ERROR_BAD_NBITS :
                VSMC_RUNTIME_ASSERT(false, "VSL_RNG_ERROR_BAD_NBITS");
                return;
            case VSL_RNG_ERROR_BAD_NSEEDS :
                VSMC_RUNTIME_ASSERT(false, "VSL_RNG_ERROR_BAD_NSEEDS");
                return;
            case VSL_RNG_ERROR_BAD_STREAM :
                VSMC_RUNTIME_ASSERT(false, "VSL_RNG_ERROR_BAD_STREAM");
                return;
            case VSL_RNG_ERROR_BAD_STREAM_STATE_SIZE :
                VSMC_RUNTIME_ASSERT(false, "VSL_RNG_ERROR_BAD_STREAM_STATE_SIZE");
                return;
            case VSL_RNG_ERROR_BAD_UPDATE :
                VSMC_RUNTIME_ASSERT(false, "VSL_RNG_ERROR_BAD_UPDATE");
                return;
            case VSL_RNG_ERROR_BAD_WORD_SIZE :
                VSMC_RUNTIME_ASSERT(false, "VSL_RNG_ERROR_BAD_WORD_SIZE");
                return;
            case VSL_RNG_ERROR_BRNG_NOT_SUPPORTED :
                VSMC_RUNTIME_ASSERT(false, "VSL_RNG_ERROR_BRNG_NOT_SUPPORTED");
                return;
            case VSL_RNG_ERROR_BRNG_TABLE_FULL :
                VSMC_RUNTIME_ASSERT(false, "VSL_RNG_ERROR_BRNG_TABLE_FULL");
                return;
            case VSL_RNG_ERROR_BRNGS_INCOMPATIBLE :
                VSMC_RUNTIME_ASSERT(false, "VSL_RNG_ERROR_BRNGS_INCOMPATIBLE");
                return;
            case VSL_RNG_ERROR_FILE_CLOSE :
                VSMC_RUNTIME_ASSERT(false, "VSL_RNG_ERROR_FILE_CLOSE");
                return;
            case VSL_RNG_ERROR_FILE_OPEN :
                VSMC_RUNTIME_ASSERT(false, "VSL_RNG_ERROR_FILE_OPEN");
                return;
            case VSL_RNG_ERROR_FILE_READ :
                VSMC_RUNTIME_ASSERT(false, "VSL_RNG_ERROR_FILE_READ");
                return;
            case VSL_RNG_ERROR_FILE_WRITE :
                VSMC_RUNTIME_ASSERT(false, "VSL_RNG_ERROR_FILE_WRITE");
                return;
            case VSL_RNG_ERROR_INVALID_ABSTRACT_STREAM :
                VSMC_RUNTIME_ASSERT(false, "VSL_RNG_ERROR_INVALID_ABSTRACT_STREAM");
                return;
            case VSL_RNG_ERROR_INVALID_BRNG_INDEX :
                VSMC_RUNTIME_ASSERT(false, "VSL_RNG_ERROR_INVALID_BRNG_INDEX");
                return;
            case VSL_RNG_ERROR_LEAPFROG_UNSUPPORTED :
                VSMC_RUNTIME_ASSERT(false, "VSL_RNG_ERROR_LEAPFROG_UNSUPPORTED");
                return;
            case VSL_RNG_ERROR_NO_NUMBERS :
                VSMC_RUNTIME_ASSERT(false, "VSL_RNG_ERROR_NO_NUMBERS");
                return;
            case VSL_RNG_ERROR_QRNG_PERIOD_ELAPSED :
                VSMC_RUNTIME_ASSERT(false, "VSL_RNG_ERROR_QRNG_PERIOD_ELAPSED");
                return;
            case VSL_RNG_ERROR_SKIPAHEAD_UNSUPPORTED :
                VSMC_RUNTIME_ASSERT(false, "VSL_RNG_ERROR_SKIPAHEAD_UNSUPPORTED");
                return;
            case VSL_RNG_ERROR_UNSUPPORTED_FILE_VER :
                VSMC_RUNTIME_ASSERT(false, "VSL_RNG_ERROR_UNSUPPORTED_FILE_VER");
                return;
            case VSL_RNG_ERROR_NONDETERM_NOT_SUPPORTED :
                VSMC_RUNTIME_ASSERT(false, "VSL_RNG_ERROR_NONDETERM_NOT_SUPPORTED");
                return;
            case VSL_RNG_ERROR_NONDETERM_NRETRIES_EXCEEDED :
                VSMC_RUNTIME_ASSERT(false, "VSL_RNG_ERROR_NONDETERM_ NRETRIES_EXCEEDED");
                return;
            default :
                return;
        }
        return;
    }
}; // class Engine

typedef Engine<VSL_BRNG_MCG59,    EngOffsetZero, EngSkipVSL<VSL_BRNG_MCG59> > MCG59;
typedef Engine<VSL_BRNG_MRG32K3A, EngOffsetZero, EngSkipVSL<VSL_BRNG_MRG32K3A> > MRG32K3A;
typedef Engine<VSL_BRNG_MT19937,   EngOffsetZero, EngSkipForce> MT19937;
typedef Engine<VSL_BRNG_SFMT19937, EngOffsetZero, EngSkipForce> SFMT19937;
typedef Engine<VSL_BRNG_MT2203, EngOffsetDynamic, EngSkipForce> MT2203;
typedef Engine<VSL_BRNG_NONDETERM, EngOffsetZero, EngSkipForce> NONDETERM;

} } // namespace vsmc::mkl

#endif // VSMC_RNG_MKL_ENGINE_HPP
