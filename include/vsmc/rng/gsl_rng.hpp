//============================================================================
// include/vsmc/rng/gsl_rng.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_RNG_GSL_RNG_HPP
#define VSMC_RNG_GSL_RNG_HPP

#include <vsmc/rng/generator_wrapper.hpp>
#include <gsl/gsl_rng.h>
#include <utility>

#define VSMC_DEFINE_RNG_GSL_RNG_TYPE_POINTER(Generator, pointer) \
    template <> struct GSLRngTypePointer< GSL_RNG_TYPE_##Generator >         \
    {static const ::gsl_rng_type *get () {return ::gsl_rng_##pointer ;}};

#define VSMC_DEFINE_RNG_GSL_RNG_MIN_MAX(Generator, Min, Max) \
template <> struct GSLRngMinMax< GSL_RNG_TYPE_##Generator >                  \
{                                                                            \
    static VSMC_CONSTEXPR const uint32_t _Min =                              \
    static_cast<uint32_t>(Min##UL);                                          \
    static VSMC_CONSTEXPR const uint32_t _Max =                              \
    static_cast<uint32_t>(Max##UL);                                          \
    static VSMC_CONSTEXPR uint32_t min VSMC_MNE () {return _Min;}            \
    static VSMC_CONSTEXPR uint32_t max VSMC_MNE () {return _Max;}            \
}; // VSMC_DEFINE_RNG_GSL_RNG_MIN_MAX

namespace vsmc {

/// \brief GSL RNG algorithms
/// \ingroup GSLRNG
enum GSLRngType {
    GSL_RNG_TYPE_MT19937,   ///< `gsl_rng_mt19937`
    GSL_RNG_TYPE_RANLXS0,   ///< `gsl_rng_ranlxs0`
    GSL_RNG_TYPE_RANLXS1,   ///< `gsl_rng_ranlxs1`
    GSL_RNG_TYPE_RANLXS2,   ///< `gsl_rng_ranlxs2`
    GSL_RNG_TYPE_RANLXD1,   ///< `gsl_rng_ranlxd1`
    GSL_RNG_TYPE_RANLXD2,   ///< `gsl_rng_ranlxd2`
    GSL_RNG_TYPE_RANLUX,    ///< `gsl_rng_ranlux`
    GSL_RNG_TYPE_RANLUX389, ///< `gsl_rng_ranlux389`
    GSL_RNG_TYPE_CMRG,      ///< `gsl_rng_cmrg`
    GSL_RNG_TYPE_MRG,       ///< `gsl_rng_mrg`
    GSL_RNG_TYPE_TAUS,      ///< `gsl_rng_taus`
    GSL_RNG_TYPE_TAUS2,     ///< `gsl_rng_taus2`
    GSL_RNG_TYPE_GFSR4      ///< `gsl_rng_gfsr4`
}; // enum GSLRngType

namespace internal {

template <GSLRngType> struct GSLRngTypePointer;

VSMC_DEFINE_RNG_GSL_RNG_TYPE_POINTER(MT19937,   mt19937)
VSMC_DEFINE_RNG_GSL_RNG_TYPE_POINTER(RANLXS0,   ranlxs0)
VSMC_DEFINE_RNG_GSL_RNG_TYPE_POINTER(RANLXS1,   ranlxs1)
VSMC_DEFINE_RNG_GSL_RNG_TYPE_POINTER(RANLXS2,   ranlxs2)
VSMC_DEFINE_RNG_GSL_RNG_TYPE_POINTER(RANLXD1,   ranlxd1)
VSMC_DEFINE_RNG_GSL_RNG_TYPE_POINTER(RANLXD2,   ranlxd2)
VSMC_DEFINE_RNG_GSL_RNG_TYPE_POINTER(RANLUX,    ranlux)
VSMC_DEFINE_RNG_GSL_RNG_TYPE_POINTER(RANLUX389, ranlux389)
VSMC_DEFINE_RNG_GSL_RNG_TYPE_POINTER(CMRG,      cmrg)
VSMC_DEFINE_RNG_GSL_RNG_TYPE_POINTER(MRG,       mrg)
VSMC_DEFINE_RNG_GSL_RNG_TYPE_POINTER(TAUS,      taus)
VSMC_DEFINE_RNG_GSL_RNG_TYPE_POINTER(TAUS2,     taus2)
VSMC_DEFINE_RNG_GSL_RNG_TYPE_POINTER(GFSR4,     gfsr4)

template <GSLRngType RngType> struct GSLRngMinMax;

VSMC_DEFINE_RNG_GSL_RNG_MIN_MAX(MT19937,   0, 0xFFFFFFFF)
VSMC_DEFINE_RNG_GSL_RNG_MIN_MAX(RANLXS0,   0, 0x00FFFFFF)
VSMC_DEFINE_RNG_GSL_RNG_MIN_MAX(RANLXS1,   0, 0x00FFFFFF)
VSMC_DEFINE_RNG_GSL_RNG_MIN_MAX(RANLXS2,   0, 0x00FFFFFF)
VSMC_DEFINE_RNG_GSL_RNG_MIN_MAX(RANLXD1,   0, 0xFFFFFFFF)
VSMC_DEFINE_RNG_GSL_RNG_MIN_MAX(RANLXD2,   0, 0xFFFFFFFF)
VSMC_DEFINE_RNG_GSL_RNG_MIN_MAX(RANLUX,    0, 0x00FFFFFF)
VSMC_DEFINE_RNG_GSL_RNG_MIN_MAX(RANLUX389, 0, 0x00FFFFFF)
VSMC_DEFINE_RNG_GSL_RNG_MIN_MAX(CMRG,      0, 0x7FFFFFFE)
VSMC_DEFINE_RNG_GSL_RNG_MIN_MAX(MRG,       0, 0x7FFFFFFE)
VSMC_DEFINE_RNG_GSL_RNG_MIN_MAX(TAUS,      0, 0xFFFFFFFF)
VSMC_DEFINE_RNG_GSL_RNG_MIN_MAX(TAUS2,     0, 0xFFFFFFFF)
VSMC_DEFINE_RNG_GSL_RNG_MIN_MAX(GFSR4,     0, 0xFFFFFFFF)
} // namespace vsmc::internal

/// \brief GSL RNG generator for use with GeneratorWrapper
/// \ingroup GSLRNG
template <GSLRngType RngType>
class GSLGenerator
{
    public :

    GSLGenerator () :
        rng_(::gsl_rng_alloc(internal::GSLRngTypePointer<RngType>::get())) {}

    GSLGenerator (const GSLGenerator<RngType> &other) :
        rng_(::gsl_rng_clone(other.rng_)) {}

    GSLGenerator<RngType> &operator= (const GSLGenerator<RngType> &other)
    {
        if (this != &other)
            ::gsl_rng_memcpy(rng_, other.rng_);

        return *this;
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    GSLGenerator (GSLGenerator<RngType> &&other) :
        rng_(other.rng_) {other.rng_ = VSMC_NULLPTR;}

    GSLGenerator<RngType> &operator= (GSLGenerator<RngType> &&other)
    {
        using std::swap;

        if (this != &other)
            swap(rng_, other.rng_);

        return *this;
    }
#endif

    ~GSLGenerator ()
    {
        if (rng_ != VSMC_NULLPTR)
            ::gsl_rng_free(rng_);
    }

    void seed (unsigned long s)
    {
        if (rng_ != VSMC_NULLPTR)
            ::gsl_rng_set(rng_, s);
    }

    unsigned long generate ()
    {
        if (rng_ != VSMC_NULLPTR)
            return ::gsl_rng_get(rng_);

        return 0;
    }

    unsigned long min VSMC_MNE () const {return ::gsl_rng_min(rng_);}
    unsigned long max VSMC_MNE () const {return ::gsl_rng_max(rng_);}

    private :

    ::gsl_rng *rng_;
}; // class GSLGenerator

/// \brief GSL RNG Engine
/// \ingroup GSLRNG
template <GSLRngType RngType>
class GSLEngine :
    public GeneratorWrapper<uint32_t, GSLGenerator<RngType>,
    internal::GSLRngMinMax<RngType> >
{
    typedef GeneratorWrapper<uint32_t, GSLGenerator<RngType>,
    internal::GSLRngMinMax<RngType> > base;

    public :

    typedef uint32_t result_type;

    GSLEngine (result_type s = 0) : base(s) {seed(s);}

    template <typename SeedSeq>
    explicit GSLEngine (SeedSeq &seq, typename cxx11::enable_if<
            internal::is_seed_seq<SeedSeq, result_type, GSLEngine<RngType>
            >::value>::type * = VSMC_NULLPTR) : base(seq) {seed(seq);}

    void seed (result_type s)
    {this->generator().seed(static_cast<unsigned long>(s));}

    template <typename SeedSeq>
    void seed (SeedSeq &seq, typename cxx11::enable_if<
            internal::is_seed_seq<SeedSeq, result_type>::value>::type * =
            VSMC_NULLPTR)
    {
        unsigned long s;
        seq.generate(&s, &s + 1);
        this->generator().seed(s);
    }
}; // class GSLEngine

/// \brief A Mersenne-Twister pseudoranom number genertor
/// \ingroup GSLRNG
typedef GSLEngine<GSL_RNG_TYPE_MT19937> GSL_MT19937;

/// \brief A RANLUX generator with luxury level 0
/// \ingroup GSLRNG
typedef GSLEngine<GSL_RNG_TYPE_RANLXS0> GSL_RANLXS0;

/// \brief A RANLUX generator with luxury level 0
/// \ingroup GSLRNG
typedef GSLEngine<GSL_RNG_TYPE_RANLXS1> GSL_RANLXS1;

/// \brief A RANLUX generator with luxury level 0
/// \ingroup GSLRNG
typedef GSLEngine<GSL_RNG_TYPE_RANLXS2> GSL_RANLXS2;

/// \brief A RANLXS generator with luxury level 1
/// \ingroup GSLRNG
typedef GSLEngine<GSL_RNG_TYPE_RANLXD1> GSL_RANLXD1;

/// \brief A RANLXS generator with luxury level 2
/// \ingroup GSLRNG
typedef GSLEngine<GSL_RNG_TYPE_RANLXD2> GSL_RANLXD2;

/// \brief A RANLUX generator
/// \ingroup GSLRNG
typedef GSLEngine<GSL_RNG_TYPE_RANLUX> GSL_RANLUX;

/// \brief A RANLUX generator with the highest level of randomness
/// \ingroup GSLRNG
typedef GSLEngine<GSL_RNG_TYPE_RANLUX389> GSL_RANLUX389;

/// \brief A combined multiple recursive generator
/// \ingroup GSLRNG
typedef GSLEngine<GSL_RNG_TYPE_CMRG> GSL_CMRG;

/// \brief A fifth-order multiple recursive generator
/// \ingroup GSL
typedef GSLEngine<GSL_RNG_TYPE_MRG> GSL_MRG;

/// \brief A maximally equidistributed combined Tausworthe generator
/// \ingroup GSLRNG
typedef GSLEngine<GSL_RNG_TYPE_TAUS> GSL_TAUS;

/// \brief A maximally equidistributed combined Tausworthe generator with
/// improved seeding procedure
/// \ingroup GSLRNG
typedef GSLEngine<GSL_RNG_TYPE_TAUS2> GSL_TAUS2;

/// \brief A a lagged-fibonacci alike generator
/// \ingroup GSLRNG
typedef GSLEngine<GSL_RNG_TYPE_GFSR4> GSL_GFSR4;

} // namespace vsmc

#endif // VSMC_RNG_GSL_RNG_HPP
