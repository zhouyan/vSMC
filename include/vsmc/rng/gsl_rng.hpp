#ifndef VSMC_RNG_GSL_RNG_HPP
#define VSMC_RNG_GSL_RNG_HPP

#include <vsmc/rng/generator_wrapper.hpp>
#include <gsl/gsl_rng.h>

namespace vsmc {

/// \brief GSL RNG algorithms
/// \ingroup GSLRNG
enum GSLRngType {
    GSL_RNG_TYPE_MT19937,    ///< gsl_rng_mt19937
    GSL_RNG_TYPE_RANLXS0,    ///< gsl_rng_ranlxs0
    GSL_RNG_TYPE_RANLXS1,    ///< gsl_rng_ranlxs1
    GSL_RNG_TYPE_RANLXS2,    ///< gsl_rng_ranlxs2
    GSL_RNG_TYPE_RANLXD1,    ///< gsl_rng_ranlxd1
    GSL_RNG_TYPE_RANLXD2,    ///< gsl_rng_ranlxd2
    GSL_RNG_TYPE_RANLUX,     ///< gsl_rng_ranlux
    GSL_RNG_TYPE_RANLUX389,  ///< gsl_rng_ranlux389
    GSL_RNG_TYPE_CMRG,       ///< gsl_rng_cmrg
    GSL_RNG_TYPE_MRG,        ///< gsl_rng_mrg
    GSL_RNG_TYPE_TAUS,       ///< gsl_rng_taus
    GSL_RNG_TYPE_TAUS2,      ///< gsl_rng_taus2
    GSL_RNG_TYPE_GFSR4       ///< gsl_rng_gfsr4
};

namespace internal {

template <GSLRngType> struct GSLRngTypePointer;

template <> struct GSLRngTypePointer<GSL_RNG_TYPE_MT19937>
{static const gsl_rng_type *get () {return gsl_rng_mt19937;}};

template <> struct GSLRngTypePointer<GSL_RNG_TYPE_RANLXS0>
{static const gsl_rng_type *get () {return gsl_rng_ranlxs0;}};

template <> struct GSLRngTypePointer<GSL_RNG_TYPE_RANLXS1>
{static const gsl_rng_type *get () {return gsl_rng_ranlxs1;}};

template <> struct GSLRngTypePointer<GSL_RNG_TYPE_RANLXS2>
{static const gsl_rng_type *get () {return gsl_rng_ranlxs2;}};

template <> struct GSLRngTypePointer<GSL_RNG_TYPE_RANLXD1>
{static const gsl_rng_type *get () {return gsl_rng_ranlxd1;}};

template <> struct GSLRngTypePointer<GSL_RNG_TYPE_RANLXD2>
{static const gsl_rng_type *get () {return gsl_rng_ranlxd2;}};

template <> struct GSLRngTypePointer<GSL_RNG_TYPE_RANLUX>
{static const gsl_rng_type *get () {return gsl_rng_ranlux;}};

template <> struct GSLRngTypePointer<GSL_RNG_TYPE_RANLUX389>
{static const gsl_rng_type *get () {return gsl_rng_ranlux389;}};

template <> struct GSLRngTypePointer<GSL_RNG_TYPE_CMRG>
{static const gsl_rng_type *get () {return gsl_rng_cmrg;}};

template <> struct GSLRngTypePointer<GSL_RNG_TYPE_MRG>
{static const gsl_rng_type *get () {return gsl_rng_mrg;}};

template <> struct GSLRngTypePointer<GSL_RNG_TYPE_TAUS>
{static const gsl_rng_type *get () {return gsl_rng_taus;}};

template <> struct GSLRngTypePointer<GSL_RNG_TYPE_TAUS2>
{static const gsl_rng_type *get () {return gsl_rng_taus2;}};

template <> struct GSLRngTypePointer<GSL_RNG_TYPE_GFSR4>
{static const gsl_rng_type *get () {return gsl_rng_gfsr4;}};

template <GSLRngType RngType>
class GSLRngGenerator
{
    public :

    GSLRngGenerator () :
        rng_(gsl_rng_alloc(internal::GSLRngTypePointer<RngType>::get())) {}

    GSLRngGenerator (const GSLRngGenerator<RngType> &other) :
        rng_(gsl_rng_clone(other.rng_)) {}

    GSLRngGenerator<RngType> &operator= (const GSLRngGenerator<RngType> &other)
    {
        if (this != &other) {
            if (rng_ != VSMC_NULLPTR)
                gsl_rng_free(rng_);
            rng_ = gsl_rng_memcpy(other.rng_);
        }

        return *this;
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    GSLRngGenerator (GSLRngGenerator<RngType> &&other) :
        rng_(other.rng_) {other.rng_ = VSMC_NULLPTR;}

    GSLRngGenerator<RngType> &operator= (GSLRngGenerator<RngType> &&other)
    {
        if (this != &other) {
            if (rng_ != VSMC_NULLPTR)
                gsl_rng_free(rng_);
            rng_ = other.rng_;
            other.rng_ = VSMC_NULLPTR;
        }

        return *this;
    }
#endif

    ~GSLRngGenerator ()
    {
        if (rng_ != VSMC_NULLPTR)
            gsl_rng_free(rng_);
    }

    void seed (unsigned long s)
    {
        if (rng_ != VSMC_NULLPTR)
            gsl_rng_set(rng_, s);
    }

    unsigned long generate ()
    {
        if (rng_ != VSMC_NULLPTR)
            return gsl_rng_get(rng_);

        return 0;
    }

    unsigned long min VSMC_MNE () const {return gsl_rng_min(rng_);}

    unsigned long max VSMC_MNE () const {return gsl_rng_max(rng_);}

    private :

    gsl_rng *rng_;
};

template <GSLRngType RngType>
struct GSLRngMinMax
{
    uint32_t min VSMC_MNE ()
    {
        return static_cast<uint32_t>(static_cast<
                GeneratorWrapper<unsigned long,
                GSLRngGenerator<RngType>,
                GSLRngMinMax<RngType> > >(this)->generator().min VSMC_MNE ());
    }

    uint32_t max VSMC_MNE ()
    {
        return static_cast<uint32_t>(static_cast<
                GeneratorWrapper<unsigned long,
                GSLRngGenerator<RngType>,
                GSLRngMinMax<RngType> > >(this)->generator().max VSMC_MNE ());
    }
};

template <GSLRngType RngType> struct GSLRngMinMaxTrait
{typedef GSLRngMinMax<RngType> type;};

template <> struct GSLRngMinMaxTrait<GSL_RNG_TYPE_MT19937>
{typedef ::vsmc::traits::GeneratorWrapperMinMaxTrait<uint32_t> type;};

template <> struct GSLRngMinMaxTrait<GSL_RNG_TYPE_RANLXD1>
{typedef ::vsmc::traits::GeneratorWrapperMinMaxTrait<uint32_t> type;};

template <> struct GSLRngMinMaxTrait<GSL_RNG_TYPE_RANLXD2>
{typedef ::vsmc::traits::GeneratorWrapperMinMaxTrait<uint32_t> type;};

template <> struct GSLRngMinMaxTrait<GSL_RNG_TYPE_TAUS>
{typedef ::vsmc::traits::GeneratorWrapperMinMaxTrait<uint32_t> type;};

template <> struct GSLRngMinMaxTrait<GSL_RNG_TYPE_TAUS2>
{typedef ::vsmc::traits::GeneratorWrapperMinMaxTrait<uint32_t> type;};

template <> struct GSLRngMinMaxTrait<GSL_RNG_TYPE_GFSR4>
{typedef ::vsmc::traits::GeneratorWrapperMinMaxTrait<uint32_t> type;};

} // namespace vsmc::internal

/// \brief GSL RNG Engine
/// \ingroup GSLRNG
template <GSLRngType RngType>
class GSLRngEngine :
    public GeneratorWrapper<uint32_t, internal::GSLRngGenerator<RngType>,
    typename internal::GSLRngMinMaxTrait<RngType>::type>
{
    typedef GeneratorWrapper<uint32_t, internal::GSLRngGenerator<RngType>,
            typename internal::GSLRngMinMaxTrait<RngType>::type> base_type;

    public :

    typedef uint32_t result_type;

    GSLRngEngine (result_type s = 0) : base_type(s) {seed(s);}

    template <typename SeedSeq>
    explicit GSLRngEngine (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_sequence<SeedSeq, result_type>::value>::type * =
            VSMC_NULLPTR) : base_type(seq) {seed(seq);}

    void seed (result_type s)
    {this->generator().seed(static_cast<unsigned long>(s));}

    template <typename SeedSeq>
    void seed (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_sequence<SeedSeq, result_type>::value>::type * =
            VSMC_NULLPTR)
    {
        unsigned long s;
        seq.generate(&s, &s + 1);
        this->generator().seed(s);
    }
}; // class GSLRngEngine

/// \brief A Mersenne-Twister pseudoranom number genertor
/// \ingroup GSLRNG
typedef GSLRngEngine<GSL_RNG_TYPE_MT19937> GSL_MT19937;

/// \brief A RANLXS with luxury level 1
/// \ingroup GSLRNG
typedef GSLRngEngine<GSL_RNG_TYPE_RANLXD1> GSL_RANLXD1;

/// \brief A RANLXS with luxury level 2
/// \ingroup GSLRNG
typedef GSLRngEngine<GSL_RNG_TYPE_RANLXD2> GSL_RANLXD2;

/// \brief A maximally equidistributed combined Tausworthe generator
/// \ingroup GSLRNG
typedef GSLRngEngine<GSL_RNG_TYPE_TAUS> GSL_TAUS;

/// \brief A maximally equidistributed combined Tausworthe generator with
/// improved seeding procedure
/// \ingroup GSLRNG
typedef GSLRngEngine<GSL_RNG_TYPE_TAUS2> GSL_TAUS2;

/// \brief A a lagged-fibonacci alike generator
/// \ingroup GSLRNG
typedef GSLRngEngine<GSL_RNG_TYPE_GFSR4> GSL_GFSR4;

} // namespace vsmc

#endif // VSMC_RNG_GSL_RNG_HPP
