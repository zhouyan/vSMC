//============================================================================
// include/vsmc/rng/seed.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_RNG_SEED_HPP
#define VSMC_RNG_SEED_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/utility/array.hpp>
#include <vsmc/utility/counter.hpp>

#define VSMC_STATIC_ASSERT_RNG_SEED_GENERATOR_RESULT_TYPE(ResultType) \
    VSMC_STATIC_ASSERT((cxx11::is_unsigned<ResultType>::value),              \
            USE_SeedGenerator_WITH_A_RESULT_TYPE_NOT_AN_UNSIGNED_INTEGER)

#define VSMC_RUNTIME_ASSERT_RNG_SEED_GENERATOR_MODULO(div, rem) \
    VSMC_RUNTIME_ASSERT((div > rem),                                         \
            ("**SeedGenerator::modulo** "                                    \
             "REMAINDER IS NOT SMALLER THAN THE DIVISOR"))

#define VSMC_RUNTIME_ASSERT_RNG_SEED_MAX(seed_max) \
    VSMC_RUNTIME_ASSERT((seed_max > 1),                                      \
            ("**SeedGenerator::modulo** "                                    \
             "THE MAXIMUM OF THE INTERNAL SEED IS NO LARGER THAN 1"))

/// \brief Default result type of Seed
/// \ingroup Config
#ifndef VSMC_SEED_RESULT_TYPE
#define VSMC_SEED_RESULT_TYPE unsigned
#endif

namespace vsmc {

/// \brief Seed generator
/// \ingroup RNG
///
/// \details
/// Let \f$S\f$ be the current internal seed, an integer in the range between 1
/// and \f$S_{\mathrm{max}}\f$ where \f$S_{\mathrm{max}}\f$ is the maximum of
/// the internal seed. Each time `get()` is called, the internal is first
/// increased by 1. If it is already equal to \f$S_{\mathrm{max}}\f$, then it
/// is set to 1. The output seed is \f$S \times D + R\f$ where \f$D\f$ is the
/// divisor and \f$R\f$ is the remainder. In other words, the output seed
/// belongs to the equivalent class \f$S\ \mathrm{mod}\ D \equiv R\f$. The
/// divisor and the remainder can be set through the `modulo(div, rem)` member
/// function.
///
/// This property is useful when using programming models such as MPI where one
/// want to have distinct seeds across nodes. Each process can then configure
/// the SeedGenerator using the `modulo` member such that \f$D\f$ is the number
/// of total nodes and \f$R\f$ is the rank of each node, counting from zero.
///
/// For multithreading programs, the access to this member function shall be
/// protected by mutex, if it is called from multiple threads. Or one can use
/// distinct type `ID` to initialized distinct SeedGenerator instances.
template <typename ID, typename ResultType = VSMC_SEED_RESULT_TYPE>
class SeedGenerator
{
    public :

    typedef ResultType result_type;

    static SeedGenerator<ID, ResultType> &instance ()
    {
        static SeedGenerator<ID, ResultType> seed;

        return seed;
    }

    /// \brief Get a new seed
    result_type get () {skip(); return seed_ * divisor_ + remainder_;}

    /// \brief Set the internal seed
    ///
    /// \details
    /// If `seed` is larger than the maximum of the internal seed, than it will
    /// be round up to fit into the range. For example, say the range is from 1
    /// to 1000, and the new internal seed is 6061, it will be round up to 61.
    void set (result_type seed) {seed_ = seed % seed_max_;}

    /// \brief The current internal seed
    result_type seed () const {return seed_;}

    /// \brief The maximum of the internal seed integer
    result_type seed_max () const {return seed_max_;}

    /// \brief The divisor of the output seed
    result_type divisor () const {return divisor_;}

    /// \brief The remainder of the output seed
    result_type remainder () const {return remainder_;}

    /// \brief Set the divisor and the remainder
    void modulo (result_type div, result_type rem)
    {
        VSMC_RUNTIME_ASSERT_RNG_SEED_GENERATOR_MODULO(div, rem);

        divisor_ = div;
        remainder_ = rem;
        seed_max_ = static_cast<ResultType>(~static_cast<ResultType>(0));
        seed_max_ -= seed_max_ % divisor_;
        seed_max_ /= divisor_;

        VSMC_RUNTIME_ASSERT_RNG_SEED_MAX(seed_max_);

        set(seed_);
    }

    /// \brief Skip the internal seed by a given steps
    void skip (result_type steps)
    {
        result_type incr = steps % seed_max_;
        result_type diff = seed_max_ - seed_;
        seed_ = incr <= diff ? (seed_ + incr) : (incr - diff);
    }

    /// \brief Skip the internal seed by 1 step
    void skip () {seed_ = seed_ < seed_max_ ? (seed_ + 1) : 1;}

    private :

    result_type seed_;
    result_type divisor_;
    result_type remainder_;
    result_type seed_max_;

    SeedGenerator () :
        seed_(0), divisor_(1), remainder_(0),
        seed_max_(static_cast<result_type>(~(static_cast<result_type>(0))))
    {VSMC_STATIC_ASSERT_RNG_SEED_GENERATOR_RESULT_TYPE(ResultType);}

    SeedGenerator (const SeedGenerator<ID, ResultType> &);

    SeedGenerator<ID, ResultType> &operator= (
            const SeedGenerator<ID, ResultType> &);
}; // class SeedGenerator

template <typename, typename> class SeedCounterGenerator;

/// \brief Seed generator counters
/// \ingroup RNG
template <typename ID, typename T, std::size_t K, typename Traits>
class SeedCounterGenerator<ID, Array<T, K, Traits> >
{
    public :

    typedef Array<T, K, Traits> result_type;

    static SeedCounterGenerator<ID, Array<T, K, Traits> > &instance ()
    {
        static SeedCounterGenerator<ID, Array<T, K, Traits> > seed;

        return seed;
    }

    result_type get () {Counter<result_type>::increment(seed_); return seed_;}

    void set (result_type seed) {seed_ = seed;}

    private :

    result_type seed_;

    SeedCounterGenerator () {Counter<result_type>::reset(seed_);}

    SeedCounterGenerator<ID, Array<T, K, Traits> > (
            const SeedCounterGenerator<ID, Array<T, K, Traits> > &);

    SeedCounterGenerator<ID, Array<T, K, Traits> > &operator= (
            const SeedCounterGenerator<ID, Array<T, K, Traits> > &);
}; // class SeedCounterGenerator

/// \brief The default Seed type
/// \ingroup RNG
typedef SeedGenerator<NullType, VSMC_SEED_RESULT_TYPE> Seed;

} // namespace vsmc

#endif // VSMC_RNG_SEED_HPP
