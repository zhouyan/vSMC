#ifndef VSMC_CXX11_RANDOM_HPP
#define VSMC_CXX11_RANDOM_HPP

#include <vsmc/internal/config.hpp>
#include <cstdlib>
#include <ctime>
#include <limits>

#if VSMC_HAS_CXX11LIB_RANDOM
#include <random>
#else // VSMC_HAS_CXX11LIB_RANDOM
#include <boost/random.hpp>
#endif // VSMC_HAS_CXX11LIB_RANDOM

#if VSMC_USE_RANDOM123
#include <Random123/philox.h>
#include <Random123/threefry.h>
#include <Random123/conventional/Engine.hpp>
#endif

namespace vsmc {

namespace cxx11 {

#if VSMC_HAS_CXX11LIB_RANDOM

using std::minstd_rand0;
using std::minstd_rand;
using std::mt19937;
using std::mt19937_64;
using std::ranlux24_base;
using std::ranlux48_base;
using std::ranlux24;
using std::ranlux48;
using std::knuth_b;

using std::uniform_int_distribution;
using std::uniform_real_distribution;

using std::bernoulli_distribution;
using std::binomial_distribution;
using std::geometric_distribution;
using std::negative_binomial_distribution;

using std::exponential_distribution;
using std::extreme_value_distribution;
using std::gamma_distribution;
using std::poisson_distribution;
using std::weibull_distribution;

using std::cauchy_distribution;
using std::chi_squared_distribution;
using std::fisher_f_distribution;
using std::lognormal_distribution;
using std::normal_distribution;
using std::student_t_distribution;

using std::discrete_distribution;
using std::piecewise_constant_distribution;
using std::piecewise_linear_distribution;

#else // VSMC_HAS_CXX11LIB_RANDOM

using boost::random::minstd_rand0;
using boost::random::minstd_rand;
using boost::random::mt19937;
using boost::random::mt19937_64;
using boost::random::ranlux24_base;
using boost::random::ranlux48_base;
typedef boost::random::subtract_with_carry_engine<uint32_t, 24, 10, 24>
    ranlux24_base;
typedef boost::random::subtract_with_carry_engine<uint64_t, 48, 5, 12>
    ranlux48_base;
using boost::random::ranlux24;
using boost::random::ranlux48;
using boost::random::knuth_b;

using boost::random::uniform_int_distribution;
using boost::random::uniform_real_distribution;

typedef boost::random::bernoulli_distribution<double> bernoulli_distribution;
using boost::random::binomial_distribution;
using boost::random::geometric_distribution;
using boost::random::negative_binomial_distribution;

using boost::random::exponential_distribution;
using boost::random::extreme_value_distribution;
using boost::random::gamma_distribution;
using boost::random::poisson_distribution;
using boost::random::weibull_distribution;

using boost::random::cauchy_distribution;
using boost::random::chi_squared_distribution;
using boost::random::fisher_f_distribution;
using boost::random::lognormal_distribution;
using boost::random::normal_distribution;
using boost::random::student_t_distribution;

using boost::random::discrete_distribution;
using boost::random::piecewise_constant_distribution;
using boost::random::piecewise_linear_distribution;

#endif // VSMC_HAS_CXX11LIB_RANDOM

#if VSMC_USE_RANDOM123

#ifdef VSMC_USE_CONSTEXPR_ENGINE

#if !VSMC_HAS_CXX11_CONSTEXPR
#error VSMC_HAS_CXX11_CONSTEXPR has to be defined to a non-zero value
#endif

/// \brief An Engine with constexpr min() and max()
/// \ingroup RNG
///
/// \details
/// vsmc::rng::Engine is an alias to r123::Engine unless the macro \c
/// VSMC_USE_CONSTEXPR_ENGINE is explicited defined before the including of
/// this header. One shall never define this macro unless it is absolutely
/// needed as explained below.
///
/// The Engine distributed by Random123 does not return \c min() and \c max()
/// as \c constexpr for a good reason. However this will be needed if \c libc++
/// is used. This mini Engine is derived from r123::Engine. The base type is
/// not actually intended to be used as a base class. As this derived class
/// does not have any data of its own. It shall be fine to use it even through
/// base type pointers, though that is certainly a bad practice. However when
/// \c min() and \c max() are of concern, one shall use the derived type to
/// invoke the static members. In furture we may write a entirely new class for
/// this case but that will be reinventing the wheel which Random123 has
/// already done. So that won't happen in any near future.
template <typename CBRNG>
class Engine : public r123::Engine<CBRNG>
{
    public :

    typedef typename r123:Engine<CBRNG> engine_type;

    typedef typename engine_type::cbrng_type  cbrng_type;
    typedef typename engine_type::ctr_type    ctr_type;
    typedef typename engine_type::key_type    key_type;
    typedef typename engine_type::ukey_type   ukey_type;
    typedef typename engine_type::result_type result_type;
    typedef typename engine_type::elem_type   elem_type;

    explicit r123_engine () : engine_type() {}
    explicit r123_engine (result_type r) : engine_type(r) {}

    template <typename SeedSeq>
    explicit r123_engine (SeedSeq &seed_seq) : engine_type(seed_seq) {}

    static constexpr result_type min VSMC_PREVENT_MIN_MAX ()
    {
        return 0;
    }

    static constexpr result_type max VSMC_PREVENT_MIN_MAX ()
    {
        return std::numeric_limits<result_type>::max VSMC_PREVENT_MIN_MAX ();
    }

}; // class r123_engine

#else // VSMC_USE_CONSTEXPR_ENGINE

using r123::Engine;

#endif // VSMC_USE_CONSTEXPR_ENGINE

#endif // VSMC_USE_RANDOM123

} // namespace vsmc::cxx11

/// \brief Seed generator
/// \ingroup RNG
///
/// \details
/// A Seed object cannot be created, copied, or assigned to/from by user.
/// Instead, user can only get a reference to a \c static Seed object through
/// \c Seed::create(). It is intended to generate distinct seed for the
/// Counter-based random number generator provided by Random123 library. The
/// seed generated from the Seed object shall not be used by other Pseudo
/// random number generator. The seed sequence is just a sequence of \c
/// unsigned integers. On most platforms this means it can used to create
/// \f$2^{32} - 1\f$ independent random streams when used with Random123. All
/// public members are of constant complexity. For Pseudo random number
/// generators, independent seeds would need much higher computational
/// complexity to generate.
///
/// \note Note that currently all interface of Seed are \b not thread-safe.
class Seed
{
    public :

    /// The type of the seed generated
    typedef unsigned result_type;

    /// Get a reference to a Seed object
    static Seed &create ()
    {
        static Seed seed;

        return seed;
    }

    /// \brief Get a seed
    ///
    /// \note If the integer space is exhausted, then it will start form zero
    /// and continue.
    result_type get ()
    {
        if (seed_  >= std::numeric_limits<result_type>::max() - 1)
            seed_ = 0;

        return ++seed_;
    }

    /// \brief Set the seed
    ///
    /// \param seed The new seed
    ///
    /// \note The next call to get() will return <tt>seed + 1</tt> or \c 1
    /// if <tt>seed >= std::numeric_limits<result_type>::max() - 1)</tt>
    void set (result_type seed)
    {
        seed_ = seed;
    }

    /// \brief Skip a sequence of seeds
    ///
    /// \param steps The number of steps to skip
    ///
    /// \note This operation is of constant complexity, but it is equivalent to
    /// call get() \c steps times.
    void skip (result_type steps)
    {
        if (seed_ >= std::numeric_limits<result_type>::max() - steps)
            seed_ = steps;
        else
            seed_ += steps;
    }

    private :

    result_type seed_;

    Seed () : seed_(VSMC_RNG_SEED)
    {
        if (!seed_)
            seed_ = std::rand();
    }

    Seed (const Seed &) {}

    const Seed &operator= (const Seed &) {return *this;}
}; // class Seed

} // namespae vsmc

#endif // VSMC_CXX11_RANDOM_HPP
