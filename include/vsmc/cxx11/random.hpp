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
