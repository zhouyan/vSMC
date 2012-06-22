#ifndef VSMC_RNG_SEED_HPP
#define VSMC_RNG_SEED_HPP

#include <vsmc/rng/common.hpp>
#include <ctime>
#include <cstdlib>

namespace vsmc { namespace rng {

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
/// \f$2^32 - 1\f$ independent random streams when used with Random123. All
/// public options are of constant complexity. For Pseudo random number
/// generators, independent seeds would need much higher computational
/// complexity to generate.
///
/// \note Note that currently all interface of Seed are \b not thread-safe.
class Seed
{
    public :

    typedef unsigned result_type;

    static Seed &create ()
    {
        static Seed seed;

        return seed;
    }

    result_type get ()
    {
        if (seed_  >= std::numeric_limits<result_type>::max() - 1)
            seed_ = 0;

        return ++seed_;
    }

    void set (result_type seed)
    {
        seed_ = seed;
    }

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
};

} } // namespace vsmc::rng

#endif // VSMC_RNG_SEED_HPP
