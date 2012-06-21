#ifndef VSMC_RNG_SEED_HPP
#define VSMC_RNG_SEED_HPP

#include <vsmc/rng/common.hpp>
#include <ctime>
#include <cstdlib>

namespace vsmc { namespace rng {

class Seed
{
    public :

    typedef unsigned long result_type;

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
