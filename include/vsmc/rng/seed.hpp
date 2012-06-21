#ifndef VSMC_RNG_SEED_HPP
#define VSMC_RNG_SEED_HPP

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

    unsigned long get ()
    {
        return ++seed_;
    }

    void set (unsigned long seed)
    {
        seed_ = seed;
    }

    void skip (unsigned long steps)
    {
        seed_ += steps;
    }

    private :

    Seed () : seed_(VSMC_RNG_SEED) {}
    Seed (const Seed &) {}
    const Seed &operator= (const Seed &) {return *this;}

    unsigned long seed_;
};

} } // namespace vsmc::rng

#endif // VSMC_RNG_SEED_HPP
