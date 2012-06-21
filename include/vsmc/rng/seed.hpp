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

    result_type get ()
    {
        return ++seed_;
    }

    void set (result_type seed)
    {
        seed_ = seed;
    }

    void skip (result_type steps)
    {
        seed_ += steps;
    }

    private :

    result_type seed_;

    Seed () : seed_(VSMC_RNG_SEED) {}
    Seed (const Seed &) {}
    const Seed &operator= (const Seed &) {return *this;}
};

} } // namespace vsmc::rng

#endif // VSMC_RNG_SEED_HPP
