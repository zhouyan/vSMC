#ifndef VSMC_UTILITY_SEED_HPP
#define VSMC_UTILITY_SEED_HPP

namespace vsmc {

/// \brief Seed generator
/// \ingroup RNG
class Seed
{
    public :

    typedef unsigned result_type;

    static Seed &instance ()
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

    Seed (const Seed &);
    Seed &operator= (const Seed &);
    ~Seed () {};
}; // class Seed

} // namespace vsmc

#endif // VSMC_UTILITY_SEED_HPP
