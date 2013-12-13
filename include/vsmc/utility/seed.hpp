#ifndef VSMC_UTILITY_SEED_HPP
#define VSMC_UTILITY_SEED_HPP

#include <vsmc/internal/common.hpp>

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

    result_type get () {skip(1); return seed_ * div_ + rem_;}

    void set (result_type seed)
    {seed_ = seed < seed_max_ ? seed : seed - seed_max_;}

    /// \brief Subsequent seed \f$S\f$ will all belong to equalivant class
    /// \f$S\ \mathrm{mod}\ \mathtt{div} \equiv \mathtt{rem}\f$
    void modulo (result_type div, result_type rem)
    {
        seed_max_ = std::numeric_limits<result_type>::max
            VSMC_MACRO_NO_EXPANSION ();
        seed_max_ -= rem_;
        seed_max_ -= seed_max_ % div_;
        seed_max_ /= div_;

        div_ = div;
        rem_ = rem;
        set(seed_);
    }

    void skip (result_type steps)
    {
        if (steps > seed_max_ - seed_)
            seed_ = steps - (seed_max_ - seed_);
        else
            seed_ += steps;
    }

    private :

    result_type seed_;
    result_type seed_max_;
    result_type div_;
    result_type rem_;

    Seed () :
        seed_(0), seed_max_(std::numeric_limits<result_type>::max
                VSMC_MACRO_NO_EXPANSION ()), div_(1), rem_(0) {}

    Seed (const Seed &);
    Seed &operator= (const Seed &);
}; // class Seed

} // namespace vsmc

#endif // VSMC_UTILITY_SEED_HPP
