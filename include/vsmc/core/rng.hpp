#ifndef VSMC_CORE_RNG_HPP
#define VSMC_CORE_RNG_HPP

#include <vsmc/internal/common.hpp>

#if VSMC_USE_RANDOM123
#include <Random123/philox.h>
#include <Random123/threefry.h>
#include <Random123/conventional/Engine.hpp>
#endif

namespace vsmc {

/// \brief Seed generator
/// \ingroup RNG
///
/// \details
/// A Seed object cannot be created, copied, or assigned to/from by user.
/// Instead, user can only get a reference to a \c static Seed object through
/// \c Seed::reference(). It is intended to generate distinct seed for the
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
    static Seed &reference ()
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

    Seed (const Seed &);
    Seed &operator= (const Seed &);
    ~Seed () {};
}; // class Seed

/// \brief Sequential RNG set class
/// \ingroup Core
class RngSetSeq
{
    public :

    /// The type of the random number generator C++11 engine
    typedef VSMC_SEQRNG_TYPE rng_type;

    /// The type of the seed sequence
    typedef VSMC_SEED_TYPE seed_type;

    /// The type of the size of the rng set
    typedef VSMC_SIZE_TYPE size_type;

    explicit RngSetSeq (size_type N) :
        rng_(static_cast<rng_type::result_type>(
                    seed_type::reference().get()))
    {}

    /// \brief Get a C++11 RNG engine
    ///
    /// \param id The position of the particle, 0 to size() - 1
    ///
    /// \return A reference to a C++11 RNG engine unique to particle at
    /// position id, and independent of others
    rng_type &rng (size_type id)
    {
        return rng_;
    }

    private :

    rng_type rng_;
}; // class RngSetSeq

/// \brief Parallel RNG set class
/// \ingroup Core
class RngSetPrl
{
    public :

    /// The type of the random number generator C++11 engine
    typedef VSMC_PRLRNG_TYPE rng_type;

    /// The type of the seed sequence
    typedef VSMC_SEED_TYPE seed_type;

    /// The type of the size of the rng set
    typedef std::vector<rng_type>::size_type size_type;

    explicit RngSetPrl (size_type N)
    {
        seed_type &seed = seed_type::reference();
        for (size_type i = 0; i != N; ++i)
            rng_.push_back(
                    rng_type(static_cast<rng_type::result_type>(seed.get())));
    }

    /// \brief Get a C++11 RNG engine
    ///
    /// \param id The position of the particle, 0 to size() - 1
    ///
    /// \return A reference to a C++11 RNG engine unique to particle at
    /// position id, and independent of others
    rng_type &rng (size_type id)
    {
        return rng_[id];
    }

    private :

    std::vector<rng_type> rng_;
}; // class RngSetPrl

} // namespace vsmc

VSMC_DEFINE_TYPE_DISPATCH_TRAIT(RngSetType, rng_set_type, RngSetPrl);

#endif // VSMC_CORE_RNG_HPP
