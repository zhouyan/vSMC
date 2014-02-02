#ifndef VSMC_RNG_SEED_HPP
#define VSMC_RNG_SEED_HPP

#include <vsmc/internal/common.hpp>

#define VSMC_RUNTIME_ASSERT_RNG_SEED_GENERATOR_MODULO(div, rem) \
    VSMC_RUNTIME_ASSERT((div > rem),                                         \
            ("**vsmc::SeedGenerator::modulo** "                              \
             "REMAINDER IS NOT SMALLER THAN THE DIVISOR"))

#define VSMC_RUNTIME_ASSERT_RNG_SEED_MAX(seed_max) \
    VSMC_RUNTIME_ASSERT((seed_max > 1),                                      \
            ("**vsmc::SeedGenerator::modulo** "                              \
             "THE MAXIMUM OF THE INTERNAL SEED IS NO LARGER THAN 1"))

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
template <typename ID>
class SeedGenerator
{
    public :

    typedef unsigned result_type;

    static SeedGenerator<ID> &instance ()
    {
        static SeedGenerator<ID> seed;

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
        seed_max_ = std::numeric_limits<result_type>::max
            VSMC_MACRO_NO_EXPANSION ();
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
    result_type seed_max_;
    result_type divisor_;
    result_type remainder_;

    SeedGenerator () :
        seed_(0), seed_max_(std::numeric_limits<result_type>::max
                VSMC_MACRO_NO_EXPANSION ()), divisor_(1), remainder_(0) {}

    SeedGenerator (const SeedGenerator<ID> &);
    SeedGenerator<ID> &operator= (const SeedGenerator<ID> &);
}; // class SeedGenerator

typedef VSMC_SEED_TYPE Seed;

} // namespace vsmc

#endif // VSMC_RNG_SEED_HPP
