//============================================================================
// vSMC/include/vsmc/rng/seed.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#ifndef VSMC_RNG_SEED_HPP
#define VSMC_RNG_SEED_HPP

#include <vsmc/rng/internal/common.hpp>

#define VSMC_STATIC_ASSERT_RNG_SEED_GENERATOR_RESULT_TYPE(T)                 \
    VSMC_STATIC_ASSERT(                                                      \
        (std::is_unsigned<T>::value),                                        \
        USE_SeedGenerator_WITH_A_RESULT_TYPE_NOT_AN_UNSIGNED_INTEGER)

#define VSMC_RUNTIME_ASSERT_RNG_SEED_GENERATOR_MODULO(div, rem)              \
    VSMC_RUNTIME_ASSERT((div > rem),                                         \
                        ("**SeedGenerator::modulo** "                        \
                         "REMAINDER IS NOT SMALLER THAN THE DIVISOR"))

#define VSMC_RUNTIME_WARNING_RNG_SEED_GENERATOR_MODULO(div, rem)             \
    VSMC_RUNTIME_WARNING((div == 1 && rem == 0),                             \
                         ("**SeedGenerator::modulo** "                       \
                          "COUNTER TYPE SEED DOES NOT SUPPORT MODULO"))

#define VSMC_RUNTIME_ASSERT_RNG_SEED_MAX(seed_max)                           \
    VSMC_RUNTIME_ASSERT(                                                     \
        (seed_max > 1),                                                      \
        ("**SeedGenerator::modulo** "                                        \
         "THE MAXIMUM OF THE INTERNAL SEED IS NO LARGER THAN 1"))

/// \brief Default result type of Seed
/// \ingroup Config
#ifndef VSMC_SEED_RESULT_TYPE
#define VSMC_SEED_RESULT_TYPE unsigned
#endif

namespace vsmc
{

/// \brief Seed generator
/// \ingroup RNG
///
/// \details
/// Let \f$S\f$ be the current internal seed, an integer in the range between
/// 1
/// and \f$S_{\mathrm{max}}\f$ where \f$S_{\mathrm{max}}\f$ is the maximum of
/// the internal seed. Each time `get()` is called, the internal is first
/// increased by 1. If it is already equal to \f$S_{\mathrm{max}}\f$, then it
/// is set to 1. The output seed is \f$S \times D + R\f$ where \f$D\f$ is the
/// divisor and \f$R\f$ is the remainder. In other words, the output seed
/// belongs to the equivalent class \f$S\ \mathrm{mod}\ D \equiv R\f$. The
/// divisor and the remainder can be set through the `modulo(div, rem)` member
/// function.
///
/// This property is useful when using programming models such as MPI where
/// one
/// want to have distinct seeds across nodes. Each process can then configure
/// the SeedGenerator using the `modulo` member such that \f$D\f$ is the
/// number
/// of total nodes and \f$R\f$ is the rank of each node, counting from zero.
///
/// For multithreading programs, the access to this member function shall be
/// protected by mutex, if it is called from multiple threads. Or one can use
/// distinct type `ID` to initialized distinct SeedGenerator instances.
template <typename ID, typename ResultType = VSMC_SEED_RESULT_TYPE>
class SeedGenerator
{
    public:
    typedef ResultType result_type;
    typedef ResultType skip_type;

    static SeedGenerator<ID, ResultType> &instance()
    {
        static SeedGenerator<ID, ResultType> seed;

        return seed;
    }

    /// \brief Get a new seed
    result_type get()
    {
        skip();
        return seed_ * divisor_ + remainder_;
    }

    /// \brief Set the internal seed
    ///
    /// \details
    /// If `seed` is larger than the maximum of the internal seed, than it
    /// will
    /// be round up to fit into the range. For example, say the range is from
    /// 1
    /// to 1000, and the new internal seed is 6061, it will be round up to 61.
    void set(result_type seed) { seed_ = seed % seed_max_; }

    /// \brief The current internal seed
    result_type seed() const { return seed_; }

    /// \brief The maximum of the internal seed integer
    result_type seed_max() const { return seed_max_; }

    /// \brief The divisor of the output seed
    skip_type divisor() const { return divisor_; }

    /// \brief The remainder of the output seed
    skip_type remainder() const { return remainder_; }

    /// \brief Set the divisor and the remainder
    void modulo(skip_type div, skip_type rem)
    {
        VSMC_RUNTIME_ASSERT_RNG_SEED_GENERATOR_MODULO(div, rem);

        divisor_ = div;
        remainder_ = rem;
        seed_max_ = static_cast<skip_type>(~static_cast<skip_type>(0));
        seed_max_ -= seed_max_ % divisor_;
        seed_max_ /= divisor_;

        VSMC_RUNTIME_ASSERT_RNG_SEED_MAX(seed_max_);

        set(seed_);
    }

    /// \brief Skip the internal seed by a given steps
    void skip(skip_type steps)
    {
        result_type incr = steps % seed_max_;
        result_type diff = seed_max_ - seed_;
        seed_ = incr <= diff ? (seed_ + incr) : (incr - diff);
    }

    /// \brief Skip the internal seed by 1 step
    void skip() { seed_ = seed_ == seed_max_ ? 1 : (seed_ + 1); }

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &
        operator<<(std::basic_ostream<CharT, Traits> &os,
                   const SeedGenerator<ID, ResultType> &sg)
    {
        if (!os.good())
            return os;

        os << sg.seed_ << ' ';
        os << sg.divisor_ << ' ';
        os << sg.remainder_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &
        operator>>(std::basic_istream<CharT, Traits> &is,
                   SeedGenerator<ID, ResultType> &sg)
    {
        if (!is.good())
            return is;

        result_type s;
        skip_type div;
        skip_type rem;
        is >> std::ws >> s;
        is >> std::ws >> div;
        is >> std::ws >> rem;

        if (is.good()) {
            sg.modulo(div, rem);
            sg.set(s);
        }

        return is;
    }

    private:
    result_type seed_;
    result_type seed_max_;
    skip_type divisor_;
    skip_type remainder_;

    SeedGenerator() : seed_(0), seed_max_(0), divisor_(1), remainder_(0)
    {
        VSMC_STATIC_ASSERT_RNG_SEED_GENERATOR_RESULT_TYPE(ResultType);

        modulo(divisor_, remainder_);
    }

    SeedGenerator(const SeedGenerator<ID, ResultType> &);

    SeedGenerator<ID, ResultType> &
        operator=(const SeedGenerator<ID, ResultType> &);
};  // class SeedGenerator

/// \brief Seed generator counters
/// \ingroup RNG
///
/// \details
/// When SeedGenerator is used with an counter, the `modulo` has no effect and
/// a warning will be generated if `div != 1` or `rem != 0`. If the program
/// need to be parallelized on distributed memory system, it shall be
/// sufficient to set the the last component of the counter to be unique to
/// each node. For example,
/// ~~~{.cpp}
/// #include <vsmc/rng/seed.hpp>
///
/// typedef SeedGeneartor<NullType, Array<uint32_t, 4> > Seed4x32;
/// Seed4x32 &seed = SeedType::instance();
/// boost::mpi::communicator world;
/// Seed4x32::result_type s;
/// s.fill(0);
/// s.back() = world.rank();
/// seed.set(s);
/// ~~~
template <typename ID, typename T, std::size_t K>
class SeedGenerator<ID, Array<T, K>>
{
    public:
    typedef Array<T, K> result_type;
    typedef T skip_type;

    static SeedGenerator<ID, Array<T, K>> &instance()
    {
        static SeedGenerator<ID, Array<T, K>> seed;

        return seed;
    }

    result_type get()
    {
        skip();
        return seed_;
    }

    void set(result_type seed) { seed_ = seed; }

    result_type seed() const { return seed_; }

    result_type seed_max() const { return seed_max_; }

    skip_type divisor() const { return divisor_; }

    skip_type remainder() const { return remainder_; }

    void modulo(skip_type div, skip_type rem)
    {
        VSMC_RUNTIME_WARNING_RNG_SEED_GENERATOR_MODULO(div, rem);

        divisor_ = div;
        remainder_ = rem;
        seed_max_.fill(static_cast<skip_type>(~static_cast<skip_type>(0)));

        set(seed_);
    }

    void skip(skip_type steps)
    {
        Counter<result_type>::increment(seed_, steps);
    }

    void skip() { Counter<result_type>::increment(seed_); }

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &
        operator<<(std::basic_ostream<CharT, Traits> &os,
                   const SeedGenerator<ID, Array<T, K>> &sg)
    {
        if (!os.good())
            return os;

        os << sg.seed_ << ' ';
        os << sg.divisor_ << ' ';
        os << sg.remainder_ << ' ';

        return os;
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &
        operator>>(std::basic_istream<CharT, Traits> &is,
                   SeedGenerator<ID, Array<T, K>> &sg)
    {
        if (!is.good())
            return is;

        result_type s;
        skip_type div;
        skip_type rem;
        is >> std::ws >> s;
        is >> std::ws >> div;
        is >> std::ws >> rem;

        if (is.good()) {
            sg.modulo(div, rem);
            sg.set(s);
        }

        return is;
    }

    private:
    result_type seed_;
    result_type seed_max_;
    skip_type divisor_;
    skip_type remainder_;

    SeedGenerator() : divisor_(1), remainder_(0)
    {
        VSMC_STATIC_ASSERT_RNG_SEED_GENERATOR_RESULT_TYPE(T);

        seed_.fill(0);
        seed_max_.fill(0);
        modulo(divisor_, remainder_);
    }

    SeedGenerator<ID, Array<T, K>>(const SeedGenerator<ID, Array<T, K>> &);

    SeedGenerator<ID, Array<T, K>> &
        operator=(const SeedGenerator<ID, Array<T, K>> &);
};  // class SeedGenerator

/// \brief The default Seed type
/// \ingroup RNG
typedef SeedGenerator<NullType, VSMC_SEED_RESULT_TYPE> Seed;

}  // namespace vsmc

#endif  // VSMC_RNG_SEED_HPP
