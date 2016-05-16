//============================================================================
// vSMC/include/vsmc/rng/seed.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2016, Yan Zhou
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
#include <vsmc/rng/counter.hpp>

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
/// The sequence of seeds are belongs to the equivalent class \f$s \mod D
/// \equiv R\f$ where \f$D > 0\f$, \f$R \ge 0\f$. The defaults are \f$1\f$ and
/// \f$0\f$ respectively. Each time `get()` is called, a new seed is returned.
///
/// The method `operator()(RNGType &rng)` is equivalent to
/// `rng.seed(static_cast<typename RNGType::result_type>(get()))`.
template <typename ID, typename ResultType = VSMC_SEED_RESULT_TYPE>
class SeedGenerator
{
    static_assert(std::is_unsigned<ResultType>::value,
        "**SeedGenerator** USED WITH ResultType OTHER THAN UNSIGEND INTEGER "
        "TYPES");

    public:
    using result_type = ResultType;

    SeedGenerator(const SeedGenerator<ID, ResultType> &) = delete;

    SeedGenerator<ID, ResultType> &operator=(
        const SeedGenerator<ID, ResultType> &) = delete;

    static SeedGenerator<ID, ResultType> &instance()
    {
        static SeedGenerator<ID, ResultType> seed;

        return seed;
    }

    /// \brief Seed a single RNG
    template <typename RNGType>
    void operator()(RNGType &rng)
    {
        rng.seed(static_cast<typename RNGType::result_type>(get()));
    }

    /// \brief Seed a sequence of RNGs
    template <typename OutputIter>
    OutputIter operator()(std::size_t n, OutputIter first)
    {
        using RNGType = typename std::iterator_traits<OutputIter>::value_type;

        for (std::size_t i = 0; i != n; ++i, ++first)
            first->seed(static_cast<typename RNGType::result_type>(get()));

        return first;
    }

    /// \brief Set the seed to `s % max() * divisor() + remainder()`
    void set(result_type s) { seed_ = s % max_; }

    /// \brief Get a seed
    result_type get()
    {
        ++seed_;
        if (seed_ >= max_)
            seed_ = 1;

        return seed_ * divisor_ + remainder_;
    }

    /// \brief The maximum of the seed
    result_type max() const { return max_; }

    /// \brief The divisor of the output seed
    result_type divisor() const { return divisor_; }

    /// \brief The remainder of the output seed
    result_type remainder() const { return remainder_; }

    /// \brief Set the divisor and the remainder
    void modulo(result_type divisor, result_type remainder)
    {
        runtime_assert(divisor > remainder, "**SeedGenerator::modulo** the "
                                            "remainder is not smaller than "
                                            "the divisor");

        result_type maxs = std::numeric_limits<result_type>::max() / divisor;
        runtime_assert(maxs > 1, "**SeedGenerator::modulo** the maximum of "
                                 "the internal seed will be no larger than 1");

        divisor_ = divisor;
        remainder_ = remainder;
        max_ = std::numeric_limits<result_type>::max() / divisor;

        set(seed_);
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os,
        const SeedGenerator<ID, ResultType> &sg)
    {
        if (!os)
            return os;

        os << sg.seed_ << ' ';
        os << sg.max_ << ' ';
        os << sg.divisor_ << ' ';
        os << sg.remainder_ << ' ';

        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is,
        SeedGenerator<ID, ResultType> &sg)
    {
        if (!is)
            return is;

        result_type seed;
        result_type max;
        result_type divisor;
        result_type remainder;
        is >> std::ws >> seed;
        is >> std::ws >> max;
        is >> std::ws >> divisor;
        is >> std::ws >> remainder;

        if (is) {
            sg.seed_ = seed;
            sg.max_ = max;
            sg.divisor_ = divisor;
            sg.remainder_ = remainder;
        }

        return is;
    }

    private:
    std::atomic<result_type> seed_;
    result_type max_;
    result_type divisor_;
    result_type remainder_;

    SeedGenerator() : seed_(0), max_(0), divisor_(1), remainder_(0)
    {
        modulo(divisor_, remainder_);
    }
}; // class SeedGenerator

/// \brief The default Seed type
/// \ingroup RNG
using Seed = SeedGenerator<NullType>;

} // namespace vsmc

#endif // VSMC_RNG_SEED_HPP
