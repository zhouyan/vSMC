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

#define VSMC_RUNTIME_ASSERT_RNG_SEED_GENERATOR_MODULO(div, rem)               \
    VSMC_RUNTIME_ASSERT((div > rem),                                          \
        "**SeedGenerator::modulo** "                                          \
        "REMAINDER IS NOT SMALLER THAN THE DIVISOR")

#define VSMC_RUNTIME_ASSERT_RNG_SEED_MAX(seed_max)                            \
    VSMC_RUNTIME_ASSERT((seed_max > 1),                                       \
        "**SeedGenerator::modulo** "                                          \
        "THE MAXIMUM OF THE INTERNAL SEED IS NO LARGER THAN 1")

/// \brief Default result type of Seed
/// \ingroup Config
#ifndef VSMC_SEED_RESULT_TYPE
#define VSMC_SEED_RESULT_TYPE std::uint32_t
#endif

namespace vsmc
{

/// \brief Seed generator
/// \ingroup RNG
///
/// \tparam ID Different ID type create indepdent SeedGenerator instances
/// \tparam ResultType The unsigned integer type of the seeds
/// \tparam K The length of the key. A key is a `std::array<ResultType, K>`
/// type object
///
/// \details
/// The sequence of seeds are belongs to the equivalent class \f$s \mod D
/// \equiv R\f$ where \f$D > 0\f$, \f$R \ge 0\f$. The defaults are \f$1\f$ and
/// \f$0\f$ respectively. Each time `get()` is called, a new seed is returned.
///
/// The sequence of keys are of the form \f$\{s_1,s_2,\dots,s_{K - 1},s\}\f$.
/// The first \f$K - 1\f$ elements forms a counter. Each time `get_key()` is
/// called, the counter is incremented and a key that combine the counter and
/// the current seed is returned.
///
/// The method `operator()(RNGType &rng)` is equivalent to
/// `rng.seed(get_key())` if `RNGType::key_type` exists and it is the same type
/// as `key_type` of this class. Otherwise, it is equivalent to
/// `rng.seed(get())`. This method is thread-safe but slower than
/// `rng.seed(get())` or `rng.seed(get_key())`, which are not thread-safe.
///
/// The only thread-safe mutable method is `operator()`. Seeding RNGs from
/// multiple threads at the same time using this method is possible. Any other
/// mutable methods are not thread-safe. In particular, if this singleton is
/// going to be used in a multi-threaded environment, make sure `instance()` is
/// called at least once in a single threaded environment.
template <typename ID, typename ResultType = VSMC_SEED_RESULT_TYPE,
    std::size_t K = 4>
class SeedGenerator
{
    static_assert(std::is_unsigned<ResultType>::value,
        "**SeedGenerator** USED WITH ResultType OTHER THAN UNSIGEND INTEGER "
        "TYPES");

    public:
    using result_type = ResultType;
    using key_type = std::array<ResultType, K>;

    SeedGenerator(const SeedGenerator<ID, ResultType, K> &) = delete;

    SeedGenerator<ID, ResultType, K> &operator=(
        const SeedGenerator<ID, ResultType, K> &) = delete;

    static SeedGenerator<ID, ResultType, K> &instance()
    {
        static SeedGenerator<ID, ResultType, K> seed;

        return seed;
    }

    /// \brief Seeding an RNG
    template <typename RNGType>
    void operator()(RNGType &rng)
    {
        rng_seed(rng, std::is_same<key_type, internal::KeyType<RNGType>>());
    }

    /// \brief Set the seed
    void set(result_type seed)
    {
        seed_ = seed % seed_max_;
        std::fill(ctr_.begin(), ctr_.end(), 0);
    }

    /// \brief Set the seed and key
    void set_key(const key_type &key)
    {
        std::copy_n(key.begin(), ctr_.size(), ctr_.begin());
        if (key.size() != 0)
            seed_ = key.back() % seed_max_;
    }

    /// \brief Get a seed
    result_type get()
    {
        if (seed_ < seed_max_)
            ++seed_;
        else
            seed_ = 1;

        return seed_ * divisor_ + remainder_;
    }

    /// \brief Get a key
    key_type get_key()
    {
        key_type key;
        increment(ctr_);
        std::copy(ctr_.begin(), ctr_.end(), key.begin());
        if (key.size() > 0)
            key.back() = get();

        return key;
    }

    /// \brief The divisor of the output seed
    result_type divisor() const { return divisor_; }

    /// \brief The remainder of the output seed
    result_type remainder() const { return remainder_; }

    /// \brief Set the divisor and the remainder
    void modulo(result_type divisor, result_type remainder)
    {
        VSMC_RUNTIME_ASSERT_RNG_SEED_GENERATOR_MODULO(divisor, remainder);

        divisor_ = divisor;
        remainder_ = remainder;
        seed_max_ = std::numeric_limits<result_type>::max() / divisor;

        VSMC_RUNTIME_ASSERT_RNG_SEED_MAX(seed_max_);

        set(seed_);
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os,
        const SeedGenerator<ID, ResultType, K> &sg)
    {
        if (!os)
            return os;

        os << sg.seed_ << ' ';
        os << sg.seed_max_ << ' ';
        os << sg.divisor_ << ' ';
        os << sg.remainder_ << ' ';
        os << sg.ctr_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is,
        SeedGenerator<ID, ResultType, K> &sg)
    {
        if (!is)
            return is;

        result_type seed;
        result_type seed_max;
        result_type divisor;
        result_type remainder;
        ctr_type ctr;
        is >> std::ws >> seed;
        is >> std::ws >> seed_max;
        is >> std::ws >> divisor;
        is >> std::ws >> remainder;
        is >> std::ws >> ctr;

        if (static_cast<bool>(is)) {
            sg.seed_ = seed;
            sg.seed_max_ = seed_max;
            sg.divisor_ = divisor;
            sg.remainder_ = remainder;
            sg.ctr_ = ctr;
        }

        return is;
    }

    private:
    using ctr_type = std::array<ResultType, K == 0 ? 0 : K - 1>;

    result_type seed_;
    result_type seed_max_;
    result_type divisor_;
    result_type remainder_;
    ctr_type ctr_;
    std::mutex mtx_;

    SeedGenerator() : seed_(0), seed_max_(0), divisor_(1), remainder_(0)
    {
        modulo(divisor_, remainder_);
        std::fill(ctr_.begin(), ctr_.end(), 0);
    }

    template <typename RNGType>
    void rng_seed(RNGType &rng, std::true_type)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        rng.seed(get_key());
    }

    template <typename RNGType>
    void rng_seed(RNGType &rng, std::false_type)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        rng.seed(static_cast<typename RNGType::result_type>(get()));
    }
}; // class SeedGenerator

/// \brief The default Seed type
/// \ingroup RNG
using Seed = SeedGenerator<NullType>;

} // namespace vsmc

#endif // VSMC_RNG_SEED_HPP
