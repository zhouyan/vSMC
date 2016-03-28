//============================================================================
// vSMC/include/vsmc/rng/discrete_distribution.hpp
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

#ifndef VSMC_RNG_DISCRETE_DISTRIBUTION_HPP
#define VSMC_RNG_DISCRETE_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/u01_distribution.hpp>

#define VSMC_RUNTIME_ASSERT_RNG_DISCRETE_DISTRIBUTION_POSITIVE(flag)          \
    VSMC_RUNTIME_ASSERT(                                                      \
        (flag), "**DiscreteDistribution** WEIGHTS ARE NOT NON-NEGATIVE")

namespace vsmc
{

/// \brief Draw a single sample given weights
/// \ingroup Distribution
template <typename IntType>
class DiscreteDistribution
{
    public:
    using result_type = IntType;
    using distribution_type = DiscreteDistribution<IntType>;

    class param_type
    {
        public:
        using result_type = IntType;
        using distribution_type = DiscreteDistribution<IntType>;

        param_type() {}

        template <typename InputIter>
        param_type(InputIter first, InputIter last) : probability_(first, last)
        {
            invariant();
        }

        param_type(std::initializer_list<double> weights)
            : probability_(weights.begin(), weights.end())
        {
            invariant();
        }

        template <typename UnaryOperation>
        param_type(std::size_t count, double xmin, double xmax,
            UnaryOperation unary_op)
        {
            probability_.reserve(count);
            double delta = (xmax - xmin) / static_cast<double>(count);
            xmin += 0.5 * delta;
            for (std::size_t i = 0; i != count; ++i)
                probability_.push_back(
                    unary_op(xmin + static_cast<double>(i) * delta));
            invariant();
        }

        Vector<double> probability() const { return probability_; }

        friend bool operator==(
            const param_type &param1, const param_type &param2)
        {
            if (param1.probability_.size() != param2.probability_.size())
                return false;

            for (std::size_t i = 0; i != param1.probability_.size(); ++i)
                if (!internal::is_equal(
                        param1.probability_[i], param2.probability_[i]))
                    return false;

            return true;
        }

        friend bool operator!=(
            const param_type &param1, const param_type &param2)
        {
            return !(param1 == param2);
        }

        template <typename CharT, typename Traits>
        friend std::basic_ostream<CharT, Traits> &operator<<(
            std::basic_ostream<CharT, Traits> &os, const param_type &param)
        {
            if (!os.good())
                return os;

            os << param.probability_;

            return os;
        }

        template <typename CharT, typename Traits>
        friend std::basic_istream<CharT, Traits> &operator>>(
            std::basic_istream<CharT, Traits> &is, param_type &param)
        {
            if (!is.good())
                return is;

            Vector<double> probability;
            is >> std::ws >> probability;

            if (is.good()) {
                double sum = 0;
                if (is_positive(probability, sum)) {
                    mul(probability.size(), 1 / sum, probability.data(),
                        probability.data());
                    param.probability_ = std::move(probability);
                } else {
                    is.setstate(std::ios_base::failbit);
                }
            }

            return is;
        }

        private:
        Vector<double> probability_;

        friend distribution_type;

        void invariant()
        {
            if (probability_.size() == 0)
                return;

            double sum = 0;
#ifndef NDEBUG
            bool flag = is_positive(probability_, sum);
            VSMC_RUNTIME_ASSERT_RNG_DISCRETE_DISTRIBUTION_POSITIVE(flag);
#else
            is_positive(probability_, sum);
#endif
            mul(probability_.size(), 1 / sum, probability_.data(),
                probability_.data());
        }

        void reset() {}

        static bool is_positive(const Vector<double> &probability, double &sum)
        {
            sum = 0;
            bool flag = true;
            for (std::size_t i = 0; i != probability.size(); ++i) {
                sum += probability[i];
                if (probability[i] < 0)
                    flag = false;
            }

            return flag && sum > 0;
        }
    }; // class param_type

    DiscreteDistribution() {}

    template <typename InputIter>
    DiscreteDistribution(InputIter first, InputIter last) : param_(first, last)
    {
    }

    DiscreteDistribution(std::initializer_list<double> weights)
        : param_(weights)
    {
    }

    template <typename UnaryOperation>
    DiscreteDistribution(
        std::size_t count, double xmin, double xmax, UnaryOperation &&unary_op)
        : param_type(count, xmin, xmax, std::forward<UnaryOperation>(unary_op))
    {
    }

    explicit DiscreteDistribution(const param_type &param) : param_(param) {}

    explicit DiscreteDistribution(param_type &&param)
        : param_(std::move(param))
    {
    }

    result_type min() const { return 0; }

    result_type max() const
    {
        return param_.size() == 0 ? 0 : param_.size() - 1;
    }

    Vector<double> probability() const { return param_.probability_; }

    template <typename RNGType>
    result_type operator()(RNGType &rng) const
    {
        return operator()(
            rng, param_.probability_.begin(), param_.probability_.end(), true);
    }

    /// \brief Draw sample with external probabilities
    ///
    /// \param rng A uniform random number generator
    /// \param first The first iterator of the weights sequence.
    /// \param last The one past the end iterator of the weights sequence.
    /// \param normalized If the weights are already normalized
    ///
    /// \details
    /// Given weights \f$(W_1,\dots,W_N)\f$, it is possible to draw the index
    /// \f$i\f$ using the `std::discrete_distribuiton` template. However, there
    /// are two drawbacks with this approach. First, if the weights are already
    /// normalized, this template does uncessary extra work to normalized the
    /// weights. Second, whenever the weights change, a new distribution need
    /// to be constructed (the `param_type` of the distribution is
    /// implementation defined and cannot be used to write portable code),
    /// which will lead to uncessary dynamic memory allocation. This function
    /// does not use dynamic memory and improve performance for normalized
    /// weights.
    template <typename RNGType, typename InputIter>
    result_type operator()(RNGType &rng, InputIter first, InputIter last,
        bool normalized = false) const
    {
        using value_type =
            typename std::iterator_traits<InputIter>::value_type;

        U01Distribution<value_type> u01;
        value_type u = u01(rng);

        if (!normalized) {
            value_type mulw =
                1 / std::accumulate(first, last, static_cast<value_type>(0));
            value_type accw = 0;
            result_type index = 0;
            while (first != last) {
                accw += *first * mulw;
                if (u <= accw)
                    return index;
                ++first;
                ++index;
            }

            return index - 1;
        }

        value_type accw = 0;
        result_type index = 0;
        while (first != last) {
            accw += *first;
            if (u <= accw)
                return index;
            ++first;
            ++index;
        }

        return index - 1;
    }

    friend bool operator==(
        const distribution_type &dist1, const distribution_type &dist2)
    {
        return dist1.param_ == dist2.param_;
    }

    friend bool operator!=(
        const distribution_type &dist1, const distribution_type &dist2)
    {
        return !(dist1 == dist2);
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os, const distribution_type &dist)
    {
        os << dist.param_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is, distribution_type &dist)
    {
        is >> std::ws >> dist.param_;
        if (is.good())
            dist.reset();

        return is;
    }

    private:
    param_type param_;
}; // class DiscreteDistribution

} // namespace vsmc

#endif // VSMC_RNG_DISCRETE_DISTRIBUTION_HPP
