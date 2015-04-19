//============================================================================
// vSMC/include/vsmc/rng/discrete_distribution.hpp
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

#ifndef VSMC_RNG_DISCRETE_DISTRIBUTION_HPP
#define VSMC_RNG_DISCRETE_DISTRIBUTION_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/math/cblas.hpp>

#define VSMC_RUNTIME_ASSERT_RNG_DISCRETE_DISTRIBUTION_POSITIVE(param) \
    VSMC_RUNTIME_ASSERT(is_positive(param),                                  \
            ("**DiscreteDistribution** WEIGHTS ARE NOT NON-NEGATIVE"));

namespace vsmc {

/// \brief Draw a single sample given weights
/// \ingroup Distribution
template <typename IntType = int>
class DiscreteDistribution
{
    public :

    typedef IntType result_type;
    typedef std::vector<double> param_type;

    DiscreteDistribution () {}

    template <typename InputIter>
    DiscreteDistribution (InputIter first, InputIter last) :
        param_(first, last)
    {
        VSMC_RUNTIME_ASSERT_RNG_DISCRETE_DISTRIBUTION_POSITIVE(param_);
        normalize();
    }

    DiscreteDistribution (std::initializer_list<double> weights) :
        param_(weights.begin(), weights.end())
    {
        VSMC_RUNTIME_ASSERT_RNG_DISCRETE_DISTRIBUTION_POSITIVE(param_);
        normalize();
    }

    template <typename UnaryOperation>
    DiscreteDistribution (std::size_t count, double xmin, double xmax,
            UnaryOperation unary_op)
    {
        param_.reserve(count);
        double delta = (xmax - xmin) / static_cast<double>(count);
        xmin += 0.5 * delta;
        for (std::size_t i = 0; i != count; ++i)
            param_.push_back(unary_op(xmin + static_cast<double>(i) * delta));
        VSMC_RUNTIME_ASSERT_RNG_DISCRETE_DISTRIBUTION_POSITIVE(param_);
        normalize();
    }

    explicit DiscreteDistribution (const param_type &param)
    {
        VSMC_RUNTIME_ASSERT_RNG_DISCRETE_DISTRIBUTION_POSITIVE(param);
        param_ = param;
        normalize();
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    explicit DiscreteDistribution (param_type &&param)
    {
        VSMC_RUNTIME_ASSERT_RNG_DISCRETE_DISTRIBUTION_POSITIVE(param);
        param_ = std::move(param);
        normalize();
    }
#endif

    param_type param () const {return param_;}

    void param (const param_type &param)
    {
        VSMC_RUNTIME_ASSERT_RNG_DISCRETE_DISTRIBUTION_POSITIVE(param);
        param_ = param;
        normalize();
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    void param (param_type &&param)
    {
        VSMC_RUNTIME_ASSERT_RNG_DISCRETE_DISTRIBUTION_POSITIVE(param);
        param_ = std::move(param);
        normalize();
    }
#endif

    void reset () {}

    result_type min VSMC_MNE () const {return 0;}
    result_type max VSMC_MNE () const
    {return param_.size() == 0 ? 0 : param_.size() - 1;}

    std::vector<double> probability () const {return param_;}

    template <typename URNG>
    result_type operator() (URNG &eng) const
    {return operator()(eng, param_.begin(), param_.end(), true);}

    /// \brief Draw sample with external probabilities
    ///
    /// \param eng A uniform random number generator
    /// \param first The first iterator of the weights sequence.
    /// \param last The one past the end iterator of the weights sequence.
    /// \param normalized If the weights are already normalized
    ///
    /// \details
    /// Given weights \f$(W_1,\dots,\W_N)\f$, it is possible to draw the index
    /// \f$i\f$ using the `std::discrete_distribuiton` template. However, there
    /// are two drawbacks with this approach. First, if the weights are
    /// already normalized, this template does uncessary extra work to
    /// normalized the weights. Second, whenever the weights change, a new
    /// distribution need to be constructed (the `param_type` of the
    /// distribution is implementation defined and cannot be used to write
    /// portable code), which will lead to uncessary
    /// dynamic memory allocation. This function does not use dynamic memory
    /// and improve performance for normalized weights.
    template <typename URNG, typename InputIter>
    result_type operator() (URNG &eng, InputIter first, InputIter last,
            bool normalized = false) const
    {
        typedef typename  std::iterator_traits<InputIter>::value_type
            value_type;

        std::uniform_real_distribution<value_type> runif(0, 1);
        value_type u = runif(eng);

        if (!normalized) {
            value_type mulw = 1 / std::accumulate(first, last,
                    static_cast<value_type>(0));
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

    friend inline bool operator== (
            const DiscreteDistribution<IntType> &rdisc1,
            const DiscreteDistribution<IntType> &rdisc2)
    {return rdisc1.param_ == rdisc2.param_;}

    friend inline bool operator!= (
            const DiscreteDistribution<IntType> &rdisc1,
            const DiscreteDistribution<IntType> &rdisc2)
    {return rdisc1.param_ != rdisc2.param_;}

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &operator<< (
            std::basic_ostream<CharT, Traits> &os,
            const DiscreteDistribution<IntType> &rdisc)
    {
        if (!os.good())
            return os;

        os << rdisc.param_.size() << ' ';

        if (rdisc.param_.size() == 0)
            return os;

        if (rdisc.param_.size() == 1) {
            os << rdisc.param_[0];
            return os;
        }

        for (std::size_t i = 0; i != rdisc.param_.size() - 1; ++i)
            os << rdisc.param_[i] << ' ';
        os << rdisc.param_.back();

        return os;
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &operator>> (
            std::basic_istream<CharT, Traits> &is,
            DiscreteDistribution<IntType> &rdisc)
    {
        if (!is.good())
            return is;

        std::size_t n;
        is >> std::ws >> n;

        std::vector<double> param(n);
        for (std::size_t i = 0; i != n; ++i)
            is >> std::ws >> param[i];

        if (is.good()) {
            if (rdisc.is_positive(param)) {
                std::swap(rdisc.param_, param);
                rdisc.normalize();
            } else {
                is.setstate(std::ios_base::failbit);
            }
        }

        return is;
    }

    private :

    param_type param_;

    void normalize ()
    {
        if (param_.size() == 0)
            return;

        double sumw = std::accumulate(param_.begin(), param_.end(), 0.0);
        math::scal(param_.size(), 1 / sumw, &param_[0]);
    }

    bool is_positive (const param_type &param)
    {
        for (std::size_t i = 0; i != param.size(); ++i)
            if (param[i] < 0)
                return false;

        if (param.size() == 0)
            return true;

        if (std::accumulate(param.begin(), param.end(), 0.0) <= 0)
            return false;

        return true;
    }
}; // class DiscreteDistribution

} // namespace vsmc

#endif // VSMC_RNG_DISCRETE_DISTRIBUTION_HPP
