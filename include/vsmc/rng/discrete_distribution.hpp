//============================================================================
// vSMC/include/vsmc/rng/discrete_distribution.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013,2014, Yan Zhou
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

namespace vsmc {

/// \brief Draw a single sample given weights
/// \ingroup Distribution
template <typename IntType = int>
class DiscreteDistribution : public cxx11::discrete_distribution<IntType>
{
    public :

    typedef cxx11::discrete_distribution<IntType> base_distribution_type;
    typedef typename base_distribution_type::result_type result_type;
    typedef typename base_distribution_type::param_type param_type;

    DiscreteDistribution () {}

    template <typename InputIter>
    DiscreteDistribution (InputIter first, InputIter last) :
        base_distribution_type(first, last) {}

#if VSMC_HAS_CXX11LIB_INITIALIZER_LIST
    DiscreteDistribution (std::initializer_list<double> weights) :
        base_distribution_type(weights.begin(), weights.end()) {}
#endif

    template <typename UnaryOperation>
    DiscreteDistribution (std::size_t count, double xmin, double xmax,
            UnaryOperation unary_op) :
        base_distribution_type(count, xmin, xmax, unary_op) {}

    explicit DiscreteDistribution (const param_type &param) :
        base_distribution_type(param) {}

    ///
    /// \param eng A uniform random number generator
    /// \param first The first iterator of the weights sequence.
    /// \param last The one past the end iterator of the weights sequence.
    /// \param normalized If the weights are already normalized
    ///
    /// \details
    /// Given weights \f$(W_1,\dots,\W_N)\f$, it is possible to draw the index
    /// \f$i\f$ using the `std::discrete_distribuiton` template. However, there
    /// are two drawbacks with this approach. First, if the weightsa are
    /// already normalized, this template does uncessary extra work to
    /// normalized the weights. Second, whenever the weights change, a new
    /// distribution need to be constructed (the `param_type` of the
    /// distribution is implementation defined and cannot be used to write
    /// portable code), which will lead to uncessary
    /// dynamic memory allocation.
    ///
    /// This function requires the *normalized* weights, specified using the
    /// iterators `first` and `last`, and return the index (counting from zero)
    /// of the random draw. No dynamic memory allocaiton will be invovled by
    /// calling this function.
    template <typename URNG, typename InputIter>
    result_type operator() (URNG &eng, InputIter first, InputIter last,
            bool normalized = false)
    {
        typedef typename  std::iterator_traits<InputIter>::value_type
            value_type;

        cxx11::uniform_real_distribution<value_type> runif(0, 1);
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
}; // class DiscreteDistribution

} // namespace vsmc

#endif // VSMC_RNG_DISCRETE_DISTRIBUTION_HPP
