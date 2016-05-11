//============================================================================
// vSMC/include/vsmc/core/weight.hpp
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

#ifndef VSMC_CORE_WEIGHT_HPP
#define VSMC_CORE_WEIGHT_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/rng/discrete_distribution.hpp>

namespace vsmc
{

/// \brief Weight class
/// \ingroup Core
class Weight
{
    public:
    using size_type = std::size_t;

    explicit Weight(size_type N = 0) : ess_(0), data_(N) { set_equal(); }

    /// \brief Size of this Weight object
    size_type size() const { return data_.size(); }

    /// \brief Resize the Weight object
    ///
    /// \details
    /// After resizing, if the size changed, equal weights are set
    void resize(size_type N)
    {
        if (N == size())
            return;

        data_.resize(N);
        set_equal();
    }

    /// \brief Reserve space
    void reserve(size_type N) { data_.reserve(N); }

    /// \brief Shrink to fit
    void shrink_to_fit() { data_.shrink_to_fit(); }

    /// \brief Return the ESS of the particle system
    double ess() const { return ess_; }

    /// \brief Pointer to data of the normalized weight
    const double *data() const { return data_.data(); }

    /// \brief Read all normalized weights to an output iterator
    template <typename OutputIter>
    OutputIter read_weight(OutputIter first) const
    {
        return std::copy(data_.begin(), data_.end(), first);
    }

    /// \brief Read all normalized weights to a random access iterator
    template <typename RandomIter>
    RandomIter read_weight(RandomIter first, int stride) const
    {
        for (std::size_t i = 0; i != size(); ++i, first += stride)
            *first = data_[i];

        return first;
    }

    /// \brief Set \f$W_i = 1/N\f$
    void set_equal()
    {
        std::fill(data_.begin(), data_.end(), 1.0 / size());
        ess_ = static_cast<double>(size());
    }

    /// \brief Set \f$W_i \propto w_i\f$
    template <typename InputIter>
    void set(InputIter first)
    {
        std::copy_n(first, size(), data_.begin());
        normalize(false);
    }

    /// \brief Set \f$W_i \propto w_i\f$
    template <typename RandomIter>
    void set(RandomIter first, int stride)
    {
        if (stride == 1) {
            set(first);
            return;
        }

        for (std::size_t i = 0; i != size(); ++i, first += stride)
            data_[i] = *first;
        normalize(false);
    }

    /// \brief Set \f$W_i \propto W_i w_i\f$
    template <typename InputIter>
    void mul(InputIter first)
    {
        for (std::size_t i = 0; i != size(); ++i, ++first)
            data_[i] *= *first;
        normalize(false);
    }

    /// \brief Set \f$W_i \propto W_i w_i\f$
    void mul(const double *first)
    {
        ::vsmc::mul(size(), first, data_.data(), data_.data());
        normalize(false);
    }

    /// \brief Set \f$W_i \propto W_i w_i\f$
    void mul(double *first) { mul(const_cast<const double *>(first)); }

    /// \brief Set \f$W_i \propto W_i w_i\f$
    template <typename RandomIter>
    void mul(RandomIter first, int stride)
    {
        if (stride == 1) {
            mul(first);
            return;
        }

        for (std::size_t i = 0; i != size(); ++i, first += stride)
            data_[i] *= *first;
        normalize(false);
    }

    /// \brief Set \f$\log W_i = v_i + \mathrm{const.}\f$
    template <typename InputIter>
    void set_log(InputIter first)
    {
        std::copy_n(first, size(), data_.begin());
        normalize(true);
    }

    /// \brief Set \f$\log W_i = v_i + \mathrm{const.}\f$
    template <typename RandomIter>
    void set_log(RandomIter first, int stride)
    {
        if (stride == 1) {
            set_log(first);
            return;
        }

        for (std::size_t i = 0; i != size(); ++i, first += stride)
            data_[i] = *first;
        normalize(true);
    }

    /// \brief Set \f$\log W_i = \log W_i + v_i + \mathrm{const.}\f$
    template <typename InputIter>
    void add_log(InputIter first)
    {
        log(size(), data_.data(), data_.data());
        for (std::size_t i = 0; i != size(); ++i)
            data_[i] += *first;
        normalize(true);
    }

    /// \brief Set \f$\log W_i = \log W_i + v_i + \mathrm{const.}\f$
    void add_log(const double *first)
    {
        log(size(), data_.data(), data_.data());
        add(size(), first, data_.data(), data_.data());
        normalize(true);
    }

    /// \brief Set \f$\log W_i = \log W_i + v_i + \mathrm{const.}\f$
    void add_log(double *first) { add_log(const_cast<const double *>(first)); }

    /// \brief Set \f$\log W_i = \log W_i + v_i + \mathrm{const.}\f$
    template <typename RandomIter>
    void add_log(RandomIter first, int stride)
    {
        if (stride == 1) {
            add_log(first);
            return;
        }

        log(size(), data_.data(), data_.data());
        for (std::size_t i = 0; i != size(); ++i, first += stride)
            data_[i] += *first;
        normalize(true);
    }

    /// \brief Draw integer index in the range \f$[0, N)\f$ according to the
    /// weights
    template <typename RNGType>
    size_type draw(RNGType &rng) const
    {
        return draw_(rng, data_.begin(), data_.end(), true);
    }

    private:
    double ess_;
    Vector<double> data_;
    DiscreteDistribution<size_type> draw_;

    template <typename InputIter>
    double max_element(std::size_t n, InputIter first)
    {
        using value_type =
            typename std::iterator_traits<InputIter>::value_type;

        value_type v = -std::numeric_limits<value_type>::infinity();
        for (std::size_t i = 0; i != n; ++i, ++first)
            if (v < *first)
                v = *first;

        return static_cast<double>(v);
    }

    void normalize(bool use_log)
    {
        double *w = data_.data();
        double accw = 0;
        double essw = 0;
        const double lmax = use_log ? max_element(size(), w) : 0;
        const std::size_t k = internal::BufferSize<double>::value;
        const std::size_t m = size() / k;
        const std::size_t l = size() % k;
        for (std::size_t i = 0; i != m; ++i, w += k)
            normalize(k, w, accw, essw, use_log, lmax);
        normalize(l, w, accw, essw, use_log, lmax);
        ::vsmc::mul(size(), 1 / accw, data_.data(), data_.data());
        ess_ = accw * accw / essw;
    }

    void normalize(std::size_t n, double *w, double &accw, double &essw,
        bool use_log, double lmax)
    {
        if (use_log) {
            sub(n, w, lmax, w);
            exp(n, w, w);
        }
        accw = std::accumulate(w, w + n, accw);
        essw +=
            internal::cblas_ddot(static_cast<VSMC_BLAS_INT>(n), w, 1, w, 1);
    }
}; // class Weight

/// \brief An empty weight set class
/// \ingroup Core
///
/// \details
/// This class provides all the interfaces of Weight, while they do nothing at
/// all and the class cost no memory space. This is primarily to be used in
/// algorithms where weights are irrelevant or managed outside the sampler for
/// any reason. Any attempt of using methods of this class will not result in
/// compile time or runtime errors.
class WeightNull
{
    public:
    using size_type = std::size_t;

    explicit WeightNull(size_type) {}

    size_type size() const { return 0; }

    void resize(size_type) {}

    void reserve(size_type) {}

    void shrink_to_fit() {}

    double ess() const { return std::numeric_limits<double>::quiet_NaN(); }

    const double *data() const { return nullptr; }

    template <typename OutputIter>
    void read_weight(OutputIter) const
    {
    }

    template <typename RandomIter>
    void read_weight(RandomIter, int) const
    {
    }

    void set_equal() {}

    template <typename InputIter>
    void set(InputIter)
    {
    }

    template <typename RandomIter>
    void set(RandomIter, int)
    {
    }

    template <typename InputIter>
    void mul(InputIter)
    {
    }

    template <typename RandomIter>
    void mul(RandomIter, int)
    {
    }

    template <typename InputIter>
    void set_log(InputIter)
    {
    }

    template <typename RandomIter>
    void set_log(RandomIter, int)
    {
    }

    template <typename InputIter>
    void add_log(InputIter)
    {
    }

    template <typename RandomIter>
    void add_log(RandomIter, int)
    {
    }

    template <typename RNGType>
    size_type draw(RNGType &) const
    {
        return 0;
    }
}; // class WeightNull

/// \brief Particle::weight_type trait
/// \ingroup Traits
VSMC_DEFINE_TYPE_DISPATCH_TRAIT(WeightType, weight_type, Weight)

} // namespace vsmc

#endif // VSMC_CORE_WEIGHT_HPP
