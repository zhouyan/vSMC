//============================================================================
// vSMC/include/vsmc/core/weight.hpp
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

#ifndef VSMC_CORE_WEIGHT_HPP
#define VSMC_CORE_WEIGHT_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/rng/discrete_distribution.hpp>

namespace vsmc
{

/// \brief Compute the ess given normalized weights
/// \ingroup Core
inline double weight_ess(std::size_t N, const double *first)
{
    return 1 / dot(N, first, 1, first, 1);
}

/// \brief Normalize weights such that the summation is one
/// \ingroup Core
inline void weight_normalize(std::size_t N, double *first)
{
    mul(N, 1 / std::accumulate(first, first + N, 0.0), first, first);
}

/// \brief Normalize logarithm weights such that the maximum is zero
/// \ingroup Core
inline void weight_normalize_log(std::size_t N, double *first)
{
    double wmax = *(std::max_element(first, first + N));
    for (std::size_t i = 0; i != N; ++i)
        first[i] -= wmax;
}

/// \brief Weight class
/// \ingroup Core
class Weight
{
    public:
    using size_type = std::size_t;

    explicit Weight(size_type N) : ess_(0), data_(N) {}

    size_type size() const { return data_.size(); }

    size_type resample_size() const { return size(); }

    double ess() const { return ess_; }

    const double *data() const { return data_.data(); }

    const double *resample_data() const { return data_.data(); }

    template <typename OutputIter>
    void read_weight(OutputIter first) const
    {
        std::copy(data_.begin(), data_.end(), first);
    }

    template <typename RandomIter>
    void read_weight(RandomIter first, int stride) const
    {
        for (size_type i = 0; i != size(); ++i, first += stride)
            *first = data_[i];
    }

    void read_resample_weight(double *first) const { read_weight(first); }

    void set_equal()
    {
        std::fill(data_.begin(), data_.end(), 1.0 / resample_size());
        post_set();
    }

    template <typename InputIter>
    void set(InputIter first)
    {
        std::copy_n(first, size(), data_.begin());
        post_set();
    }

    template <typename RandomIter>
    void set(RandomIter first, int stride)
    {
        for (size_type i = 0; i != size(); ++i, first += stride)
            data_[i] = *first;
        post_set();
    }

    template <typename InputIter>
    void mul(InputIter first)
    {
        for (size_type i = 0; i != size(); ++i, ++first)
            data_[i] *= *first;
        post_set();
    }

    void mul(const double *first)
    {
        ::vsmc::mul(size(), first, data_.data(), data_.data());
        post_set();
    }

    void mul(double *first) { mul(const_cast<const double *>(first)); }

    template <typename RandomIter>
    void mul(RandomIter first, int stride)
    {
        for (size_type i = 0; i != size(); ++i, first += stride)
            data_[i] *= *first;
        post_set();
    }

    template <typename InputIter>
    void set_log(InputIter first)
    {
        std::copy_n(first, size(), data_.begin());
        post_set_log();
    }

    template <typename RandomIter>
    void set_log(RandomIter first, int stride)
    {
        for (size_type i = 0; i != size(); ++i, first += stride)
            data_[i] = *first;
        post_set_log();
    }

    template <typename InputIter>
    void add_log(InputIter first)
    {
        log(size(), data_.data(), data_.data());
        for (size_type i = 0; i != size(); ++i, ++first)
            data_[i] += *first;
        post_set_log();
    }

    void add_log(const double *first)
    {
        log(size(), data_.data(), data_.data());
        add(size(), data_.data(), first, data_.data());
        post_set_log();
    }

    void add_log(double *first) { add_log(const_cast<const double *>(first)); }

    template <typename RandomIter>
    void add_log(RandomIter first, int stride)
    {
        log(size(), data_.data(), data_.data());
        for (size_type i = 0; i != size(); ++i, first += stride)
            data_[i] += *first;
        post_set_log();
    }

    template <typename URNG>
    size_type draw(URNG &eng) const
    {
        return draw_(eng, data_.begin(), data_.end(), true);
    }

    protected:
    double *mutable_data() { return data_.data(); }

    private:
    double ess_;
    Vector<double> data_;
    DiscreteDistribution<size_type> draw_;

    void post_set() { ess_ = normalize(false); }

    void post_set_log()
    {
        weight_normalize_log(size(), data_.data());
        ess_ = normalize(true);
    }

    double normalize(bool use_log)
    {
        double *w = data_.data();
        double accw = 0;
        const std::size_t k = 1000;
        const std::size_t m = size() / k;
        const std::size_t l = size() % k;
        for (std::size_t i = 0; i != m; ++i, w += k)
            normalize_eval(k, w, accw, use_log);
        normalize_eval(l, w, accw, use_log);
        ::vsmc::mul(size(), 1 / accw, data_.data(), data_.data());

        return 1 / dot(size(), data_.data(), 1, data_.data(), 1);
    }

    void normalize_eval(std::size_t n, double *w, double &accw, bool use_log)
    {
        if (use_log)
            exp(n, w, w);
        accw = std::accumulate(w, w + n, accw);
    }
}; // class Weight

/// \brief An empty weight set class
/// \ingroup Core
///
/// \details
/// This class provides all the interfaces of Weight, while they do nothing
/// at all and the class cost no memory usage. This is primarily to be used in
/// algorithms where weights are irrelevant. Any attempt of using member
/// functions of this class will not result in compile time or runtime errors,
/// but the results might not be what one will be expecting.
class WeightNull
{
    public:
    using size_type = std::size_t;

    explicit WeightNull(size_type) {}

    size_type size() const { return 0; }

    size_type resample_size() const { return 0; }

    double ess() const { return std::numeric_limits<double>::quiet_NaN(); }

    const double *resample_data() const { return nullptr; }

    const double *data() const { return nullptr; }

    template <typename OutputIter>
    void read_weight(OutputIter) const
    {
    }

    template <typename RandomIter>
    void read_weight(RandomIter, int) const
    {
    }

    void read_resample_weight(double *) const {}

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

    template <typename URNG>
    size_type draw(URNG &) const
    {
        return 0;
    }
}; // class WeightNull

/// \brief Particle::weight_type trait
/// \ingroup Traits
VSMC_DEFINE_TYPE_DISPATCH_TRAIT(WeightType, weight_type, Weight)

} // namespace vsmc

#endif // VSMC_CORE_WEIGHT_HPP
