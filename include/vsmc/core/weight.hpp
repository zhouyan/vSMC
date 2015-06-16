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
    return 1 / math::dot(N, first, 1, first, 1);
}

/// \brief Normalize weights such that the summation is one
/// \ingroup Core
static void weight_normalize(std::size_t N, double *first)
{
    math::scal(N, 1 / math::asum(N, first, 1), first, 1);
}

/// \brief Normalize logarithm weights such that the maximum is zero
/// \ingroup Core
static void weight_normalize_log(std::size_t N, double *first)
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

    explicit Weight(size_type N) : size_(N), ess_(0), data_(N) {}

    Weight(const Weight &) = default;
    Weight &operator=(const Weight &) = default;
    Weight(Weight &&) = default;
    Weight &operator=(Weight &&) = default;

    virtual ~Weight() {}

    /// \brief The number of particles
    size_type size() const { return size_; }

    /// \brief Size of the weight set for the purpose of resampling
    virtual size_type resample_size() const { return size(); }

    /// \brief ESS of the current weights
    double ess() const { return ess_; }

    /// \brief Read only access to the raw data of weight
    const double *data() const { return data_.data(); }

    /// \brief Read only access to the resampling weights
    virtual const double *resample_data() const { return data_.data(); }

    /// \brief Read normalized weights through an output iterator
    template <typename OutputIter>
    void read_weight(OutputIter first) const
    {
        std::copy(data_.begin(), data_.end(), first);
    }

    /// \brief Read normalized weights through a random access iterator with
    /// (possible non-uniform stride)
    template <typename RandomIter>
    void read_weight(RandomIter first, int stride) const
    {
        for (size_type i = 0; i != size_; ++i, first += stride)
            *first = data_[i];
    }

    /// \brief Read normalized weights through a pointer for the purpose of
    /// resampling
    virtual void read_resample_weight(double *first) const
    {
        read_weight(first);
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS
    /// such that each particle has a equal weight
    void set_equal()
    {
        std::fill(data_.begin(), data_.end(), 1.0);
        post_set();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) weights directly through an input
    /// iterator
    template <typename InputIter>
    void set(InputIter first)
    {
        std::copy_n(first, size_, data_.begin());
        post_set();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) weights directly through a random
    /// access iterator with (possible non-uniform) stride
    template <typename RandomIter>
    void set(RandomIter first, int stride)
    {
        for (size_type i = 0; i != size_; ++i, first += stride)
            data_[i] = *first;
        post_set();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// multiply the normalized weight with (possible unnormalized)
    /// incremental
    /// weights through an input iterator
    template <typename InputIter>
    void mul(InputIter first)
    {
        for (size_type i = 0; i != size_; ++i, ++first)
            data_[i] *= *first;
        post_set();
    }

    void mul(const double *first)
    {
        math::vMul(size_, data_.data(), first, data_.data());
        post_set();
    }

    void mul(double *first) { mul(const_cast<const double *>(first)); }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// multiply the normalized weight with (possible unnormalized)
    /// incremental
    /// weights through a random access iterator with (possible non-uniform)
    /// stride
    template <typename RandomIter>
    void mul(RandomIter first, int stride)
    {
        for (size_type i = 0; i != size_; ++i, first += stride)
            data_[i] *= *first;
        post_set();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) logarithm weights directly
    /// through
    /// an input iterator
    template <typename InputIter>
    void set_log(InputIter first)
    {
        std::copy_n(first, size_, data_.begin());
        post_set_log();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) logarithm weights directly
    /// through
    /// a random access iterator with (possible non-uniform) stride
    template <typename RandomIter>
    void set_log(RandomIter first, int stride)
    {
        for (size_type i = 0; i != size_; ++i, first += stride)
            data_[i] = *first;
        post_set_log();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// adding to the unnormalized logarithm weights with (possible
    /// unormalized) logarithm incremental weights through an input iterator
    template <typename InputIter>
    void add_log(InputIter first)
    {
        math::vLn(size_, data_.data(), data_.data());
        for (size_type i = 0; i != size_; ++i, ++first)
            data_[i] += *first;
        post_set_log();
    }

    void add_log(const double *first)
    {
        math::vLn(size_, data_.data(), data_.data());
        math::vAdd(size_, data_.data(), first, data_.data());
        post_set_log();
    }

    void add_log(double *first) { add_log(const_cast<const double *>(first)); }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// adding to the unnormalized logarithm weights with (possible
    /// unormalized) logarithm incremental weights through a ranodm access
    /// iterator with (possible non-uniform) stride
    template <typename RandomIter>
    void add_log(RandomIter first, int stride)
    {
        math::vLn(size_, data_.data(), data_.data());
        for (size_type i = 0; i != size_; ++i, first += stride)
            data_[i] += *first;
        post_set_log();
    }

    /// \brief Draw a sample according to the weights
    template <typename URNG>
    size_type draw(URNG &eng) const
    {
        return draw_(eng, data_.begin(), data_.end(), true);
    }

    protected:
    /// \brief Write access to the raw data of weight
    double *mutable_data() { return data_.data(); }

    private:
    size_type size_;
    double ess_;
    Vector<double> data_;
    DiscreteDistribution<size_type> draw_;

    void post_set()
    {
        normalize();
        ess_ = get_ess();
    }

    void post_set_log()
    {
        normalize_log();
        math::vExp(size_, data_.data(), data_.data());
        post_set();
    }

    virtual double get_ess() const { return weight_ess(size_, data_.data()); }

    virtual void normalize() { weight_normalize(size_, data_.data()); }

    virtual void normalize_log() { weight_normalize_log(size_, data_.data()); }
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

    double ess() const { return std::numeric_limits<double>::infinity(); }

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
}; // class WeightEmtpy

/// \brief Particle::weight_type trait
/// \ingroup Traits
VSMC_DEFINE_TYPE_DISPATCH_TRAIT(WeightType, weight_type, Weight)

} // namespace vsmc

#endif // VSMC_CORE_WEIGHT_HPP
