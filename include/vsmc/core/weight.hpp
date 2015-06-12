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

/// \brief Weight class
/// \ingroup Core
class Weight
{
    public:
    using size_type = std::size_t;

    explicit Weight(size_type N)
        : size_(N), ess_(static_cast<double>(N)), data_(N)
    {
    }

    Weight(const Weight &) = default;
    Weight &operator=(const Weight &) = default;
    Weight(Weight &&) = default;
    Weight &operator=(Weight &&) = default;

    virtual ~Weight() {}

    /// \brief Set all weights to be equal, normalized and return the ESS
    ///
    /// \param N Number of particles in weight
    /// \param RN Number  of particles used for resampling
    /// \param first Weights to be equalized
    static double set_equal(std::size_t N, std::size_t RN, double *first)
    {
        double ess = static_cast<double>(RN);
        if (first != nullptr)
            std::fill_n(first, N, 1 / ess);

        return ess;
    }

    /// \brief Normalize weights such that the summation is one and return the
    /// ESS
    static double normalize(std::size_t N, double *first)
    {
        double coeff = 1 / math::asum(N, first, 1);
        math::scal(N, coeff, first, 1);

        return 1 / math::dot(N, first, 1, first, 1);
    }

    /// \brief Normalize logarithm weights such that the maximum is zero
    static void normalize_log(std::size_t N, double *first)
    {
        double wmax = *(std::max_element(first, first + N));
        for (std::size_t i = 0; i != N; ++i)
            first[i] -= wmax;
    }

    /// \brief The number of particles
    size_type size() const { return size_; }

    /// \brief ESS of the current weights
    double ess() const { return ess_; }

    /// \brief Compute ESS given (log) incremental weights
    template <typename InputIter>
    double ess(InputIter first, bool use_log) const
    {
        Vector<double> buffer(size_);
        std::copy_n(first, size_, buffer.begin());

        return compute_ess(buffer.data(), use_log);
    }

    /// \brief Compute ESS given (log) incremental weights
    template <typename RandomIter>
    double ess(RandomIter first, int stride, bool use_log) const
    {
        Vector<double> buffer(size_);
        for (size_type i = 0; i != size_; ++i, first += stride)
            buffer[i] = *first;

        return compute_ess(buffer.data(), use_log);
    }

    /// \brief Compute CESS given (log) incremental weights
    template <typename InputIter>
    double cess(InputIter first, bool use_log) const
    {
        Vector<double> buffer(size_);
        std::copy_n(first, size_, buffer.begin());

        return compute_cess(buffer.data(), use_log);
    }

    /// \brief Compute CESS given (log) incremental weights
    template <typename RandomIter>
    double cess(RandomIter first, int stride, bool use_log) const
    {
        Vector<double> buffer(size_);
        for (size_type i = 0; i != size_; ++i, first += stride)
            buffer[i] = *first;

        return compute_cess(buffer.data(), use_log);
    }

    /// \brief Size of the weight set for the purpose of resampling
    virtual size_type resample_size() const { return size(); }

    /// \brief Read normalized weights through a pointer for the purpose of
    /// resampling
    ///
    /// \details
    /// In this class, `resample_weight` and `weight` are identical. However,
    /// in derived classes they might not be. For example, in `WeightMPI`,
    /// the `resample_weight` give a vector of normalized weights on all nodes
    /// while `weight` give the weights on the calling node.
    virtual void read_resample_weight(double *first) const
    {
        read_weight(first);
    }

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

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS
    /// such that each particle has a equal weight
    void set_equal()
    {
        set_ess(set_equal(size_, resample_size(), data_.data()));
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

    /// \brief Read only access to the resampling weights
    virtual const double *resample_data() const { return data_.data(); }

    /// \brief Read only access to the raw data of weight
    const double *data() const { return data_.data(); }

    protected:
    /// \brief Set the value of ESS;
    void set_ess(double e) { ess_ = e; }

    /// \brief Write access to the raw data of weight
    double *mutable_data() { return data_.data(); }

    /// \brief Normalize weights such that the summation is one
    virtual void normalize() { set_ess(normalize(size_, data_.data())); }

    /// \brief Normalize logarithm weights such that the maximum is zero
    virtual void normalize_log() { normalize_log(size_, data_.data()); }

    /// \brief Compute ESS given (logarithm) unormalzied incremental weights
    virtual double compute_ess(const double *first, bool use_log) const
    {
        Vector<double> buffer(size_);
        if (use_log) {
            math::vLn(size_, data_.data(), buffer.data());
            math::vAdd(size_, buffer.data(), first, buffer.data());
            double wmax = *(std::max_element(buffer.begin(), buffer.end()));
            for (size_type i = 0; i != size_; ++i)
                buffer[i] -= wmax;
            math::vExp(size_, buffer.data(), buffer.data());
        } else {
            math::vMul(size_, data_.data(), first, buffer.data());
        }

        double coeff = math::asum(size_, buffer.data(), 1);
        math::scal(size_, 1 / coeff, buffer.data(), 1);

        return 1 / math::dot(size_, buffer.data(), 1, buffer.data(), 1);
    }

    /// \brief Compute CESS given (logarithm) unormalized incremental weights
    virtual double compute_cess(const double *first, bool use_log) const
    {
        Vector<double> buffer;
        if (use_log) {
            buffer.resize(size_);
            math::vExp(size_, first, buffer.data());
        }
        const double *const bptr = use_log ? buffer.data() : first;

        double above = 0;
        double below = 0;
        for (size_type i = 0; i != size_; ++i) {
            double wb = data_[i] * bptr[i];
            above += wb;
            below += wb * bptr[i];
        }

        return above * above / below;
    }

    private:
    size_type size_;
    double ess_;
    Vector<double> data_;
    DiscreteDistribution<size_type> draw_;

    void post_set() { normalize(); }

    void post_set_log()
    {
        normalize_log();
        math::vExp(size_, data_.data(), data_.data());
        normalize();
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

    double ess() const { return ess_inf(); }

    template <typename InputIter>
    double ess(InputIter, bool) const
    {
        return ess_inf();
    }

    template <typename RandomIter>
    double ess(RandomIter, int, bool) const
    {
        return ess_inf();
    }

    template <typename InputIter>
    double cess(InputIter, bool) const
    {
        return ess_inf();
    }

    template <typename RandomIter>
    double cess(RandomIter, int, bool) const
    {
        return ess_inf();
    }

    size_type resample_size() const { return 0; }

    void read_resample_weight(double *) const {}

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

    template <typename URNG>
    size_type draw(URNG &) const
    {
        return 0;
    }

    const double *resample_data() const { return nullptr; }

    const double *data() const { return nullptr; }

    private:
    static double ess_inf() { return std::numeric_limits<double>::infinity(); }
}; // class WeightEmtpy

/// \brief Particle::weight_type trait
/// \ingroup Traits
VSMC_DEFINE_TYPE_DISPATCH_TRAIT(WeightType, weight_type, Weight)

} // namespace vsmc

#endif // VSMC_CORE_WEIGHT_HPP
