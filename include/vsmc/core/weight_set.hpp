//============================================================================
// vSMC/include/vsmc/core/weight_set.hpp
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

#ifndef VSMC_CORE_WEIGHT_SET_HPP
#define VSMC_CORE_WEIGHT_SET_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/rng/discrete_distribution.hpp>

namespace vsmc
{

/// \brief Weight set class
/// \ingroup Core
class WeightSet
{
    public:
    using size_type = std::size_t;

    explicit WeightSet(size_type N)
        : size_(N), ess_(static_cast<double>(N)), weight_(N), log_weight_(N)
    {
    }

    WeightSet(const WeightSet &) = default;
    WeightSet &operator=(const WeightSet &) = default;
    WeightSet(WeightSet &&) = default;
    WeightSet &operator=(WeightSet &&) = default;

    virtual ~WeightSet() {}

    /// \brief Set all weights to be equal, normalized and return the ESS
    ///
    /// \param N Number of particles in weight
    /// \param RN Number  of particles used for resampling
    /// \param weight Weights to be equalized
    static double set_equal_weight(
        std::size_t N, std::size_t RN, double *weight)
    {
        double ess = static_cast<double>(RN);
        if (weight != nullptr)
            std::fill_n(weight, N, 1 / ess);

        return ess;
    }

    /// \brief Normalize logarithm weights such that the maximum is zero
    static void normalize_log_weight(std::size_t N, double *log_weight)
    {
        double dmax = *(std::max_element(log_weight, log_weight + N));
        for (std::size_t i = 0; i != N; ++i)
            log_weight[i] -= dmax;
    }

    /// \brief Normalize weights such that the summation is one and return the
    /// ESS
    static double normalize_weight(std::size_t N, double *weight)
    {
        double coeff = 1 / math::asum(N, weight, 1);
        math::scal(N, coeff, weight, 1);

        return 1 / math::dot(N, weight, 1, weight, 1);
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
    /// in derived classes they might not be. For example, in `WeightSetMPI`,
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
        std::copy(weight_.begin(), weight_.end(), first);
    }

    /// \brief Read normalized weights through a random access iterator with
    /// (possible non-uniform stride)
    template <typename RandomIter>
    void read_weight(RandomIter first, int stride) const
    {
        for (size_type i = 0; i != size_; ++i, first += stride)
            *first = weight_[i];
    }

    /// \brief Read unnormalized logarithm weights through an output iterator
    template <typename OutputIter>
    void read_log_weight(OutputIter first) const
    {
        std::copy(log_weight_.begin(), log_weight_.end(), first);
    }

    /// \brief Read unnormalized logarithm weights through a random access
    /// iterator with (possible non-uniform stride)
    template <typename RandomIter>
    void read_log_weight(RandomIter first, int stride) const
    {
        for (size_type i = 0; i != size_; ++i, first += stride)
            *first = log_weight_[i];
    }

    /// \brief Get the normalized weight of the id'th particle
    double weight(size_type id) const { return weight_[id]; }

    /// \brief Get the unnormalized logarithm weight of the id'th particle
    double log_weight(size_type id) const { return log_weight_[id]; }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS
    /// such that each particle has a equal weight
    void set_equal_weight()
    {
        set_ess(set_equal_weight(size_, resample_size(), weight_.data()));
        std::memset(log_weight_.data(), 0, sizeof(double) * size_);
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) weights directly through an input
    /// iterator
    template <typename InputIter>
    void set_weight(InputIter first)
    {
        std::copy_n(first, size_, weight_.begin());
        post_set_weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) weights directly through a random
    /// access iterator with (possible non-uniform) stride
    template <typename RandomIter>
    void set_weight(RandomIter first, int stride)
    {
        for (size_type i = 0; i != size_; ++i, first += stride)
            weight_[i] = *first;
        post_set_weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// multiply the normalized weight with (possible unnormalized)
    /// incremental
    /// weights through an input iterator
    template <typename InputIter>
    void mul_weight(InputIter first)
    {
        for (size_type i = 0; i != size_; ++i, ++first)
            weight_[i] *= *first;
        post_set_weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// multiply the normalized weight with (possible unnormalized)
    /// incremental
    /// weights through a random access iterator with (possible non-uniform)
    /// stride
    template <typename RandomIter>
    void mul_weight(RandomIter first, int stride)
    {
        for (size_type i = 0; i != size_; ++i, first += stride)
            weight_[i] *= *first;
        post_set_weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) logarithm weights directly
    /// through
    /// an input iterator
    template <typename InputIter>
    void set_log_weight(InputIter first)
    {
        std::copy_n(first, size_, log_weight_.begin());
        post_set_log_weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) logarithm weights directly
    /// through
    /// a random access iterator with (possible non-uniform) stride
    template <typename RandomIter>
    void set_log_weight(RandomIter first, int stride)
    {
        for (size_type i = 0; i != size_; ++i, first += stride)
            log_weight_[i] = *first;
        post_set_log_weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// adding to the unnormalized logarithm weights with (possible
    /// unormalized) logarithm incremental weights through an input iterator
    template <typename InputIter>
    void add_log_weight(InputIter first)
    {
        for (size_type i = 0; i != size_; ++i, ++first)
            log_weight_[i] += *first;
        post_set_log_weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// adding to the unnormalized logarithm weights with (possible
    /// unormalized) logarithm incremental weights through a ranodm access
    /// iterator with (possible non-uniform) stride
    template <typename RandomIter>
    void add_log_weight(RandomIter first, int stride)
    {
        for (size_type i = 0; i != size_; ++i, first += stride)
            log_weight_[i] += *first;
        post_set_log_weight();
    }

    /// \brief Draw a sample according to the weights
    template <typename URNG>
    size_type draw(URNG &eng) const
    {
        return draw_(eng, weight_.begin(), weight_.end(), true);
    }

    /// \brief Read only access to the resampling weights
    virtual const double *resample_weight_data() const
    {
        return weight_.data();
    }

    /// \brief Read only access to the raw data of weight
    const double *weight_data() const { return weight_.data(); }

    /// \brief Read only access to the raw data of logarithm weight
    const double *log_weight_data() const { return log_weight_.data(); }

    protected:
    /// \brief Set the value of ESS;
    void set_ess(double e) { ess_ = e; }

    /// \brief Write access to the raw data of weight
    double *mutable_weight_data() { return weight_.data(); }

    /// \brief Write access to the raw data of logarithm weight
    double *mutable_log_weight_data() { return log_weight_.data(); }

    /// \brief Compute unormalized logarithm weights from normalized weights
    virtual void log_weight2weight()
    {
        math::vExp(size_, log_weight_.data(), weight_.data());
    }

    /// \brief Compute unormalized weights from normalized logarithm weights
    virtual void weight2log_weight()
    {
        math::vLn(size_, weight_.data(), log_weight_.data());
    }

    /// \brief Normalize logarithm weights such that the maximum is zero
    virtual void normalize_log_weight()
    {
        normalize_log_weight(size_, log_weight_.data());
    }

    /// \brief Normalize weights such that the summation is one
    virtual void normalize_weight()
    {
        set_ess(normalize_weight(size_, weight_.data()));
    }

    /// \brief Compute ESS given (logarithm) unormalzied incremental weights
    virtual double compute_ess(const double *first, bool use_log) const
    {
        Vector<double> buffer(size_);
        if (use_log) {
            math::vAdd(size_, log_weight_.data(), first, buffer.data());
            double dmax = buffer[0];
            for (size_type i = 0; i != size_; ++i)
                if (dmax < buffer[i])
                    dmax = buffer[i];
            dmax = -dmax;
            for (size_type i = 0; i != size_; ++i)
                buffer[i] += dmax;
            math::vExp(size_, buffer.data(), buffer.data());
        } else {
            math::vMul(size_, weight_.data(), first, buffer.data());
        }

        double coeff = 1 / math::asum(size_, buffer.data(), 1);
        math::scal(size_, coeff, buffer.data(), 1);

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
            double wb = weight_[i] * bptr[i];
            above += wb;
            below += wb * bptr[i];
        }

        return above * above / below;
    }

    private:
    size_type size_;
    double ess_;
    Vector<double> weight_;
    Vector<double> log_weight_;
    DiscreteDistribution<size_type> draw_;

    void post_set_log_weight()
    {
        normalize_log_weight();
        log_weight2weight();
        normalize_weight();
    }

    void post_set_weight()
    {
        normalize_weight();
        weight2log_weight();
        normalize_log_weight();
    }
}; // class WeightSet

/// \brief An empty weight set class
/// \ingroup Core
///
/// \details
/// This class provides all the interfaces of WeightSet, while they do nothing
/// at all and the class cost no memory usage. This is primarily to be used in
/// algorithms where weights are irrelevant. Any attempt of using member
/// functions of this class will not result in compile time or runtime errors,
/// but the results might not be what one will be expecting.
class WeightSetNull
{
    public:
    using size_type = std::size_t;

    explicit WeightSetNull(size_type) {}

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

    template <typename OutputIter>
    void read_log_weight(OutputIter) const
    {
    }

    template <typename RandomIter>
    void read_log_weight(RandomIter, int) const
    {
    }

    double weight(size_type) const { return 1; }

    double log_weight(size_type) const { return 0; }

    void set_equal_weight() {}

    template <typename InputIter>
    void set_weight(InputIter)
    {
    }

    template <typename RandomIter>
    void set_weight(RandomIter, int)
    {
    }

    template <typename InputIter>
    void mul_weight(InputIter)
    {
    }

    template <typename RandomIter>
    void mul_weight(RandomIter, int)
    {
    }

    template <typename InputIter>
    void set_log_weight(InputIter)
    {
    }

    template <typename RandomIter>
    void set_log_weight(RandomIter, int)
    {
    }

    template <typename InputIter>
    void add_log_weight(InputIter)
    {
    }

    template <typename RandomIter>
    void add_log_weight(RandomIter, int)
    {
    }

    template <typename URNG>
    size_type draw(URNG &) const
    {
        return 0;
    }

    const double *resample_weight_data() const { return nullptr; }

    const double *weight_data() const { return nullptr; }

    const double *log_weight_data() const { return nullptr; }

    private:
    static double ess_inf() { return std::numeric_limits<double>::infinity(); }
}; // class WeightSetEmtpy

/// \brief Particle::weight_set_type trait
/// \ingroup Traits
VSMC_DEFINE_TYPE_DISPATCH_TRAIT(WeightSetType, weight_set_type, WeightSet)

} // namespace vsmc

#endif // VSMC_CORE_WEIGHT_SET_HPP
