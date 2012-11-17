#ifndef VSMC_CORE_WEIGHT_HPP
#define VSMC_CORE_WEIGHT_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

/// \brief Weight set class
/// \ingroup Core
class WeightSetBase
{
    public :

    typedef std::vector<double>::size_type size_type;

    explicit WeightSetBase (size_type N) :
        size_(N), ess_(static_cast<double>(N)), weight_(N), log_weight_(N) {}

    /// \brief Read normalized weights through an output iterator
    template <typename OutputIter>
    OutputIter read_weight (OutputIter first) const
    {
        return std::copy(weight_.begin(), weight_.end(), first);
    }

    /// \brief Read normalized weights through a pointer
    double *read_weight (double *first) const
    {
        const double *const wptr = &weight_[0];
        VSMC_RUNTIME_ASSERT(
                (std::abs(first - wptr) > static_cast<std::ptrdiff_t>(size_)),
                "The destination of **WeightBase::read_weight** is "
                "overlapping with the source.\n"
                "How did you get this address?");
        std::memcpy(first, wptr, sizeof(double) * size_);

        return first + size_;
    }

    /// \brief Read unnormalized logarithm weights through an output iterator
    template <typename OutputIter>
    OutputIter read_log_weight (OutputIter first) const
    {
        return std::copy(log_weight_.begin(), log_weight_.end(), first);
    }

    /// \brief Read unnormalized logarithm weights through a pointer
    double *read_log_weight (double *first) const
    {
        const double *const lwptr = &log_weight_[0];
        VSMC_RUNTIME_ASSERT(
                (std::abs(first - lwptr) > static_cast<std::ptrdiff_t>(size_)),
                "The destination of **WeightBase::read_weight** is "
                "overlapping with the source.\n"
                "How did you get this address?");
        std::memcpy(first, lwptr, sizeof(double) * size_);

        return first + size_;
    }

    /// \brief Get the normalized weight of the id'th particle
    double weight (size_type id) const
    {
        return weight_[id];
    }

    /// \brief Get the unnormalized logarithm weight of the id'th particle
    double log_weight (size_type id) const
    {
        return log_weight_[id];
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS
    /// such that each particle has a equal weight
    void set_equal_weight ()
    {
        ess_ = static_cast<double>(weight_.size());
        std::fill(weight_.begin(), weight_.end(), 1.0 / weight_.size());
        std::fill(log_weight_.begin(), log_weight_.end(), 0);
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) weights directly through an input
    /// iterator
    template <typename InputIter>
    void set_weight (InputIter first)
    {
        for (size_type i = 0; i != size_; ++i, ++first)
            weight_[i] = *first;
        weight2log_weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) weights directly through a random
    /// access iterator with (possible non-uniform) stride
    template <typename RandomIter>
    void set_weight (RandomIter first, int stride)
    {
        for (size_type i = 0; i != size_; ++i, first += stride)
            weight_[i] = *first;
        weight2log_weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) weights directly through a pointer
    void set_weight (const double *first)
    {
        double *const wptr = &weight_[0];
        VSMC_RUNTIME_ASSERT(
                (std::abs(first - wptr) > static_cast<std::ptrdiff_t>(size_)),
                "The source of **WeightBase::set_weight** is "
                "overlapping with the destination.\n"
                "How did you get this address?");
        std::memcpy(wptr, first, sizeof(double) * size_);
        weight2log_weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// multiply the normalized weight with (possible unnormalized) incremental
    /// weights through an input iterator
    template <typename InputIter>
    void mul_weight (InputIter first)
    {
        for (size_type i = 0; i != size_; ++i, ++first)
            weight_[i] *= *first;
        weight2log_weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// multiply the normalized weight with (possible unnormalized) incremental
    /// weights through a random access iterator with (possible non-uniform)
    /// stride
    template <typename RandomIter>
    void mul_weight (RandomIter first, int stride)
    {
        for (size_type i = 0; i != size_; ++i, first += stride)
            weight_[i] *= *first;
        weight2log_weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) logarithm weights directly through
    /// an input iterator
    template <typename InputIter>
    void set_log_weight (InputIter first)
    {
        for (size_type i = 0; i != size_; ++i, ++first)
            log_weight_[i] = *first;
        log_weight2weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) logarithm weights directly through
    /// a random access iterator with (possible non-uniform) stride
    template <typename RandomIter>
    void set_log_weight (RandomIter first, int stride)
    {
        for (size_type i = 0; i != size_; ++i, first += stride)
            log_weight_[i] = *first;
        log_weight2weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) logarithm weights directly through
    /// a pointer
    void set_log_weight (const double *first)
    {
        double *const lwptr = &log_weight_[0];
        VSMC_RUNTIME_ASSERT(
                (std::abs(first - lwptr) > static_cast<std::ptrdiff_t>(size_)),
                "The source of **WeightBase::set_log_weight** is "
                "overlapping with the destination.\n"
                "How did you get this address?");
        std::memcpy(lwptr, first, sizeof(double) * size_);
        log_weight2weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// adding to the unnormalized logarithm weights with (possible
    /// unormalized) logarithm incremental weights through an input iterator
    template <typename InputIter>
    void add_log_weight (InputIter first)
    {
        for (size_type i = 0; i != size_; ++i, ++first)
            log_weight_[i] += *first;
        log_weight2weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// adding to the unnormalized logarithm weights with (possible
    /// unormalized) logarithm incremental weights through a ranodm access
    /// iterator with (possible non-uniform) stride
    template <typename RandomIter>
    void add_log_weight (RandomIter first, int stride)
    {
        for (size_type i = 0; i != size_; ++i, first += stride)
            log_weight_[i] += *first;
        log_weight2weight();
    }

    /// \brief Get the ESS of the particle collection based on the current
    /// weights
    double ess () const
    {
        return ess_;
    }

    private :

    size_type size_;
    double ess_;
    std::vector<double> weight_;
    std::vector<double> log_weight_;

    void log_weight2weight ()
    {
        using std::exp;

        std::vector<double>::const_iterator id_max = std::max_element(
                log_weight_.begin(), log_weight_.end());
        double max_weight = *id_max;
        for (size_type i = 0; i != log_weight_.size(); ++i)
            log_weight_[i] -= max_weight;

        for (size_type i = 0; i != weight_.size(); ++i)
            weight_[i] = exp(log_weight_[i]);
        double coeff = std::accumulate(weight_.begin(), weight_.end(),
                static_cast<double>(0));
        coeff = 1 / coeff;
        for (size_type i = 0; i != weight_.size(); ++i)
            weight_[i] *= coeff;

        ess_ = 0;
        for (size_type i = 0; i != weight_.size(); ++i)
            ess_ += weight_[i] * weight_[i];
        ess_ = 1/ ess_;
    }

    void weight2log_weight ()
    {
        using std::log;

        double coeff = std::accumulate(weight_.begin(), weight_.end(),
                static_cast<double>(0));
        coeff = 1 / coeff;
        for (size_type i = 0; i != weight_.size(); ++i)
            weight_[i] *= coeff;

        for (size_type i = 0; i != weight_.size(); ++i)
            log_weight_[i] = log(weight_[i]);

        ess_ = 0;
        for (size_type i = 0; i != weight_.size(); ++i)
            ess_ += weight_[i] * weight_[i];
        ess_ = 1/ ess_;
    }
}; // class WeightSetBase

} // namespace vsmc

VSMC_DEFINE_TYPE_DISPATCH_TRAIT(WeightSetType, weight_set_type, WeightSetBase);

#endif // VSMC_CORE_WEIGHT_HPP
