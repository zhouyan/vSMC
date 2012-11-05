#ifndef VSMC_CORE_WEIGHT_HPP
#define VSMC_CORE_WEIGHT_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

/// \brief Weight set class
/// \ingroup WeightSet
class WeightSetBase
{
    public :

    typedef std::vector<double>::size_type size_type;

    explicit WeightSetBase (size_type N) :
        size_(N), ess_(static_cast<double>(N)), weight_(N), log_weight_(N) {}

    template <typename OutputIter>
    OutputIter read_weight (OutputIter first) const
    {
        return std::copy(weight_.begin(), weight_.end(), first);
    }

    double *read_weight (double *first) const
    {
        const double *const wptr = &weight_[0];
        VSMC_RUNTIME_ASSERT(
                (std::abs(first - wptr) > static_cast<std::ptrdiff_t>(size_)),
                "The destination of **WeightBase::read_weight** is "
                "overlapping with the source\n"
                "How did you get this address?");
        std::memcpy(first, wptr, sizeof(double) * size_);

        return first + size_;
    }

    template <typename OutputIter>
    OutputIter read_log_weight (OutputIter first) const
    {
        return std::copy(log_weight_.begin(), log_weight_.end(), first);
    }

    double *read_log_weight (double *first) const
    {
        const double *const lwptr = &log_weight_[0];
        VSMC_RUNTIME_ASSERT(
                (std::abs(first - lwptr) > static_cast<std::ptrdiff_t>(size_)),
                "The destination of **WeightBase::read_weight** is "
                "overlapping with the source\n"
                "How did you get this address?");
        std::memcpy(first, lwptr, sizeof(double) * size_);

        return first + size_;
    }

    double weight (size_type id) const
    {
        return weight_[id];
    }

    double log_weight (size_type id) const
    {
        return log_weight_[id];
    }

    void set_equal_weight ()
    {
        ess_ = static_cast<double>(weight_.size());
        std::fill(weight_.begin(), weight_.end(), 1.0 / weight_.size());
        std::fill(log_weight_.begin(), log_weight_.end(), 0);
    }

    void set_weight (const double *nw, int stride = 1)
    {
        for (size_type i = 0; i != size_; ++i, nw += stride)
            weight_[i] = *nw;
        weight2log_weight();
    }

    void mul_weight (const double *nw, int stride = 1)
    {
        for (size_type i = 0; i != size_; ++i, nw += stride)
            weight_[i] *= *nw;
        weight2log_weight();
    }

    void set_log_weight (const double *nw, int stride = 1)
    {
        for (size_type i = 0; i != size_; ++i, nw += stride)
            log_weight_[i] = *nw;
        log_weight2weight();
    }

    void add_log_weight (const double *nw, int stride = 1)
    {
        for (size_type i = 0; i != size_; ++i, nw += stride)
            log_weight_[i] += *nw;
        log_weight2weight();
    }

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
