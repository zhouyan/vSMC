#ifndef VSMC_CORE_WEIGHT_HPP
#define VSMC_CORE_WEIGHT_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

/// \brief Weight set class
/// \ingroup Core
class WeightSetBase
{
    public :

    /// The type of the size of the weight set
    typedef std::size_t size_type;

    explicit WeightSetBase (size_type N) :
        ess_(static_cast<double>(N)), weight_(N), log_weight_(N) {}

    /// Read only access to the weights
    template <typename OutputIter>
    void read_weight (OutputIter first) const
    {
        std::copy(&weight_[0], &weight_[0] + weight_.size(), first);
    }

    /// Read only access to the log weights
    template <typename OutputIter>
    void read_log_weight (OutputIter first) const
    {
        std::copy(&log_weight_[0], &log_weight_[0] + log_weight_.size(),
                first);
    }

    /// Read only access to the weight of a particle
    double weight (size_type id) const
    {
        return weight_[id];
    }

    /// Read only access to the log weight of a particle
    double log_weight (size_type id) const
    {
        return log_weight_[id];
    }

    /// Set equal weights for all particles
    void set_equal_weight ()
    {
        ess_ = static_cast<double>(weight_.size());
        std::fill(&weight_[0], &weight_[0] + weight_.size(),
                1.0 / weight_.size());
        std::fill(&log_weight_[0], &log_weight_[0] + log_weight_.size(), 0);
    }

    /// \brief Set the log weights with a pointer
    ///
    /// \param nw The position to start the reading, it shall be valid
    /// after increments of size() times.
    void set_log_weight (const double *nw)
    {
        std::copy(nw, nw + log_weight_.size(), &log_weight_[0]);
        set_weight();
    }

    /// \brief Add to the log weights with a pointer
    ///
    /// \param iw The position to start the reading, it shall be valid
    /// after increments of size() times.
    void add_log_weight (const double *iw)
    {
        for (size_type i = 0; i != log_weight_.size(); ++i)
            log_weight_[i] += iw[i];
        set_weight();
    }

    /// The current ESS (Effective Sample Size)
    double ess () const
    {
        return ess_;
    }

    private :

    mutable double ess_;
    mutable std::valarray<double> weight_;
    mutable std::valarray<double> log_weight_;

    void set_weight ()
    {
        using std::exp;

        double max_weight = log_weight_.max();
        log_weight_ -= max_weight;
        weight_ = exp(log_weight_);
        double sum = weight_.sum();
        weight_ *= 1 / sum;

        ess_ = 1 / (weight_ * weight_).sum();
    }
}; // class WeightSetBase

} // namespace vsmc

VSMC_DEFINE_TYPE_DISPATCH_TRAIT(WeightSetType, weight_set_type, WeightSetBase);

#endif // VSMC_CORE_WEIGHT_HPP
