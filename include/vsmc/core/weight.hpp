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
    typedef std::vector<double>::size_type size_type;

    explicit WeightSetBase (size_type N) :
        ess_(static_cast<double>(N)), weight_(N), log_weight_(N) {}

    /// Read only access to the weights
    template <typename OutputIter>
    void read_weight (OutputIter first) const
    {
        std::copy(weight_.begin(), weight_.end(), first);
    }

    /// Read only access to the log weights
    template <typename OutputIter>
    void read_log_weight (OutputIter first) const
    {
        std::copy(log_weight_.begin(), log_weight_.end(), first);
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
        std::fill(weight_.begin(), weight_.end(), 1.0 / weight_.size());
        std::fill(log_weight_.begin(), log_weight_.end(), 0);
    }

    /// \brief Set the weights with a pointer
    ///
    /// \param nw The position to start the reading, it shall be valid
    /// after increments of size() times.
    void set_weight (const double *nw, int inc = 1)
    {
        std::copy(nw, nw + weight_.size(), weight_.begin());
        weight2log_weight();
    }

    /// \brief Multiple the weight with a pointer
    ///
    /// \param iw The position to start the reading, it shall be valid
    /// after increments of size() times.
    void mul_weight (const double *iw, int inc = 1)
    {
        for (size_type i = 0; i != log_weight_.size(); ++i)
            weight_[i] *= iw[i];
        weight2log_weight();
    }

    /// \brief Set the log weights with a pointer
    ///
    /// \param nw The position to start the reading, it shall be valid
    /// after increments of size() times.
    void set_log_weight (const double *nw, int inc = 1)
    {
        std::copy(nw, nw + log_weight_.size(), log_weight_.begin());
        log_weight2weight();
    }

    /// \brief Add to the log weights with a pointer
    ///
    /// \param iw The position to start the reading, it shall be valid
    /// after increments of size() times.
    void add_log_weight (const double *iw, int inc = 1)
    {
        for (size_type i = 0; i != log_weight_.size(); ++i)
            log_weight_[i] += iw[i];
        log_weight2weight();
    }

    /// The current ESS (Effective Sample Size)
    double ess () const
    {
        return ess_;
    }

    private :

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
