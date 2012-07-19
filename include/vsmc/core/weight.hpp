#ifndef VSMC_CORE_WEIGHT_HPP
#define VSMC_CORE_WEIGHT_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

/// \brief Weight set class
/// \ingroup Core
class WeightSetBase
{
    public :

    /// The type of the weight and log weight vectors
    typedef Eigen::VectorXd weight_type;

    template <typename SizeType>
    explicit WeightSetBase (SizeType N) :
        ess_(static_cast<double>(N)), weight_(N), log_weight_(N) {}

    /// Read only access to the weights
    template <typename OutputIter>
    void read_weight (OutputIter first) const
    {
        std::copy(weight_.data(), weight_.data() + weight_.size(), first);
    }

    /// Read only access to the log weights
    template <typename OutputIter>
    void read_log_weight (OutputIter first) const
    {
        std::copy(log_weight_.data(), log_weight_.data() + log_weight_.size(),
                first);
    }

    /// Read only access to the weight of a particle
    template <typename SizeType>
    double weight (SizeType id) const
    {
        return weight_[id];
    }

    /// Read only access to the log weight of a particle
    template <typename SizeType>
    double log_weight (SizeType id) const
    {
        return log_weight_[id];
    }

    /// Set equal weights for all particles
    void set_equal_weight ()
    {
        ess_ = static_cast<double>(weight_.size());
        weight_.setConstant(1.0 / weight_.size());
        log_weight_.setConstant(0);
    }

    /// \brief Set the log weights with a pointer
    ///
    /// \param nw The position to start the reading, it shall be valid
    /// after increments of size() times.
    void set_log_weight (const double *nw)
    {
        Eigen::Map<const weight_type> w(nw, log_weight_.size());
        set_log_weight(w);
    }

    /// \brief Set the log weights with a Eigen object
    ///
    /// \param nw An Eigen::DenseBase object. One dimension Array, Vector,
    /// RowVector are supported. The Scalar type also need to be \c double.
    /// Otherwise it will be a compile-time error
    template <typename D>
    void set_log_weight (const Eigen::DenseBase<D> &nw)
    {
        log_weight_ = nw.head(log_weight_.size());
        set_weight();
    }

    /// \brief Add to the log weights with a pointer
    ///
    /// \param iw The position to start the reading, it shall be valid
    /// after increments of size() times.
    void add_log_weight (const double *iw)
    {
        Eigen::Map<const weight_type> w(iw, log_weight_.size());
        add_log_weight(w);
    }

    /// \brief Add to the log weights with a weight_object object
    ///
    /// \param iw An Eigen::DenseBase object. One dimension Array, Vector,
    /// RowVector are supported. The Scalar type also need to be \c double.
    /// Otherwise it will be a compile-time error
    template <typename D>
    void add_log_weight (const Eigen::DenseBase<D> &iw)
    {
        log_weight_ += iw.head(log_weight_.size());
        set_weight();
    }

    /// The current ESS (Effective Sample Size)
    double ess () const
    {
        return ess_;
    }

    private :

    mutable double ess_;
    mutable weight_type weight_;
    mutable weight_type log_weight_;

    void set_weight ()
    {
        double max_weight = log_weight_.maxCoeff();
        log_weight_ = log_weight_.array() - max_weight;

        weight_ = log_weight_.array().exp();
        double sum = weight_.sum();
        weight_ *= 1 / sum;

        ess_ = 1 / weight_.squaredNorm();
    }
}; // class WeightSetBase

} // namespace vsmc

VSMC_DEFINE_TYPE_DISPATCH_TRAIT(WeightSetType, weight_set_type, WeightSetBase);

#endif // VSMC_CORE_WEIGHT_HPP
