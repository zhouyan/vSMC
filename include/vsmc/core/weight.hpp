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
        ess_(static_cast<double>(N)), weight_(N), log_weight_(N),
        ess_cached_(false), weight_cached_(false), log_weight_cached_(false) {}

    /// Read only access to the weights
    const weight_type &weight () const
    {
        if (!weight_cached_) {
            weight_ = log_weight().array().exp();
            double sum = weight_.sum();
            weight_ *= 1 / sum;
            weight_cached_ = true;
        }

        return weight_;
    }

    /// Read only access to the weights
    template <typename SizeType, typename OutputIter>
    void weight (SizeType N, OutputIter *first) const
    {
        assert(weight_.size() >= N);
        std::copy(weight().data(), weight().data() + N, first);
    }

    /// Read only access to the log weights
    const weight_type &log_weight () const
    {
        if (!log_weight_cached_) {
            double max_weight = log_weight_.maxCoeff();
            log_weight_ = log_weight_.array() - max_weight;
            log_weight_cached_ = true;
        }

        return log_weight_;
    }

    /// Read only access to the log weights
    template <typename SizeType, typename OutputIter>
    void log_weight (SizeType N, OutputIter *first) const
    {
        VSMC_RUNTIME_ASSERT((log_weight_.size() >= N),
                "Size of weight set is too small")

        std::copy(log_weight().data(), log_weight().data() + N, first);
    }

    /// Set equal weights for all particles
    void set_equal_weight ()
    {
        ess_ = static_cast<double>(weight_.size());
        weight_.setConstant(1.0 / weight_.size());
        log_weight_.setConstant(0);

        ess_cached_ = true;
        weight_cached_ = true;
        log_weight_cached_ = true;
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
        if (!ess_cached_) {
            ess_ = 1 / weight().squaredNorm();
            ess_cached_ = true;
        }

        return ess_;
    }

    private :

    mutable double ess_;
    mutable weight_type weight_;
    mutable weight_type log_weight_;

    mutable bool ess_cached_;
    mutable bool weight_cached_;
    mutable bool log_weight_cached_;

    void set_weight ()
    {
        ess_cached_ = false;
        weight_cached_ = false;
        log_weight_cached_ = false;
    }
}; // class WeightSetBase

} // namespace vsmc

VSMC_DEFINE_TYPE_DISPATCH_TRAIT(WeightSetType, weight_set_type, WeightSetBase);

#endif // VSMC_CORE_WEIGHT_HPP
