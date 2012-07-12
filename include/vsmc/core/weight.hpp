#ifndef VSMC_CORE_WEIGHT_HPP
#define VSMC_CORE_WEIGHT_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

/// \brief Weight set class
/// \ingroup Core
template <typename Mutex, template <typename> class LockGuard>
class WeightSetBase
{
    public :

    /// The type of the weight and log weight vectors
    typedef Eigen::VectorXd weight_type;

    template <typename SizeType>
    explicit WeightSetBase (SizeType N) :
        ess_(static_cast<double>(N)), weight_(N), log_weight_(N),
        ess_new_(true), weight_new_(true), log_weight_new_(true) {}

    WeightSetBase (const WeightSetBase &other) :
        ess_(other.ess_),
        weight_(other.weight_), log_weight_(other.log_weight_),
        ess_new_(other.ess_new_),
        weight_new_(other.weight_new_), log_weight_new_(other.log_weight_new_)
    {}

    const WeightSetBase &operator= (const WeightSetBase &other)
    {
        if (&other != this) {
            LockGuard<Mutex> lock_e(ess_mutex_);
            LockGuard<Mutex> lock_w(weight_mutex_);
            LockGuard<Mutex> lock_l(log_weight_mutex_);

            ess_            = other.ess_;
            weight_         = other.weight_;
            log_weight_     = other.log_weight_;
            ess_new_        = other.ess_new_;
            weight_new_     = other.weight_new_;
            log_weight_new_ = other.log_weight_new_;
        }

        return *this;
    }

    /// Read only access to the weights
    const weight_type &weight () const
    {
        if (weight_new_) {
            LockGuard<Mutex> lock(weight_mutex_);
            if (weight_new_) {
                weight_ = log_weight().array().exp();
                double sum = weight_.sum();
                weight_ *= 1 / sum;
                weight_new_ = false;
            }
        }

        return weight_;
    }

    /// Read only access to the weights
    template <typename SizeType, typename OutputIter>
    void weight (SizeType N, OutputIter *first) const
    {
        std::copy(weight().data(), weight().data() + N, first);
    }

    template <typename SizeType>
    double weight (SizeType id) const
    {
        return weight()[id];
    }

    /// Read only access to the log weights
    const weight_type &log_weight () const
    {
        if (log_weight_new_) {
            LockGuard<Mutex> lock(log_weight_mutex_);
            if (log_weight_new_) {
                double max_weight = log_weight_.maxCoeff();
                log_weight_ = log_weight_.array() - max_weight;
                log_weight_new_ = false;
            }
        }

        return log_weight_;
    }

    /// Read only access to the log weights
    template <typename SizeType, typename OutputIter>
    void log_weight (SizeType N, OutputIter *first) const
    {
        std::copy(log_weight().data(), log_weight().data() + N, first);
    }

    template <typename SizeType>
    double log_weight (SizeType id) const
    {
        return log_weight()[id];
    }

    /// Set equal weights for all particles
    void set_equal_weight ()
    {
        ess_new_ = true;
        LockGuard<Mutex> lock_e(ess_mutex_);
        if (ess_new_) {
            ess_ = static_cast<double>(weight_.size());
            ess_new_ = false;
        }

        weight_new_ = true;
        LockGuard<Mutex> lock_w(weight_mutex_);
        if (weight_new_) {
            weight_.setConstant(1.0 / weight_.size());
            weight_new_ = false;
        }

        log_weight_new_ = true;
        LockGuard<Mutex> lock_l(log_weight_mutex_);
        if (log_weight_new_) {
            log_weight_.setConstant(0);
            log_weight_new_ = false;
        }
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
        LockGuard<Mutex> lock(log_weight_mutex_);
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
        LockGuard<Mutex> lock(log_weight_mutex_);
        log_weight_ += iw.head(log_weight_.size());
        set_weight();
    }

    /// The current ESS (Effective Sample Size)
    double ess () const
    {
        if (ess_new_) {
            LockGuard<Mutex> lock(ess_mutex_);
            if (ess_new_) {
                ess_ = 1 / weight().squaredNorm();
                ess_new_ = false;
            }
        }

        return ess_;
    }

    private :

    mutable double ess_;
    mutable weight_type weight_;
    mutable weight_type log_weight_;

    mutable bool ess_new_;
    mutable bool weight_new_;
    mutable bool log_weight_new_;

    mutable Mutex ess_mutex_;
    mutable Mutex weight_mutex_;
    mutable Mutex log_weight_mutex_;

    void set_weight ()
    {
        ess_new_ = true;
        weight_new_ = true;
        log_weight_new_ = true;
    }
}; // class WeightSetBase

#if VSMC_HAS_LIB_THREAD
typedef WeightSetBase<internal::mutex, internal::lock_guard> WeightSetPrl;
#endif
typedef WeightSetBase<NullMutex, NullLockGuard> WeightSetSeq;

} // namespace vsmc

#if VSMC_HAS_LIB_THREAD
VSMC_DEFINE_TYPE_DISPATCH_TRAIT(WeightSetType, weight_set_type, WeightSetPrl);
#else
VSMC_DEFINE_TYPE_DISPATCH_TRAIT(WeightSetType, weight_set_type, WeightSetSeq);
#endif

#endif // VSMC_CORE_WEIGHT_HPP
