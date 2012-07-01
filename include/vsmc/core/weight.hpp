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

    protected :

    template <typename SizeType>
    explicit WeightSetBase (SizeType N) :
        ess_(static_cast<double>(N)), weight_(N), log_weight_(N),
        ess_cached_(false), weight_cached_(false), log_weight_cached_(false),
        zconst_(0), inc_weight_(N)
    {
        set_equal_weight();
    }

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
    /// \param delta A multiplier appiled to the new log weights
    void set_log_weight (const double *nw, double delta = 1)
    {
        Eigen::Map<const weight_type> w(nw, log_weight_.size());
        set_log_weight(w, delta);
    }

    /// \brief Set the log weights with a Eigen object
    ///
    /// \param nw An Eigen::DenseBase object. One dimension Array, Vector,
    /// RowVector are supported. The Scalar type also need to be \c double.
    /// Otherwise it will be a compile-time error
    /// \param delta A multiplier appiled to the new log weights
    template <typename D>
    void set_log_weight (const Eigen::DenseBase<D> &nw, double delta = 1)
    {
        log_weight_ = nw.head(log_weight_.size());
        if (delta != 1)
            log_weight_ *= delta;
        set_weight();
    }

    /// \brief Add to the log weights with a pointer
    ///
    /// \param iw The position to start the reading, it shall be valid
    /// after increments of size() times.
    /// \param delta A multiplier appiled to the new incremental log weights
    /// \param add_zconst Whether this incremental weights shall contribute to
    /// the SMC normalizing constant estimate
    void add_log_weight (const double *iw, double delta = 1,
            bool add_zconst = true)
    {
        Eigen::Map<const weight_type> w(iw, log_weight_.size());
        add_log_weight(w, delta, add_zconst);
    }

    /// \brief Add to the log weights with a weight_object object
    ///
    /// \param iw An Eigen::DenseBase object. One dimension Array, Vector,
    /// RowVector are supported. The Scalar type also need to be \c double.
    /// Otherwise it will be a compile-time error
    /// \param delta A multiplier appiled to the new incremental log weights
    /// \param add_zconst Whether this incremental weights shall contribute to
    /// the SMC normalizing constant estimate
    template <typename D>
    void add_log_weight (const Eigen::DenseBase<D> &iw, double delta = 1,
            bool add_zconst = true)
    {
        using std::log;

        inc_weight_ = iw.head(log_weight_.size());
        if (delta != 1)
            inc_weight_ *= delta;
        log_weight_ += inc_weight_;
        if (add_zconst)
            zconst_ += log(weight().dot(inc_weight_.array().exp().matrix()));
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

    /// Get the value of the logarithm of SMC normalizing constant
    double zconst () const
    {
        return zconst_;
    }

    /// Reset the value of logarithm of SMC normalizing constant to zero
    void reset_zconst ()
    {
        zconst_ = 0;
    }

    private :

    mutable double ess_;
    mutable weight_type weight_;
    mutable weight_type log_weight_;

    mutable bool ess_cached_;
    mutable bool weight_cached_;
    mutable bool log_weight_cached_;

    double zconst_;
    weight_type inc_weight_;

    void set_weight ()
    {
        ess_cached_ = false;
        weight_cached_ = false;
        log_weight_cached_ = false;
    }
}; // class WeightSetBase

namespace internal {

template <typename T>
class HasWeightSetType
{
    private :

    struct char2 {char c1; char c2;};

    template <typename S>
    static char test (typename S::weight_set_type *);

    template <typename S>
    static char2 test (...);

    public :

    static const bool value = sizeof(test<T>(VSMC_NULLPTR)) == sizeof(char);
};

template <typename T, bool>
class WeightSetTypeDispatch;

template <typename T>
class WeightSetTypeDispatch<T, true>
{
    public :

    typedef typename T::weight_set_type type;
};

template <typename T>
class WeightSetTypeDispatch<T, false>
{
    public :

    typedef WeightSetBase type;
};

} // namespace vsmc::internal

/// \brief Trait class of weight_set_type
/// \ingroup Core
template <typename T>
class WeightSetTypeTrait
{
    public :

    /// \brief Type of T::weight_set_type if it exist, otherwise WeightSetBase
    typedef typename internal::WeightSetTypeDispatch<T,
            internal::HasWeightSetType<T>::value>::type type;
}; // class WeightSetTypeTrait

} // namespace vsmc

#endif // VSMC_CORE_WEIGHT_HPP
