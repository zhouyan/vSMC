#ifndef VSMC_INTERNAL_WEIGHT_HPP
#define VSMC_INTERNAL_WEIGHT_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc { namespace internal {

/// \brief Base weight set
class WeightBase
{
    public :

    /// The type of the size of weight and log weight vectors
    typedef VSMC_SIZE_TYPE size_type;

    /// The type of the weight and log weight vectors
    typedef Eigen::VectorXd weight_type;

    explicit WeightBase (size_type N) :
        size_(N), ess_(static_cast<double>(N)), weight_(N), log_weight_(N),
        ess_cached_(false), weight_cached_(false), log_weight_cached_(false),
        zconst_(0), inc_weight_(N) {}

    /// Size of the weight set
    size_type size () const
    {
        return size_;
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

    /// Read only access to individua log weight
    double log_weight (size_type id) const
    {
        return log_weight()[id];
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

    /// Set equal weights for all particles
    void set_equal_weight ()
    {
        ess_ = static_cast<double>(size_);
        weight_.setConstant(1.0 / size_);
        log_weight_.setConstant(0);

        ess_cached_ = true;
        weight_cached_ = true;
        log_weight_cached_ = true;
    }

    /// \brief Set the log weights with a pointer
    ///
    /// \param new_weight The position to start the reading, it shall be valid
    /// after increments of size() times.
    /// \param delta A multiplier appiled to the new log weights
    void set_log_weight (const double *new_weight, double delta = 1)
    {
        Eigen::Map<const weight_type> w(new_weight, size_);
        set_log_weight(w, delta);
    }

    /// Set the log weights with a weight_type object
    void set_log_weight (const weight_type &new_weight, double delta = 1)
    {
        log_weight_ = new_weight.head(size_);
        if (delta != 1)
            log_weight_ *= delta;
        set_log_weight();
    }

    /// \brief Add to the log weights with a pointer
    ///
    /// \param inc_weight The position to start the reading, it shall be valid
    /// after increments of size() times.
    /// \param delta A multiplier appiled to the new incremental log weights
    /// \param add_zconst Whether this incremental weights shall contribute to
    /// the SMC normalizing constant estimate
    void add_log_weight (const double *inc_weight, double delta = 1,
            bool add_zconst = true)
    {
        Eigen::Map<const weight_type> w(inc_weight, size_);
        add_log_weight(w, delta, add_zconst);
    }

    /// Add to the log weights with a weight_object object
    void add_log_weight (const weight_type &inc_weight, double delta = 1,
            bool add_zconst = true)
    {
        using std::log;

        inc_weight_ = inc_weight.head(size_);
        if (delta != 1)
            inc_weight_ *= delta;
        log_weight_ += inc_weight_;
        if (add_zconst)
            zconst_ += log(weight().dot(inc_weight_.array().exp().matrix()));
        set_log_weight();
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

    size_type size_;

    mutable double ess_;
    mutable weight_type weight_;
    mutable weight_type log_weight_;

    mutable bool ess_cached_;
    mutable bool weight_cached_;
    mutable bool log_weight_cached_;

    double zconst_;
    weight_type inc_weight_;

    void set_log_weight ()
    {
        ess_cached_ = false;
        weight_cached_ = false;
        log_weight_cached_ = false;

        assert(weight_.size() == size_);
        assert(log_weight_.size() == size_);
        assert(inc_weight_.size() == size_);
    }
}; // class WeightBase

template <typename T>
class Weight : public WeightBase
{
    public :

    typedef VSMC_SIZE_TYPE size_type;

    Weight (size_type N) : WeightBase(N) {}
}; // class Weight

/// \brief Copy weight
///
/// \tparam T1 Requirement: support index operation src[i]
/// \tparam T2 Requirement: support index operation des[i]
template <typename T1, typename T2>
void copy_weight (const T1 &src, T2 &des)
{
    for (std::size_t i = 0; i != src.size(); ++i)
        des[i] = src[i];
}

template <typename T>
void copy_weight (const T &src, T &des)
{
    des = src;
}

} } // namespace vsmc::internal

#endif // VSMC_INTERNAL_WEIGHT_HPP
