#ifndef VSMC_CORE_PARTICLE_HPP
#define VSMC_CORE_PARTICLE_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/parallel_rng.hpp>
#include <vsmc/core/weight.hpp>

namespace vsmc {

/// \brief Particle class representing the whole particle set
/// \ingroup Core
///
/// \tparam T Requirement:
/// \li Constructor compatible with
/// \code T(Particle<T>::size_type N) \endcode
/// \li member function copy method compatible with
/// \code
/// copy(const size_type *copy_from)
/// \endcode
/// where <tt>copy_from[to]</tt> is the index of the particle to be copied into
/// position to. That is you should replace particle at position \c to with
/// another at position <tt>from = copy_from[to]</tt>.
template <typename T>
class Particle : public ParallelRNG<T>, public Weight<T>
{
    public :

    /// The type of the number of particles
    typedef typename SizeTypeTrait<T>::type size_type;

    /// The type of the particle values
    typedef T value_type;

    using typename ParallelRNG<T>::rng_type;
    using typename Weight<T>::weight_type;

    using ParallelRNG<T>::rng;
    using Weight<T>::weight;
    using Weight<T>::log_weight;
    using Weight<T>::set_equal_weight;
    using Weight<T>::set_log_weight;
    using Weight<T>::add_log_weight;
    using Weight<T>::ess;
    using Weight<T>::zconst;
    using Weight<T>::reset_zconst;

    /// \brief Construct a Particle object with a given number of particles
    ///
    /// \param N The number of particles
    explicit Particle (size_type N) :
        ParallelRNG<T>(N), Weight<T>(N), size_(N), value_(N),
        replication_(N), copy_from_(N), weight_(N), remain_(N),
        resampled_(false)
    {
        set_equal_weight();
    }

    /// Size of the particle set
    size_type size () const
    {
        return size_;
    }

    /// Read and write access to particle values
    value_type &value ()
    {
        return value_;
    }

    /// Read only access to particle values
    const value_type &value () const
    {
        return value_;
    }

    /// Whether resampling was performed when resampling(scheme, threshold) was
    /// last called.
    bool resampled () const
    {
        return resampled_;
    }

    /// \brief Perform resampling if ess() < threshold * size()
    ///
    /// \param scheme The resampling scheme, see ResamplingScheme
    /// \param threshold The threshold for resampling
    void resample (ResampleScheme scheme, double threshold)
    {
        using internal::copy_weight;

        assert(replication_.size() == size());

        resampled_ = ess() < threshold * size_;
        if (resampled_) {
            copy_weight(weight(), weight_);
            switch (scheme) {
                case MULTINOMIAL :
                    resample_multinomial();
                    break;
                case RESIDUAL :
                    resample_residual();
                    break;
                case STRATIFIED :
                    resample_stratified();
                    break;
                case SYSTEMATIC :
                    resample_systematic();
                    break;
                case RESIDUAL_STRATIFIED :
                    resample_residual_stratified ();
                    break;
                default :
                    resample_stratified();
                    break;
            }
            resample_do();
        }
    }

    private :

    size_type size_;
    value_type value_;

    Eigen::Matrix<size_type, Eigen::Dynamic, 1> replication_;
    Eigen::Matrix<size_type, Eigen::Dynamic, 1> copy_from_;
    Eigen::VectorXd weight_;
    Eigen::VectorXd remain_;
    bool resampled_;

    void resample_multinomial ()
    {
        weight2replication(size_);
    }

    void resample_residual ()
    {
        using std::modf;

        for (size_type i = 0; i != size_; ++i)
            weight_[i] = modf(size_ * weight_[i], &remain_[i]);
        weight2replication(static_cast<size_type>(weight_.sum()));
        for (size_type i = 0; i != size_; ++i)
            replication_[i] += static_cast<size_type>(remain_[i]);
    }

    void resample_stratified ()
    {
        replication_.setConstant(0);
        size_type j = 0;
        size_type k = 0;
        rng::uniform_real_distribution<double> unif(0,1);
        double u = unif(rng(j));
        double cw = weight_[0];
        while (j != size_) {
            while (j < cw * size_ - u && j != size_) {
                ++replication_[k];
                u = unif(rng(j));
                ++j;
            }
            if (k == size_ - 1)
                break;
            cw += weight_[++k];
        }
    }

    void resample_systematic ()
    {
        replication_.setConstant(0);
        size_type j = 0;
        size_type k = 0;
        rng::uniform_real_distribution<double> unif(0,1);
        double u = unif(rng(j));
        double cw = weight_[0];
        while (j != size_) {
            while (j < cw * size_ - u && j != size_) {
                ++replication_[k];
                ++j;
            }
            if (k == size_ - 1)
                break;
            cw += weight_[++k];
        }
    }

    void resample_residual_stratified ()
    {
        using std::modf;

        replication_.setConstant(0);
        for (size_type i = 0; i != size_; ++i)
            weight_[i] = modf(size_ * weight_[i], &remain_[i]);
        double dsize = (weight_.sum());
        size_type size = static_cast<size_type>(dsize);
        weight_ /= dsize;
        size_type j = 0;
        size_type k = 0;
        rng::uniform_real_distribution<double> unif(0,1);
        double u = unif(rng(j));
        double cw = weight_[0];
        while (j != size) {
            while (j < cw * size - u && j != size) {
                ++replication_[k];
                u = unif(rng(j));
                ++j;
            }
            if (k == size_ - 1)
                break;
            cw += weight_[++k];
        }
        for (size_type i = 0; i != size_; ++i)
            replication_[i] += static_cast<size_type>(remain_[i]);
    }

    void resample_residual_systematic ()
    {
        using std::modf;

        replication_.setConstant(0);
        for (size_type i = 0; i != size_; ++i)
            weight_[i] = modf(size_ * weight_[i], &remain_[i]);
        double dsize = (weight_.sum());
        size_type size = static_cast<size_type>(dsize);
        weight_ /= dsize;
        size_type j = 0;
        size_type k = 0;
        rng::uniform_real_distribution<double> unif(0,1);
        double u = unif(rng(j));
        double cw = weight_[0];
        while (j != size) {
            while (j < cw * size - u && j != size) {
                ++replication_[k];
                ++j;
            }
            if (k == size_ - 1)
                break;
            cw += weight_[++k];
        }
        for (size_type i = 0; i != size_; ++i)
            replication_[i] += remain_[i];
    }

    void weight2replication (size_type size)
    {
        double sum_w = weight_.sum();
        double acc_w = 0;
        size_type acc_s = 0;
        replication_.setConstant(0);
        for (size_type i = 0; i != size_; ++i) {
            if (acc_s < size && weight_[i] > 0) {
                typedef typename internal::make_signed<size_type>::type s_t;
                s_t s = size - acc_s;
                double p = weight_[i] / (sum_w - acc_w);
#ifndef NDEBUG
                if (p < 0)
                    assert(p > -1e-6);
                if (p > 1)
                    assert(p - 1 < 1e-6);
#endif
                p = std::max(p, 0.0);
                p = std::min(p, 1.0);
                rng::binomial_distribution<s_t> binom(s, p);
                replication_[i] = binom(rng(i));
            }
            acc_w += weight_[i];
            acc_s += replication_[i];
        }
    }

    void resample_do ()
    {
        // Some times the nuemrical round error can cause the total childs
        // differ from number of particles
        size_type sum = replication_.sum();
        if (sum != size_) {
            size_type id_max;
            replication_.maxCoeff(&id_max);
            replication_[id_max] += size_ - sum;
        }

        size_type from = 0;
        size_type time = 0;
        for (size_type to = 0; to != size_; ++to) {
            if (replication_[to]) {
                copy_from_[to] = to;
            } else {
                // replication_[to] has zero child, copy from elsewhere
                if (replication_[from] - time <= 1) {
                    // only 1 child left on replication_[from]
                    time = 0;
                    do // move from to some position with at least 2 children
                        ++from;
                    while (replication_[from] < 2);
                }
                copy_from_[to] = from;
                ++time;
            }
        }

        const size_type *cf = copy_from_.data();
        value_.copy(cf);
        set_equal_weight();
    }
}; // class Particle

} // namespace vsmc

#endif // VSMC_CORE_PARTICLE_HPP
