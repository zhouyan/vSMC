#ifndef VSMC_CORE_PARTICLE_HPP
#define VSMC_CORE_PARTICLE_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/weight.hpp>
#include <vsmc/rng/random.hpp>

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
class Particle : public Weight<T>
{
    public :

    /// The type of the number of particles
    typedef typename SizeTypeTrait<T>::type size_type;

    /// The type of the particle values
    typedef T value_type;

    /// The type of the Counter-based random number generator
    typedef VSMC_CBRNG_TYPE cbrng_type;

    /// The type of the Counter-based random number generator C++11 engine
    typedef rng::Engine<VSMC_CBRNG_TYPE> rng_type;

    /// \brief Construct a Particle object with a given number of particles
    ///
    /// \param N The number of particles
    explicit Particle (size_type N) :
        Weight<T>(N), size_(N), value_(N),
        replication_(N), copy_from_(N), weight_(N), remain_(N),
        resampled_(false), rng_(N)
    {
        rng::Seed &seed = rng::Seed::create();
        for (size_type i = 0; i != size_; ++i) {
            rng_[i] = rng_type(
                    static_cast<rng_type::result_type>(seed.get()));
        }
        this->set_equal_weight();
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

        resampled_ = this->ess() < threshold * size_;
        if (resampled_) {
            copy_weight(this->weight(), weight_);
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

    /// \brief Get a C++11 RNG engine
    ///
    /// \param id The position of the particle, 0 to size() - 1
    ///
    /// \return A reference to a C++11 RNG engine unique to particle at
    /// position id, and independent of others
    rng_type &rng (size_type id)
    {
        return rng_[id];
    }

    private :

    size_type size_;
    value_type value_;

    Eigen::Matrix<size_type, Eigen::Dynamic, 1> replication_;
    Eigen::Matrix<size_type, Eigen::Dynamic, 1> copy_from_;
    Eigen::VectorXd weight_;
    Eigen::VectorXd remain_;
    bool resampled_;

    std::vector<rng_type> rng_;

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
        double u = unif(rng_[j]);
        double cw = weight_[0];
        while (j != size_) {
            while (j < cw * size_ - u && j != size_) {
                ++replication_[k];
                u = unif(rng_[j]);
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
        double u = unif(rng_[j]);
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
        double u = unif(rng_[j]);
        double cw = weight_[0];
        while (j != size) {
            while (j < cw * size - u && j != size) {
                ++replication_[k];
                u = unif(rng_[j]);
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
        double u = unif(rng_[j]);
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
        double tp = weight_.sum();
        double sum_p = 0;
        size_type sum_n = 0;
        replication_.setConstant(0);
        for (size_type i = 0; i != size_; ++i) {
            if (sum_n < size && weight_[i] > 0) {
                rng::binomial_distribution<
                    typename internal::make_signed<size_type>::type>
                    binom(size - sum_n, weight_[i] / (tp - sum_p));
                replication_[i] = binom(rng_[i]);
            }
            sum_p += weight_[i];
            sum_n += replication_[i];
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
        this->set_equal_weight();
    }
}; // class Particle

} // namespace vsmc

#endif // VSMC_CORE_PARTICLE_HPP
