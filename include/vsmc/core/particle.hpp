#ifndef VSMC_CORE_PARTICLE_HPP
#define VSMC_CORE_PARTICLE_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/resample.hpp>
#include <vsmc/core/rng.hpp>
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
class Particle :
    public RngSetTypeTrait<T>::type,
    public WeightSetTypeTrait<T>::type
{
    public :

    /// The type of the number of particles
    typedef typename SizeTypeTrait<T>::type size_type;

    /// The type of the particle values
    typedef T value_type;

    /// The type of the RNG set
    typedef typename RngSetTypeTrait<T>::type rng_set_type;

    /// The type of the particle weights set
    typedef typename WeightSetTypeTrait<T>::type weight_set_type;

    /// The type of resampling operation functor
    typedef cxx11::function<void
        (size_type, rng_set_type &, double *, size_type *)> resample_op_type;

    using typename rng_set_type::rng_type;
    using typename weight_set_type::weight_type;

    using rng_set_type::rng;
    using weight_set_type::weight;
    using weight_set_type::log_weight;
    using weight_set_type::read_weight;
    using weight_set_type::read_log_weight;
    using weight_set_type::set_equal_weight;
    using weight_set_type::set_log_weight;
    using weight_set_type::add_log_weight;
    using weight_set_type::ess;

    /// \brief Construct a Particle object with a given number of particles
    ///
    /// \param N The number of particles
    explicit Particle (size_type N) :
        rng_set_type(N), weight_set_type(N), size_(N), value_(N),
        replication_(N), copy_from_(N), weight_(N), resampled_(false)
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

    /// \brief Try resampling with current scheme
    ///
    /// \param threshold The threshold for resampling
    void resample (double threshold)
    {
        resample(resample_op_, threshold);
    }

    /// \brief Try resampling with an external resampling operation functor
    ///
    /// \param res_op The resampling operation functor (will be used and not
    /// copied)
    /// \param threshold The threshold for resampling
    void resample (resample_op_type &res_op, double threshold)
    {
        resampled_ = ess() < threshold * size_;
        if (resampled_) {
            read_weight(weight_.data());
            resample_op_(size_, *this, weight_.data(), replication_.data());
            resample_do();
        }
    }

    /// \brief Try resampling with a new resampling operation functor
    ///
    /// \param res_op The new resampling operation functor (will be copied and
    /// replace the old one)
    /// \param threshold The threshold for resampling
    void resample (const resample_op_type &res_op, double threshold)
    {
        resample_scheme(res_op);
        resample(threshold);
    }

    /// \brief Try resampling with a new resampling built-in scheme
    ///
    /// \param scheme The new resampling scheme
    /// \param threshold The threshold for resampling
    void resample (ResampleScheme scheme, double threshold)
    {
        resample_scheme(scheme);
        resample(threshold);
    }

    /// Whether resampling was performed when resampling(threshold) was last
    /// called.
    bool resampled () const
    {
        return resampled_;
    }

    /// Replace the resampling operation functor with a new one
    void resample_scheme (const resample_op_type &res_op)
    {
        resample_op_ = res_op;
    }

    /// Replace the resampling operation functor with a new built-in scheme
    void resample_scheme (ResampleScheme scheme)
    {
        switch (scheme) {
            case MULTINOMIAL :
                resample_op_ =
                    Resample<ResampleType<ResampleScheme, MULTINOMIAL>,
                    size_type, rng_set_type>();
                break;
            case RESIDUAL :
                resample_op_ =
                    Resample<ResampleType<ResampleScheme, RESIDUAL>,
                    size_type, rng_set_type>();
                break;
            case STRATIFIED :
                resample_op_ =
                    Resample<ResampleType<ResampleScheme, STRATIFIED>,
                    size_type, rng_set_type>();
                break;
            case SYSTEMATIC :
                resample_op_ =
                    Resample<ResampleType<ResampleScheme, SYSTEMATIC>,
                    size_type, rng_set_type>();
                break;
            case RESIDUAL_STRATIFIED :
                resample_op_ =
                    Resample<ResampleType<ResampleScheme, RESIDUAL_STRATIFIED>,
                    size_type, rng_set_type>();
                break;
            case RESIDUAL_SYSTEMATIC :
                resample_op_ =
                    Resample<ResampleType<ResampleScheme, RESIDUAL_SYSTEMATIC>,
                    size_type, rng_set_type>();
                break;
            default :
                resample_op_ =
                    Resample<ResampleType<ResampleScheme, STRATIFIED>,
                    size_type, rng_set_type>();
                break;
        }
    }

    /// \brief Replace the resampling operation functor with a new user defined
    /// scheme
    ///
    /// \tparam EnumType The enumeration type of the scheme
    /// \tparam S The name of the scheme
    ///
    /// \details
    /// A partial specialization
    /// of <tt>Resample<ResampleType<EnumType, S>, SizeType, RngSetType></tt>
    /// shall exist
    template <typename EnumType, EnumType S>
    void resample_scheme ()
    {
        resample_scheme<ResampleType<EnumType, S> >();
    }

    /// \brief Replace the resampling operation functor with a new user defined
    /// type
    ///
    /// \tparam ResType The type of the resampling specialization
    ///
    /// \details
    /// A partial specialization
    /// of <tt>Resample<ResType, SizeType, RngSetType></tt> shall exist
    template <typename ResType>
    void resample_scheme ()
    {
        resample_op_ = Resample<ResType, size_type, rng_set_type>();
    }

    private :

    size_type size_;
    value_type value_;

    Eigen::Matrix<size_type, Eigen::Dynamic, 1> replication_;
    Eigen::Matrix<size_type, Eigen::Dynamic, 1> copy_from_;
    Eigen::VectorXd weight_;
    bool resampled_;
    resample_op_type resample_op_;

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
