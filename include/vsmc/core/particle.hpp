#ifndef VSMC_CORE_PARTICLE_HPP
#define VSMC_CORE_PARTICLE_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/resample.hpp>
#include <vsmc/core/rng.hpp>
#include <vsmc/core/weight.hpp>

namespace vsmc {

/// \brief Particle class representing the whole particle set
/// \ingroup Core
/// \sa WeightSetBase
/// \sa RngSetSeq RngSetPrl
template <typename T>
class Particle
{
    public :

    typedef typename traits::SizeTypeTrait<T>::type size_type;
    typedef T value_type;
    typedef typename traits::WeightSetTypeTrait<T>::type
        weight_set_type;
    typedef typename traits::RngSetTypeTrait<T>::type
        rng_set_type;
    typedef typename rng_set_type::rng_type rng_type;
    typedef typename traits::ResampleRngSetTypeTrait<T>::type
        resample_rng_set_type;
    typedef cxx11::function<
        void (size_type, resample_rng_set_type &, double *, size_type *)>
        resample_op_type;

    explicit Particle (size_type N) :
        size_(N), value_(N), weight_set_(N), rng_set_(N),
        replication_(N), copy_from_(N), weight_(N), resampled_(false),
        resample_rng_set_(N)
    {
        weight_set_value_ptr(
                typename has_set_value_ptr_<weight_set_type>::value_type());
        rng_set_value_ptr(
                typename has_set_value_ptr_<rng_set_type>::value_type());
        weight_set_.set_equal_weight();
    }

    /// \brief Number of particles
    size_type size () const
    {
        return size_;
    }

    /// \brief Read and write access to the value collection object
    value_type &value ()
    {
        return value_;
    }

    /// \brief Read only access to the value collection object
    const value_type &value () const
    {
        return value_;
    }

    /// \brief Read and write access to the weight collection object
    weight_set_type &weight_set ()
    {
        return weight_set_;
    }

    /// \brief Read only access to the weight collection object
    const weight_set_type &weight_set () const
    {
        return weight_set_;
    }

    /// \brief Read and write access to the RNG collection object
    rng_set_type &rng_set ()
    {
        return rng_set_;
    }

    /// \brief Read only access to the RNG collection object
    const rng_set_type &rng_set () const
    {
        return rng_set_;
    }

    /// \brief Read normalized weights through an output iterator
    template <typename OutputIter>
    OutputIter read_weight (OutputIter first) const
    {
        return weight_set_.read_weight(first);
    }

    /// \brief Read normalized weights through a random access iterator with
    /// (possible non-uniform stride)
    template <typename RandomIter>
    RandomIter read_weight (RandomIter first, int stride) const
    {
        return weight_set_.read_weight(first, stride);
    }

    /// \brief Read normalized weights through a pointer
    double *read_weight (double *first) const
    {
        return weight_set_.read_weight(first);
    }

    /// \brief Read unnormalized logarithm weights through an output iterator
    template <typename OutputIter>
    OutputIter read_log_weight (OutputIter first) const
    {
        return weight_set_.read_log_weight(first);
    }

    /// \brief Read unnormalized logarithm weights through a random access
    /// iterator with (possible non-uniform stride)
    template <typename RandomIter>
    RandomIter read_log_weight (RandomIter first, int stride) const
    {
        return weight_set_.read_log_weight(first, stride);
    }

    /// \brief Read unnormalized logarithm weights through a pointer
    double *read_log_weight (double *first) const
    {
        return weight_set_.read_log_weight(first);
    }

    /// \brief Get the normalized weight of the id'th particle
    double weight (size_type id) const
    {
        return weight_set_.weight(id);
    }

    /// \brief Get the unnormalized logarithm weight of the id'th particle
    double log_weight (size_type id) const
    {
        return weight_set_.log_weight(id);
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS
    /// such that each particle has a equal weight
    void set_equal_weight ()
    {
        weight_set_.set_equal_weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) weights directly through an input
    /// iterator
    template <typename InputIter>
    void set_weight (InputIter first)
    {
        weight_set_.set_weight(first);
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) weights directly through a random
    /// access iterator with (possible non-uniform) stride
    template <typename RandomIter>
    void set_weight (RandomIter first, int stride)
    {
        weight_set_.set_weight(first, stride);
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) weights directly through a pointer
    void set_weight (const double *first)
    {
        weight_set_.set_weight(first);
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// multiply the normalized weight with (possible unnormalized) incremental
    /// weights through an input iterator
    template <typename InputIter>
    void mul_weight (InputIter first)
    {
        weight_set_.mul_weight(first);
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// multiply the normalized weight with (possible unnormalized) incremental
    /// weights through a random access iterator with (possible non-uniform)
    /// stride
    template <typename RandomIter>
    void mul_weight (RandomIter first, int stride)
    {
        weight_set_.mul_weight(first, stride);
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) logarithm weights directly through
    /// an input iterator
    template <typename InputIter>
    void set_log_weight (InputIter first)
    {
        weight_set_.set_log_weight(first);
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) logarithm weights directly through
    /// a random access iterator with (possible non-uniform) stride
    template <typename RandomIter>
    void set_log_weight (RandomIter first, int stride)
    {
        weight_set_.set_log_weight(first, stride);
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) logarithm weights directly through
    /// a pointer
    void set_log_weight (const double *first)
    {
        weight_set_.set_log_weight(first);
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// adding to the unnormalized logarithm weights with (possible
    /// unormalized) logarithm incremental weights through an input iterator
    template <typename InputIter>
    void add_log_weight (InputIter first)
    {
        weight_set_.add_log_weight(first);
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// adding to the unnormalized logarithm weights with (possible
    /// unormalized) logarithm incremental weights through a ranodm access
    /// iterator with (possible non-uniform) stride
    template <typename RandomIter>
    void add_log_weight (RandomIter first, int stride)
    {
        weight_set_.add_log_weight(first, stride);
    }

    /// \brief Get the ESS of the particle collection based on the current
    /// weights
    double ess () const
    {
        return weight_set_.ess();
    }

    /// \brief Get an RNG stream for a given particle
    rng_type &rng (size_type id)
    {
        return rng_set_.rng(id);
    }

    /// \brief Performing resampling if ESS/N < threshold
    ///
    /// \param threshold The threshold of ESS/N below which resampling will be
    /// performed
    ///
    /// \return true if resampling was performed
    bool resample (double threshold)
    {
        resampled_ = weight_set_.ess() < threshold * size_;
        if (resampled_) {
            weight_set_.read_weight(&weight_[0]);
            resample_op_(size_, resample_rng_set_,
                    &weight_[0], &replication_[0]);
            resample_do();
        }

        return resampled_;
    }

    /// \brief Whether last attempt of resample is actually performed
    bool resampled () const
    {
        return resampled_;
    }

    /// \brief Set resampling method by a resample_op_type object
    void resample_scheme (const resample_op_type &res_op)
    {
        resample_op_ = res_op;
    }

    /// \brief Set resampling method by a built-in scheme name
    ///
    /// \param scheme A ResampleScheme scheme name
    ///
    /// \return  true if scheme is valid and the resampling scheme is actually
    /// changed, false otherwise
    bool resample_scheme (ResampleScheme scheme)
    {
        switch (scheme) {
            case Multinomial :
                resample_op_ =
                    Resample<ResampleType<ResampleScheme, Multinomial>,
                    size_type, resample_rng_set_type>();
                break;
            case Residual :
                resample_op_ =
                    Resample<ResampleType<ResampleScheme, Residual>,
                    size_type, resample_rng_set_type>();
                break;
            case Stratified :
                resample_op_ =
                    Resample<ResampleType<ResampleScheme, Stratified>,
                    size_type, resample_rng_set_type>();
                break;
            case Systematic :
                resample_op_ =
                    Resample<ResampleType<ResampleScheme, Systematic>,
                    size_type, resample_rng_set_type>();
                break;
            case ResidualStratified :
                resample_op_ =
                    Resample<ResampleType<ResampleScheme, ResidualStratified>,
                    size_type, resample_rng_set_type>();
                break;
            case ResidualSystematic :
                resample_op_ =
                    Resample<ResampleType<ResampleScheme, ResidualSystematic>,
                    size_type, resample_rng_set_type>();
                break;
            default :
                return false;
                break;
        }

        return true;
    }

    /// \brief Set resampling method by a scheme name from a collection
    ///
    /// \details
    /// An object of type Resample<ResampleType<EnumType, S>, size_type,
    /// resample_rng_set_type> will constructed as the resampling method. This
    /// can be a user defined partial specializing of Resample clas template
    ///
    /// For example, resample_scheme<ResampleScheme, Stratified>() is
    /// equivalent to resample_scheme(Stratified)
    template <typename EnumType, EnumType S>
    void resample_scheme ()
    {
        resample_scheme<ResampleType<EnumType, S> >();
    }

    /// \brief Set resampling method by the type of resampling object
    ///
    /// \details
    /// An object of type Resample<ResType, size_type, resample_rng_set_type>,
    /// will constructed as the resampling method. This can be a user defined
    /// partial specializing of Resample class template
    template <typename ResType>
    void resample_scheme ()
    {
        resample_op_ = Resample<ResType, size_type, resample_rng_set_type>();
    }

    private :

    size_type size_;
    value_type value_;
    weight_set_type weight_set_;
    rng_set_type rng_set_;

    std::vector<size_type> replication_;
    std::vector<size_type> copy_from_;
    std::vector<double> weight_;
    bool resampled_;
    resample_op_type resample_op_;
    resample_rng_set_type resample_rng_set_;

    void resample_do ()
    {
        size_type sum = std::accumulate(
                replication_.begin(), replication_.end(),
                static_cast<size_type>(0));
        if (sum != size_) {
            typename std::vector<size_type>::iterator id_max =
                std::max_element(replication_.begin(), replication_.end());
            *id_max += size_ - sum;
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

        value_.copy(&copy_from_[0]);
        weight_set_.set_equal_weight();
    }
}; // class Particle

} // namespace vsmc

#endif // VSMC_CORE_PARTICLE_HPP
