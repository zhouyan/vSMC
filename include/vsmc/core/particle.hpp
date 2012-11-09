#ifndef VSMC_CORE_PARTICLE_HPP
#define VSMC_CORE_PARTICLE_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/resample.hpp>
#include <vsmc/core/rng.hpp>
#include <vsmc/core/weight.hpp>

namespace vsmc {

/// \brief Particle class representing the whole particle set
/// \ingroup Core
template <typename T>
class Particle :
    public RngSetTypeTrait<T>::type,
    public WeightSetTypeTrait<T>::type
{
    public :

    typedef typename SizeTypeTrait<T>::type size_type;
    typedef T value_type;
    typedef typename RngSetTypeTrait<T>::type rng_set_type;
    typedef typename ResampleRngSetTypeTrait<T>::type resample_rng_set_type;
    typedef typename rng_set_type::rng_type rng_type;
    typedef typename WeightSetTypeTrait<T>::type weight_set_type;
    typedef cxx11::function<void (size_type, resample_rng_set_type &,
            double *, size_type *)> resample_op_type;

    using rng_set_type::rng;
    using weight_set_type::weight;
    using weight_set_type::log_weight;
    using weight_set_type::read_weight;
    using weight_set_type::read_log_weight;
    using weight_set_type::set_equal_weight;
    using weight_set_type::set_weight;
    using weight_set_type::mul_weight;
    using weight_set_type::set_log_weight;
    using weight_set_type::add_log_weight;
    using weight_set_type::ess;

    explicit Particle (size_type N) :
        rng_set_type(N), weight_set_type(N), size_(N), value_(N),
        replication_(N), copy_from_(N), weight_(N), resampled_(false),
        resample_rng_set_(N)
    {
        set_equal_weight();
    }

    size_type size () const
    {
        return size_;
    }

    value_type &value ()
    {
        return value_;
    }

    const value_type &value () const
    {
        return value_;
    }

    void resample (double threshold)
    {
        resample(resample_op_, threshold);
    }

    void resample (resample_op_type &res_op, double threshold)
    {
        resampled_ = ess() < threshold * size_;
        if (resampled_) {
            read_weight(&weight_[0]);
            resample_op_(size_, resample_rng_set_,
                    &weight_[0], &replication_[0]);
            resample_do();
        }
    }

    void resample (const resample_op_type &res_op, double threshold)
    {
        resample_scheme(res_op);
        resample(threshold);
    }

    void resample (ResampleScheme scheme, double threshold)
    {
        resample_scheme(scheme);
        resample(threshold);
    }

    bool resampled () const
    {
        return resampled_;
    }

    void resample_scheme (const resample_op_type &res_op)
    {
        resample_op_ = res_op;
    }

    void resample_scheme (ResampleScheme scheme)
    {
        switch (scheme) {
            case MULTINOMIAL :
                resample_op_ =
                    Resample<ResampleType<ResampleScheme, MULTINOMIAL>,
                    size_type, resample_rng_set_type>();
                break;
            case RESIDUAL :
                resample_op_ =
                    Resample<ResampleType<ResampleScheme, RESIDUAL>,
                    size_type, resample_rng_set_type>();
                break;
            case STRATIFIED :
                resample_op_ =
                    Resample<ResampleType<ResampleScheme, STRATIFIED>,
                    size_type, resample_rng_set_type>();
                break;
            case SYSTEMATIC :
                resample_op_ =
                    Resample<ResampleType<ResampleScheme, SYSTEMATIC>,
                    size_type, resample_rng_set_type>();
                break;
            case RESIDUAL_STRATIFIED :
                resample_op_ =
                    Resample<ResampleType<ResampleScheme, RESIDUAL_STRATIFIED>,
                    size_type, resample_rng_set_type>();
                break;
            case RESIDUAL_SYSTEMATIC :
                resample_op_ =
                    Resample<ResampleType<ResampleScheme, RESIDUAL_SYSTEMATIC>,
                    size_type, resample_rng_set_type>();
                break;
            default :
                resample_op_ =
                    Resample<ResampleType<ResampleScheme, STRATIFIED>,
                    size_type, resample_rng_set_type>();
                break;
        }
    }

    template <typename EnumType, EnumType S>
    void resample_scheme ()
    {
        resample_scheme<ResampleType<EnumType, S> >();
    }

    template <typename ResType>
    void resample_scheme ()
    {
        resample_op_ = Resample<ResType, size_type, resample_rng_set_type>();
    }

    private :

    size_type size_;
    value_type value_;

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
        set_equal_weight();
    }
}; // class Particle

} // namespace vsmc

#endif // VSMC_CORE_PARTICLE_HPP
