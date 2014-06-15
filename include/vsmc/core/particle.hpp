#ifndef VSMC_CORE_PARTICLE_HPP
#define VSMC_CORE_PARTICLE_HPP

#include <vsmc/core/single_particle.hpp>
#include <vsmc/core/weight_set.hpp>
#include <vsmc/cxx11/functional.hpp>
#include <vsmc/resample/basic.hpp>
#include <vsmc/rng/rng_set.hpp>
#include <vsmc/rng/seed.hpp>
#include <limits>

namespace vsmc {

/// \brief Particle class representing the whole particle set
/// \ingroup Core
template <typename T>
class Particle
{
    public :

    typedef typename traits::SizeTypeTrait<T>::type size_type;
    typedef T value_type;
    typedef typename traits::WeightSetTypeTrait<T>::type weight_set_type;
    typedef typename traits::RngSetTypeTrait<T>::type rng_set_type;
    typedef typename traits::ResampleRngTypeTrait<T>::type resample_rng_type;
    typedef typename rng_set_type::rng_type rng_type;
    typedef cxx11::function<void (std::size_t, std::size_t,
            resample_rng_type &, const double *, size_type *)> resample_type;

    explicit Particle (size_type N) :
        size_(N), value_(N),
        weight_set_(static_cast<typename
                traits::SizeTypeTrait<weight_set_type>::type>(N)),
        rng_set_(static_cast<typename
                traits::SizeTypeTrait<rng_set_type>::type>(N)),
        resample_replication_(N), resample_copy_from_(N), resample_weight_(N),
        resample_rng_(static_cast<typename resample_rng_type::result_type>(
                    Seed::instance().get())),
        sp_(N + 2, SingleParticle<T>(0, VSMC_NULLPTR)),
        csp_(N + 2, ConstSingleParticle<T>(0, VSMC_NULLPTR))
    {
        weight_set_.set_equal_weight();
        sp_[0] = SingleParticle<T>(std::numeric_limits<size_type>::max
                VSMC_MNE (), this);
        csp_[0] = ConstSingleParticle<T>(std::numeric_limits<size_type>::max
                VSMC_MNE (), this);
        for (size_type i = 1; i != size_ + 2; ++i) {
            sp_[i] = SingleParticle<T>(i - 1, this);
            csp_[i] = ConstSingleParticle<T>(i - 1, this);
        }
    }

    /// \brief Number of particles
    size_type size () const {return size_;}

    /// \brief Read and write access to the value collection object
    value_type &value () {return value_;}

    /// \brief Read only access to the value collection object
    const value_type &value () const {return value_;}

    /// \brief Read and write access to the weight collection object
    weight_set_type &weight_set () {return weight_set_;}

    /// \brief Read only access to the weight collection object
    const weight_set_type &weight_set () const {return weight_set_;}

    /// \brief Read and write access to the RNG collection object
    rng_set_type &rng_set () {return rng_set_;}

    /// \brief Read only access to the RNG collection object
    const rng_set_type &rng_set () const {return rng_set_;}

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS
    /// such that each particle has a equal weight
    void set_equal_weight () {weight_set_.set_equal_weight();}

    /// \brief Read normalized weights through an output iterator
    template <typename OutputIter>
    OutputIter read_weight (OutputIter first) const
    {return weight_set_.read_weight(first);}

    /// \brief Get the ESS of the particle collection based on the current
    /// weights
    double ess () const {return weight_set_.ess();}

    /// \brief Get an (parallel) RNG stream for a given particle
    rng_type &rng (size_type id) {return rng_set_[id];}

    /// \brief Get the (sequential) RNG used stream for resampling
    resample_rng_type &resample_rng () {return resample_rng_;}

    /// \brief Performing resampling if ESS/N < threshold
    ///
    /// \param op The resampling operation funcitor
    /// \param threshold The threshold of ESS/N below which resampling will be
    /// performed
    ///
    /// \return true if resampling was performed
    ///
    /// \details
    /// The out of box behavior are suitable for most applications. However,
    /// the user can also gain full control over the internal of this
    /// function by changing the `weight_set_type` and other type traits for
    /// specific value collection type. The sequence of operations of this
    /// function is listed as in the following pseudo-code
    /// 1. Determine if resampling is required
    ///     * Set `N = weight_set.resample_size()`
    ///     * Set `resampled = weight_set.ess() < threshold * N`
    ///     * If `!resampled`, GO TO Step 8. Otherwise, GO TO Step 2
    /// 2. Determine if resampling algorithms shall be performed. This is only
    /// useful for MPI implementations. the boolean value `resampled` ensures
    /// that collective actions like `copy` will be called by all nodes.
    /// However, the resampling replication numbers can be computed by one
    /// node instead of by all. Therefore, the program can actually collect and
    /// read the resampling weights on one node, and do nothing on other nodes.
    /// This difference shall be reflected by the returning iterator of
    /// `read_resample_weight`
    ///     * (Allocate `double *weight` for size `N`)
    ///     * Set `double *end = weight_set.read_resample_weight(weight)`
    ///     * If `end != weight + N`, GO TO step 5. Otherwise, GO TO Step 3
    /// 3. Performing resampling, produce replication number of each particle
    ///     * (Allocate `size_type *replication` for size `N`)
    ///     * (Create RNG engine `resample_rng` according to
    ///     traits::ResampleRngTypeTrait)
    ///     * Call `op(N, resample_rng, weight, replication)`
    /// 4. Transform replication numbers into parent particle indices
    ///     * (Allocat `size_type *copy_from` for size `N`)
    ///     * (Create functor `r2c` according to
    ///     traits::ResampleCopyFromReplicationTypeTrait)
    ///     * Call `r2c(N, replication, copy_from)`
    /// 5. Set `const size_type *cptr` according to results of Step 2.
    ///     * If Step 3 and 4 are skipped according to Step 2, then set
    ///     `cptr = 0`. Otherwise,
    ///     * Set `cptr = copy_from`
    /// 6. Performing copying of particles
    ///     * Call `value.copy(N, cptr)`
    /// 7. Performing post resampling weight manipulation
    ///     * (Create functor `post` according to
    ///     traits::ResamplePostCopyTypeTrait)
    ///     * `post(weight_set)`
    /// 8. `return resampled`
    bool resample (const resample_type &op, double threshold)
    {
        std::size_t N = static_cast<std::size_t>(weight_set_.resample_size());
        bool resampled = weight_set_.ess() < threshold * N;
        if (resampled) {
            resample_copy_from_.resize(N);
            resample_replication_.resize(N);
            resample_weight_.resize(N);
            double *end = weight_set_.read_resample_weight(
                    &resample_weight_[0]);
            const size_type *cptr = VSMC_NULLPTR;
            if (end == &resample_weight_[0] + N) {
                op(N, N, resample_rng_, &resample_weight_[0],
                        &resample_replication_[0]);
                resample_copy_from_replication_(N, N,
                        &resample_replication_[0], &resample_copy_from_[0]);
                cptr = &resample_copy_from_[0];
            }
            value_.copy(N, cptr);
            resample_post_copy_(weight_set_);
        }

        return resampled;
    }

    private :

    size_type size_;
    value_type value_;
    weight_set_type weight_set_;
    rng_set_type rng_set_;

    std::vector<size_type> resample_replication_;
    std::vector<size_type> resample_copy_from_;
    std::vector<double> resample_weight_;
    typename traits::ResampleCopyFromReplicationTypeTrait<T>::type
        resample_copy_from_replication_;
    typename traits::ResamplePostCopyTypeTrait<T>::type resample_post_copy_;
    resample_rng_type resample_rng_;
    std::vector<SingleParticle<T> > sp_;
    std::vector<ConstSingleParticle<T> > csp_;
}; // class Particle

} // namespace vsmc

#endif // VSMC_CORE_PARTICLE_HPP
