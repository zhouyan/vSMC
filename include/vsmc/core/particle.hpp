#ifndef VSMC_CORE_PARTICLE_HPP
#define VSMC_CORE_PARTICLE_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/resample.hpp>
#include <vsmc/core/single_particle.hpp>
#include <vsmc/core/weight.hpp>
#include <vsmc/utility/rng_set.hpp>
#include <vsmc/utility/seed.hpp>

namespace vsmc {

/// \brief Particle class representing the whole particle set
/// \ingroup Core
/// \sa WeightSetBase
/// \sa RngSet
template <typename T>
class Particle
{
    public :

    typedef typename traits::SizeTypeTrait<T>::type size_type;
    typedef T value_type;
    typedef typename traits::WeightSetTypeTrait<T>::type weight_set_type;
    typedef typename traits::RngSetTypeTrait<T>::type rng_set_type;
    typedef typename rng_set_type::rng_type rng_type;
    typedef typename traits::ResampleRngTypeTrait<T>::type resample_rng_type;
    typedef cxx11::function<
        void (size_type, resample_rng_type &, double *, size_type *)>
        resample_type;
    typedef ParticleIterator<T, SingleParticle> iterator;
    typedef ParticleIterator<T, ConstSingleParticle> const_iterator;
    typedef std::reverse_iterator<iterator> reverse_iterator;
    typedef std::reverse_iterator<const_iterator> const_reverse_iterator;

    explicit Particle (size_type N) :
        size_(N), value_(N), weight_set_(N), rng_set_(N),
        replication_(N), copy_from_(N), weight_(N),
        resample_rng_(VSMC_SEED_TYPE::instance().get()),
        sp_(N + 2, SingleParticle<T>(0, VSMC_NULLPTR)),
        csp_(N + 2, ConstSingleParticle<T>(0, VSMC_NULLPTR))
    {
        weight_set_.set_equal_weight();
        sp_[0] = SingleParticle<T>(
                std::numeric_limits<size_type>::max
                VSMC_MACRO_NO_EXPANSION (), this);
        csp_[0] = ConstSingleParticle<T>(
                std::numeric_limits<size_type>::max
                VSMC_MACRO_NO_EXPANSION (), this);
        for (size_type i = 1; i != size_ + 2; ++i) {
            sp_[i] = SingleParticle<T>(i - 1, this);
            csp_[i] = ConstSingleParticle<T>(i - 1, this);
        }
    }

    /// \brief Number of particles
    size_type size () const
    {
        return size_;
    }

    const SingleParticle<T> &sp (size_type id)
    {
        return sp_[id + 1];
    }

    const ConstSingleParticle<T> &sp (size_type id) const
    {
        return csp_[id + 1];
    }

    iterator begin()
    {
        return iterator(&sp(0));
    }

    iterator end()
    {
        return iterator(&sp(size_));
    }

    const_iterator begin() const
    {
        return cosnt_iterator(&sp(0));
    }

    const_iterator end() const
    {
        return const_iterator(&sp(size_));
    }

    const_iterator cbegin() const
    {
        return const_iterator(&sp(0));
    }

    const_iterator cend() const
    {
        return const_iterator(&sp(size_));
    }

    reverse_iterator rbegin()
    {
        return reverse_iterator(end());
    }

    reverse_iterator rend()
    {
        return reverse_iterator(begin());
    }

    const_reverse_iterator rbegin() const
    {
        return const_reverse_iterator(end());
    }

    const_reverse_iterator rend() const
    {
        return const_reverse_iterator(begin());
    }

    const_reverse_iterator crbegin() const
    {
        return const_reverse_iterator(cend());
    }

    const_reverse_iterator crend() const
    {
        return const_reverse_iterator(cbegin());
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

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS
    /// such that each particle has a equal weight
    void set_equal_weight ()
    {
        weight_set_.set_equal_weight();
    }

    /// \brief Read normalized weights through an output iterator
    template <typename OutputIter>
    OutputIter read_weight (OutputIter first) const
    {
        return weight_set_.read_weight(first);
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
    /// \param op The resampling operation funcitor
    /// \param threshold The threshold of ESS/N below which resampling will be
    /// performed
    ///
    /// \return true if resampling was performed
    bool resample (const resample_type &op, double threshold)
    {
        bool resampled = weight_set_.ess() < threshold * size_;
        size_type N = weight_set_.resample_size();
        resampled = resampled && N > 0;
        if (resampled) {
            weight_.resize(N);
            replication_.resize(N);
            weight_set_.read_resample_weight(&weight_[0]);
            op(size_, resample_rng_,
                    &weight_[0], &replication_[0]);
            replication2copy_from(N);
            value_.copy(N, &copy_from_[0]);
            weight_set_.set_equal_weight();
        }

        return resampled;
    }

    private :

    size_type size_;
    value_type value_;
    weight_set_type weight_set_;
    rng_set_type rng_set_;

    std::vector<size_type> replication_;
    std::vector<size_type> copy_from_;
    std::vector<double> weight_;
    resample_rng_type resample_rng_;
    std::vector<SingleParticle<T> > sp_;
    std::vector<ConstSingleParticle<T> > csp_;

    void replication2copy_from (size_type N)
    {
        copy_from_.resize(N);

        size_type from = 0;
        size_type time = 0;
        for (size_type to = 0; to != N; ++to) {
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
    }
}; // class Particle

} // namespace vsmc

#endif // VSMC_CORE_PARTICLE_HPP
