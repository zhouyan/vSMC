//============================================================================
// vSMC/include/vsmc/core/particle.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#ifndef VSMC_CORE_PARTICLE_HPP
#define VSMC_CORE_PARTICLE_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/single_particle.hpp>
#include <vsmc/core/weight_set.hpp>
#include <vsmc/resample/resample.hpp>
#include <vsmc/rng/rng_set.hpp>
#include <vsmc/rng/seed.hpp>
#include <vsmc/utility/aligned_memory.hpp>

namespace vsmc
{

/// \brief Particle class representing the whole particle set
/// \ingroup Core
template <typename T>
class Particle
{
    public:
    typedef typename traits::SizeTypeTrait<T>::type size_type;
    typedef T value_type;
    typedef typename traits::WeightSetTypeTrait<T>::type weight_set_type;
    typedef typename traits::RngSetTypeTrait<T>::type rng_set_type;
    typedef typename traits::ResampleRngTypeTrait<T>::type resample_rng_type;
    typedef typename rng_set_type::rng_type rng_type;
    typedef SingleParticle<T> sp_type;
    typedef ConstSingleParticle<T> csp_type;

    typedef std::function<void(std::size_t, std::size_t, resample_rng_type &,
        const double *, size_type *)> resample_type;

    explicit Particle(size_type N)
        : size_(N)
        , value_(N)
        , weight_set_(static_cast<
              typename traits::SizeTypeTrait<weight_set_type>::type>(N))
        , rng_set_(
              static_cast<typename traits::SizeTypeTrait<rng_set_type>::type>(
                  N))
        , resample_rng_(Seed::instance().get())
    {
    }

    /// \brief Clone the particle system except the RNG engines
    ///
    /// \param new_rng If true, the new particle system has new-seeded RNG.
    /// Otherwise false, it is exactly the same as the current.
    Particle<T> clone(bool new_rng) const
    {
        Particle<T> particle(*this);
        if (new_rng) {
            particle.rng_set().seed();
            particle.resample_rng().seed(Seed::instance().get());
        }

        return particle;
    }

    /// \brief Clone another particle system except the RNG engines
    ///
    /// \param other The particle system to be cloned
    /// \param retain_rng If true, retain the current system's RNG. Otherwise,
    /// it is exactly the same as the new one.
    Particle<T> &clone(const Particle<T> &other, bool retain_rng)
    {
        if (this != &other) {
            if (retain_rng) {
                rng_set_type rset(std::move(rng_set_));
                resample_rng_type rrng(std::move(resample_rng_));
                *this = other;
                rng_set_ = std::move(rset);
                resample_rng_ = std::move(rrng);
                rng_set_.resize(other.size());
            } else {
                *this = other;
            }
        }

        return *this;
    }

    Particle<T> &clone(Particle<T> &&other, bool retain_rng)
    {
        if (this != &other) {
            if (retain_rng) {
                rng_set_type rset(std::move(rng_set_));
                resample_rng_type rrng(std::move(resample_rng_));
                *this = std::move(other);
                rng_set_ = std::move(rset);
                resample_rng_ = std::move(rrng);
                rng_set_.resize(other.size());
            } else {
                *this = std::move(other);
            }
        }

        return *this;
    }

    /// \brief Number of particles
    size_type size() const { return size_; }

    /// \brief Read and write access to the value collection object
    value_type &value() { return value_; }

    /// \brief Read only access to the value collection object
    const value_type &value() const { return value_; }

    /// \brief Read and write access to the weight collection object
    weight_set_type &weight_set() { return weight_set_; }

    /// \brief Read only access to the weight collection object
    const weight_set_type &weight_set() const { return weight_set_; }

    /// \brief Read and write access to the RNG collection object
    rng_set_type &rng_set() { return rng_set_; }

    /// \brief Read only access to the RNG collection object
    const rng_set_type &rng_set() const { return rng_set_; }

    /// \brief Get an (parallel) RNG stream for a given particle
    rng_type &rng(size_type id) { return rng_set_[id]; }

    /// \brief Get a SingleParticle
    sp_type sp(size_type id) { return sp_type(id, this); }

    /// \brief Get a ConstSingleParticle
    csp_type sp(size_type id) const { return csp_type(id, this); }

    /// \brief Get a ConstSingleParticle
    csp_type csp(size_type id) { return csp_type(id, this); }

    /// \brief Get a ConstSingleParticle
    csp_type csp(size_type id) const { return csp_type(id, this); }

    /// \brief Get the (sequential) RNG used stream for resampling
    resample_rng_type &resample_rng() { return resample_rng_; }

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
    /// node instead of by all. Therefore, the program can actually collect
    /// and
    /// read the resampling weights on one node, and do nothing on other
    /// nodes.
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
    /// 5. Set `const size_type *cptr` according to results of Step 2.
    ///     * If Step 3 and 4 are skipped according to Step 2, then set
    ///     `cptr = 0`. Otherwise,
    ///     * Set `cptr = copy_from`
    /// 6. Performing copying of particles
    ///     * Call `value.copy(N, cptr)`
    /// 7. `return resampled`
    bool resample(const resample_type &op, double threshold)
    {
        std::size_t N = static_cast<std::size_t>(weight_set_.resample_size());
        bool resampled = weight_set_.ess() < threshold * N;
        if (resampled) {
            const double *const rwptr = weight_set_.resample_weight_data();
            if (rwptr != nullptr) {
                copy_from_.resize(N);
                op(N, N, resample_rng_, rwptr, copy_from_.data());
                value_.copy(N, copy_from_.data());
            } else {
                value_.copy(N, static_cast<const std::size_t *>(nullptr));
            }
            weight_set_.set_equal_weight();
        }

        return resampled;
    }

    private:
    size_type size_;
    value_type value_;
    weight_set_type weight_set_;
    rng_set_type rng_set_;
    resample_rng_type resample_rng_;

    std::vector<size_type, AlignedAllocator<size_type>> copy_from_;
}; // class Particle

} // namespace vsmc

#endif // VSMC_CORE_PARTICLE_HPP
