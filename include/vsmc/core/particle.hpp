//============================================================================
// vSMC/include/vsmc/core/particle.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2016, Yan Zhou
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
#include <vsmc/core/weight.hpp>
#include <vsmc/resample/resample.hpp>
#include <vsmc/rng/rng_set.hpp>
#include <vsmc/rng/seed.hpp>

namespace vsmc
{

/// \brief Particle class representing the whole particle set
/// \ingroup Core
template <typename T>
class Particle
{
    public:
    using size_type = SizeType<T>;
    using value_type = T;
    using weight_type = WeightType<T>;
    using rng_set_type = RNGSetType<T>;
    using rng_type = typename rng_set_type::rng_type;
    using resample_type = std::function<void(
        std::size_t, std::size_t, rng_type &, const double *, size_type *)>;

    explicit Particle(size_type N)
        : size_(N)
        , value_(N)
        , weight_(static_cast<SizeType<weight_type>>(N))
        , rng_set_(static_cast<SizeType<rng_set_type>>(N))
    {
        Seed::instance().seed_rng(rng_);
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
            Seed::instance().seed_rng(particle.rng());
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
            size_ = other.size_;
            value_ = other.value_;
            weight_ = other.weight_;

            if (!retain_rng) {
                rng_set_ = other.rng_set_;
                rng_ = other.rng_;
            }
        }

        return *this;
    }

    Particle<T> &clone(Particle<T> &&other, bool retain_rng)
    {
        if (this != &other) {
            size_ = other.size_;
            value_ = std::move(other.value_);
            weight_ = std::move(other.weight_);

            if (!retain_rng) {
                rng_set_ = other.rng_set_;
                rng_ = other.rng_;
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
    weight_type &weight() { return weight_; }

    /// \brief Read only access to the weight collection object
    const weight_type &weight() const { return weight_; }

    /// \brief Read and write access to the RNG collection object
    rng_set_type &rng_set() { return rng_set_; }

    /// \brief Read only access to the RNG collection object
    const rng_set_type &rng_set() const { return rng_set_; }

    /// \brief Get an (parallel) RNG stream for a given particle
    rng_type &rng(size_type id) { return rng_set_[id]; }

    /// \brief Get the (sequential) RNG used stream for resampling
    rng_type &rng() { return rng_; }

    /// \brief Get a SingleParticle object
    SingleParticle<T> sp(size_type id) { return SingleParticle<T>(id, this); }

    /// \brief Performing resampling if ESS/N < threshold
    ///
    /// \param op The resampling operation funcitor
    /// \param threshold The threshold of ESS/N below which resampling will be
    /// performed
    ///
    /// \return true if resampling was performed
    bool resample(const resample_type &op, double threshold)
    {
        std::size_t N = static_cast<std::size_t>(weight_.resample_size());
        bool resampled = weight_.ess() < threshold * N;
        if (resampled) {
            const double *const rwptr = weight_.resample_data();
            if (rwptr != nullptr) {
#if VSMC_USE_TBB
                Vector<size_type> &rep = rep_.local();
                Vector<size_type> &idx = idx_.local();
                rep.resize(N);
                idx.resize(N);
#else  // VSMC_USE_TBB
                Vector<size_type> rep(N);
                Vector<size_type> idx(N);
#endif // VSMC_USE_TBB

                op(N, N, rng_, rwptr, rep.data());
                resample_trans_rep_index(N, N, rep.data(), idx.data());
                value_.copy(N, idx.data());
            } else {
                value_.copy(N, static_cast<const size_type *>(nullptr));
            }
            weight_.set_equal();
        }

        return resampled;
    }

    private:
    size_type size_;
    value_type value_;
    weight_type weight_;
    rng_set_type rng_set_;
    rng_type rng_;
#if VSMC_USE_TBB
    ::tbb::combinable<Vector<size_type>> rep_;
    ::tbb::combinable<Vector<size_type>> idx_;
#endif
}; // class Particle

} // namespace vsmc

#endif // VSMC_CORE_PARTICLE_HPP
