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
#include <vsmc/resample/algorithm.hpp>
#include <vsmc/rng/rng_set.hpp>
#include <vsmc/rng/seed.hpp>

#define VSMC_RUNTIME_ASSERT_CORE_PARTICLE_RESIZE_BY_RANGE                     \
    VSMC_RUNTIME_ASSERT(                                                      \
        (!internal::is_negative(first) && first < last && last <= size_),     \
        "**Particle::resize_by_range** INDICES OUT OF RANGE")

namespace vsmc
{

/// \brief A subset of particles
/// \ingroup Core
template <typename T>
class ParticleRange
{
    public:
    ParticleRange(typename Particle<T>::size_type begin,
        typename Particle<T>::size_type end, Particle<T> *pptr)
        : pptr_(pptr), begin_(begin), end_(end)
    {
    }

    Particle<T> &particle() const { return *pptr_; }

    Particle<T> *particle_ptr() const { return pptr_; }

    typename Particle<T>::size_type size() const { return end_ - begin_; }

    typename Particle<T>::size_type begin() const { return begin_; }

    typename Particle<T>::size_type end() const { return end_; }

    typename Particle<T>::rng_type &rng() const { return pptr_->rng(begin_); }

    private:
    Particle<T> *pptr_;
    typename Particle<T>::size_type begin_;
    typename Particle<T>::size_type end_;
}; // class ParticleRange

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
    using sp_type = SingleParticle<T>;
    using range_type = ParticleRange<T>;

    Particle() : size_(0), value_(0), weight_(0), rng_set_(0)
    {
        Seed::instance()(rng_);
    }

    template <typename... Args>
    explicit Particle(size_type N, Args &&... args)
        : size_(N)
        , value_(N, std::forward<Args>(args)...)
        , weight_(static_cast<SizeType<weight_type>>(N))
        , rng_set_(static_cast<SizeType<rng_set_type>>(N))
    {
        Seed::instance()(rng_);
    }

    /// \brief Clone the particle system except the RNG engines
    ///
    /// \param new_rng If true, the new particle system has new-seeded RNG.
    /// Otherwise false, it is exactly the same as the current.
    Particle<T> clone(bool new_rng) const
    {
        Particle<T> particle(*this);
        if (new_rng) {
            particle.rng_set_.seed();
            Seed::instance()(particle.rng_);
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

    /// \brief Resize by selecting according to user supplied index vector
    ///
    /// \param N The new sample size
    /// \param index N-vector of parent indices
    template <typename InputIter>
    void resize_by_index(size_type N, InputIter index)
    {
        resize_by_index(
            N, index, std::is_convertible<InputIter, const size_type *>());
    }

    /// \brief Resize by selecting according to user supplied mask vector
    ///
    /// \param N The new sample size
    /// \param mask A mask vector of length at least `size()`. Each element,
    ///
    /// \details
    /// when converted to `bool`, if `true`, the corresponding particle is
    /// preserved. The vector of preserved particles is copied repeatedly until
    /// there are enough particles to fill the new system.
    template <typename InputIter>
    void resize_by_mask(size_type N, InputIter mask)
    {
        Vector<size_type> idx(static_cast<std::size_t>(N));
        Vector<size_type> mask_index(static_cast<std::size_t>(size_));
        std::size_t M = 0;
        for (size_type i = 0; i != size_; ++i, ++mask)
            if (*mask)
                mask_index[M++] = i;
        resize_copy_index(
            static_cast<std::size_t>(N), M, mask_index.data(), idx.data());
        resize(N, idx.data());
    }

    /// \brief Resize by resampling
    ///
    /// \param N The new sample size
    /// \param op The resampling function object
    ///
    /// \details
    /// The particle system is resampled to a new system with possibly
    /// different size. The system will be changed even if `N == size()`.
    template <typename ResampleType>
    void resize_by_resample(size_type N, ResampleType &&op)
    {
        Vector<size_type> rep(static_cast<std::size_t>(size_));
        Vector<size_type> idx(static_cast<std::size_t>(N));
        op(static_cast<std::size_t>(size_), static_cast<std::size_t>(N), rng_,
            weight_.data(), rep.data());
        resample_trans_rep_index(static_cast<std::size_t>(size_),
            static_cast<std::size_t>(N), rep.data(), idx.data());
        resize(N, idx.data());
    }

    /// \brief Resize by uniformly selecting from all particles
    ///
    /// \param N The new sample size
    ///
    /// \details
    /// This is equivalent to first set the weights to equal, and then perform
    /// Multinomial resampling. The particle system will be changed even if
    /// `N == size()`.
    void resize_by_uniform(size_type N)
    {
        weight_.set_equal();
        resize_by_resample(N, ResampleMultinomial());
    }

    /// \brief Equivalent to `resize_by_range(N, 0, size());
    void resize_by_range(size_type N) { resize_by_range(N, 0, size_); }

    /// \brief Equivalent to `resize_by_range(N, first, size());
    void resize_by_range(size_type N, size_type first)
    {
        resize_by_range(N, first, size_);
    }

    /// \brief Resize by selecting a range of particles
    ///
    /// \param N The new sample size
    /// \param first The index of the first particle to be preserved
    /// \param last The index past the last particle to be preserved
    ///
    /// \details
    /// This is the most destructive resizing method. The particles in the
    /// range [first, last) are copied repeatedly until there are enough
    /// particles to fill the new system.
    void resize_by_range(size_type N, size_type first, size_type last)
    {
        VSMC_RUNTIME_ASSERT_CORE_PARTICLE_RESIZE_BY_RANGE;

        if (N == size_ && first == 0)
            return;

        Vector<size_type> idx(static_cast<std::size_t>(N));
        size_type M = last - first;
        if (M >= N) {
            for (size_type i = 0; i != N; ++i, ++first)
                idx[static_cast<std::size_t>(i)] = first;
        } else {
            for (size_type i = 0; i != M; ++i, ++first)
                idx[static_cast<std::size_t>(i)] = first;
            resize_copy_index(static_cast<std::size_t>(N - M),
                static_cast<std::size_t>(M), idx.data(), idx.data() + M);
        }
        resize(N, idx.data());
    }

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
    rng_type &rng(size_type id)
    {
        return rng_set_[static_cast<std::size_t>(id)];
    }

    /// \brief Get an (parallel) RNG stream for a given particle
    const rng_type &rng(size_type id) const
    {
        return rng_set_[static_cast<std::size_t>(id)];
    }

    /// \brief Get the (sequential) RNG used stream for resampling
    rng_type &rng() { return rng_; }

    /// \brief Get the (sequential) RNG used stream for resampling
    const rng_type &rng() const { return rng_; }

    /// \brief Get a SingleParticle<T> object
    sp_type sp(size_type id) { return SingleParticle<T>(id, this); }

    /// \brief Get a ParticleRange<T> object
    range_type range(size_type begin, size_type end)
    {
        return ParticleRange<T>(begin, end, this);
    }

    /// \brief Get a ParticleRange<T> object with `begin == 0`, `end == size()`
    range_type range() { return ParticleRange<T>(0, size(), this); }

    /// \brief Get a SingleParticle<T> object for the first particle
    sp_type begin() { return sp(0); }

    /// \brief Get a SingleParticle<T> object for the first particle
    sp_type end() { return sp(size_); }

    private:
    static constexpr std::size_t M_ = internal::BufferSize<size_type>::value;

    size_type size_;
    value_type value_;
    weight_type weight_;
    rng_set_type rng_set_;
    rng_type rng_;

    void resize(size_type N, const size_type *idx)
    {
        size_ = N;
        value_.copy(N, idx);
        weight_.resize(static_cast<SizeType<weight_type>>(N));
        rng_set_.resize(static_cast<SizeType<rng_set_type>>(N));
    }

    template <typename InputIter>
    void resize_by_index(size_type N, InputIter index, std::true_type)
    {
        resize(N, static_cast<const size_type *>(index));
    }

    template <typename InputIter>
    void resize_by_index(size_type N, InputIter index, std::false_type)
    {
        Vector<size_type> idx(static_cast<std::size_t>(N));
        std::copy_n(index, N, idx.data());
        resize(N, idx.data());
    }

    template <typename InputIter, typename OutputIter>
    void resize_copy_index(
        std::size_t N, std::size_t M, InputIter src, OutputIter dst)
    {
        while (N > M) {
            dst = std::copy_n(src, M, dst);
            N -= M;
        }
        std::copy_n(src, N, dst);
    }
}; // class Particle

} // namespace vsmc

#endif // VSMC_CORE_PARTICLE_HPP
