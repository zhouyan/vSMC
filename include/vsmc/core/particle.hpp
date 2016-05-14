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
#include <vsmc/core/weight.hpp>
#include <vsmc/resample/algorithm.hpp>
#include <vsmc/rng/rng_set.hpp>
#include <vsmc/rng/seed.hpp>

#define VSMC_RUNTIME_ASSERT_CORE_PARTICLE_INDEX_COMPARE(idx1, idx2)           \
    VSMC_RUNTIME_ASSERT((idx1.particle_ptr() == idx2.particle_ptr()),         \
        "COMPARE TWO ParticleIndex OBJECTS THAT BELONG TO TWO PARTICLE "      \
        "SYSTEMS");

#define VSMC_RUNTIME_ASSERT_CORE_PARTICLE_INDEX_DIFFERENCE(idx1, idx2)        \
    VSMC_RUNTIME_ASSERT((idx1.particle_ptr() == idx2.particle_ptr()),         \
        "SUBSTRACT TWO ParticleIndex OBJECTS THAT BELONG TO TWO PARTICLE "    \
        "SYSTEMS");

#define VSMC_RUNTIME_ASSERT_CORE_PARTICLE_RANGE(n, b, e, g)                   \
    VSMC_RUNTIME_ASSERT(                                                      \
        (!internal::is_negative(b) && b < e && e <= n && g > 0),              \
        "**ParticleRange::ParticleRange** INVALID ARGUMENTS")

namespace vsmc
{

/// \brief A thin wrapper over a complete Particle
/// \ingroup Core
template <typename T>
class ParticleIndexBase
{
    public:
    using particle_type = Particle<T>;
    using size_type = typename particle_type::size_type;
    using rng_type = typename particle_type::rng_type;

    ParticleIndexBase(size_type id, particle_type *pptr) : pptr_(pptr), id_(id)
    {
    }

    particle_type &particle() const { return *pptr_; }

    particle_type *particle_ptr() const { return pptr_; }

    size_type id() const { return id_; }

    rng_type &rng() const { return pptr_->rng(id_); }

    private:
    particle_type *pptr_;
    size_type id_;
}; // class ParticleIndexBase

/// \brief ParticleIndex base class trait
/// \ingroup Traits
VSMC_DEFINE_TYPE_TEMPLATE_DISPATCH_TRAIT(
    ParticleIndexBaseType, particle_index_type, ParticleIndexBase)

/// \brief A thin wrapper over a complete Particle
/// \ingroup Core
///
/// \details
/// This is the basic ParticleIndex available for any type of Particle. To
/// extend it for type `T`. One can either specialize
/// vsmc::ParticleIndexBaseTypeTrait<T> or define a class template named
/// `particle_index_type` within `T` with the following minimum requirement.
/// ~~~{.cpp}
/// template <typename S> // S: StateType, such as StateMatrix<Dim, T>
/// class particle_index_type
/// {
///     public:
///     particle_index_type(
///         typename Particle<T>::size_type id, Particle<S> *pptr);
///     size_type id() const;
///     Particle<S> &particle() const;
///     Particle<S> *patricle_ptr() const;
/// };
/// ~~~
/// Usually you can safely derive `particle_index_type<S>` from
/// ParticleIndexBase<S> and add methods specific to `S`.
template <typename T>
class ParticleIndex : public ParticleIndexBaseType<T>
{
    public:
    using particle_type = Particle<T>;
    using size_type = typename particle_type::size_type;
    using rng_type = typename particle_type::rng_type;

    ParticleIndex(size_type id, particle_type *pptr)
        : ParticleIndexBaseType<T>(id, pptr)
    {
    }

    template <typename IntType>
    ParticleIndex<T> operator[](IntType n)
    {
        return ParticleIndex<T>(static_cast<typename Particle<T>::size_type>(
                                    static_cast<std::ptrdiff_t>(this->id()) +
                                    static_cast<std::ptrdiff_t>(n)),
            this->particle_ptr());
    }

    ParticleIndex<T> &operator*() { return *this; }

    const ParticleIndex<T> &operator*() const { return *this; }

    ParticleIndex<T> *operator->() { return this; }

    const ParticleIndex<T> *operator->() const { return this; }
}; // class ParticleIndex

template <typename T>
inline bool operator==(
    const ParticleIndex<T> &idx1, const ParticleIndex<T> &idx2)
{
    VSMC_RUNTIME_ASSERT_CORE_PARTICLE_INDEX_COMPARE(idx1, idx2);

    return idx1.id() == idx2.id();
}

template <typename T>
inline bool operator!=(
    const ParticleIndex<T> &idx1, const ParticleIndex<T> &idx2)
{
    VSMC_RUNTIME_ASSERT_CORE_PARTICLE_INDEX_COMPARE(idx1, idx2);

    return idx1.id() != idx2.id();
}

template <typename T>
inline bool operator<(
    const ParticleIndex<T> &idx1, const ParticleIndex<T> &idx2)
{
    VSMC_RUNTIME_ASSERT_CORE_PARTICLE_INDEX_COMPARE(idx1, idx2);

    return idx1.id() < idx2.id();
}

template <typename T>
inline bool operator>(
    const ParticleIndex<T> &idx1, const ParticleIndex<T> &idx2)
{
    VSMC_RUNTIME_ASSERT_CORE_PARTICLE_INDEX_COMPARE(idx1, idx2);

    return idx1.id() > idx2.id();
}

template <typename T>
inline bool operator<=(
    const ParticleIndex<T> &idx1, const ParticleIndex<T> &idx2)
{
    VSMC_RUNTIME_ASSERT_CORE_PARTICLE_INDEX_COMPARE(idx1, idx2);

    return idx1.id() <= idx2.id();
}

template <typename T>
inline bool operator>=(
    const ParticleIndex<T> &idx1, const ParticleIndex<T> &idx2)
{
    VSMC_RUNTIME_ASSERT_CORE_PARTICLE_INDEX_COMPARE(idx1, idx2);

    return idx1.id() >= idx2.id();
}

template <typename T>
inline ParticleIndex<T> &operator++(ParticleIndex<T> &idx)
{
    idx = ParticleIndex<T>(idx.id() + 1, idx.particle_ptr());

    return idx;
}

template <typename T>
inline ParticleIndex<T> operator++(ParticleIndex<T> &idx, int)
{
    ParticleIndex<T> idx_tmp(idx);
    idx = ParticleIndex<T>(idx.id() + 1, idx.particle_ptr());

    return idx_tmp;
}

template <typename T>
inline ParticleIndex<T> &operator--(ParticleIndex<T> &idx)
{
    idx = ParticleIndex<T>(idx.id() - 1, idx.particle_ptr());

    return idx;
}

template <typename T>
inline ParticleIndex<T> operator--(ParticleIndex<T> &idx, int)
{
    ParticleIndex<T> idx_tmp(idx);
    idx = ParticleIndex<T>(idx.id() - 1, idx.particle_ptr());

    return idx_tmp;
}

template <typename T, typename IntType>
inline ParticleIndex<T> operator+(const ParticleIndex<T> &idx, IntType n)
{
    return ParticleIndex<T>(static_cast<typename Particle<T>::size_type>(
                                static_cast<std::ptrdiff_t>(idx.id()) +
                                static_cast<std::ptrdiff_t>(n)),
        idx.particle_ptr());
}

template <typename T, typename IntType>
inline ParticleIndex<T> operator+(IntType n, const ParticleIndex<T> &idx)
{
    return ParticleIndex<T>(static_cast<typename Particle<T>::size_type>(
                                static_cast<std::ptrdiff_t>(idx.id()) +
                                static_cast<std::ptrdiff_t>(n)),
        idx.particle_ptr());
}

template <typename T, typename IntType>
inline ParticleIndex<T> operator-(const ParticleIndex<T> &idx, IntType n)
{
    return ParticleIndex<T>(static_cast<typename Particle<T>::size_type>(
                                static_cast<std::ptrdiff_t>(idx.id()) -
                                static_cast<std::ptrdiff_t>(n)),
        idx.particle_ptr());
}

template <typename T, typename IntType>
inline ParticleIndex<T> &operator+=(ParticleIndex<T> &idx, IntType n)
{
    idx = idx + n;

    return idx;
}

template <typename T, typename IntType>
inline ParticleIndex<T> &operator-=(ParticleIndex<T> &idx, IntType n)
{
    idx = idx - n;

    return idx;
}

template <typename T>
inline std::ptrdiff_t operator-(
    const ParticleIndex<T> &idx1, const ParticleIndex<T> &idx2)
{
    VSMC_RUNTIME_ASSERT_CORE_PARTICLE_INDEX_COMPARE(idx1, idx2);

    return static_cast<std::ptrdiff_t>(idx1.id()) -
        static_cast<std::ptrdiff_t>(idx2.id());
}

/// \brief A subset of particles
/// \ingroup Core
template <typename T>
class ParticleRange
{
    public:
    using particle_type = Particle<T>;
    using size_type = typename particle_type::size_type;

    ParticleRange(size_type first, size_type last, particle_type *pptr,
        size_type grainsize = 1)
        : pptr_(pptr), first_(first), last_(last), grainsize_(grainsize)
    {
        VSMC_RUNTIME_ASSERT_CORE_PARTICLE_RANGE(
            pptr_->size(), first_, last_, grainsize_);
    }

    template <typename SplitType>
    ParticleRange(ParticleRange<T> &other, SplitType)
        : pptr_(other.pptr_)
        , first_((other.first_ + other.last_) / 2)
        , last_(other.last_)
        , grainsize_(other.grainsize_)
    {
        VSMC_RUNTIME_ASSERT_CORE_PARTICLE_RANGE(
            pptr_->size(), first_, last_, grainsize_);

        other.last_ = first_;
    }

    particle_type &particle() const { return *pptr_; }

    particle_type *particle_ptr() const { return pptr_; }

    size_type size() const { return last_ - first_; }

    size_type first() const { return first_; }

    size_type last() const { return last_; }

    ParticleIndex<T> begin() const { return pptr_->index(first_); }

    ParticleIndex<T> end() const { return pptr_->index(last_); }

    bool empty() const { return first_ == last_; }

    bool is_divisible() const { return size() > grainsize_; }

    private:
    particle_type *pptr_;
    size_type first_;
    size_type last_;
    size_type grainsize_;
}; // class ParticleRange

/// \brief Particle class representing the whole particle set
/// \ingroup Core
template <typename T>
class Particle
{
    public:
    using size_type = SizeType<T>;
    using state_type = T;
    using weight_type = WeightType<T>;
    using rng_set_type = RNGSetType<T>;
    using rng_type = typename rng_set_type::rng_type;

    Particle() : size_(0), state_(0), weight_(0), rng_set_(0)
    {
        Seed::instance()(rng_);
    }

    template <typename... Args>
    explicit Particle(size_type N, Args &&... args)
        : size_(N)
        , state_(N, std::forward<Args>(args)...)
        , weight_(static_cast<SizeType<weight_type>>(N))
        , rng_set_(static_cast<SizeType<rng_set_type>>(N))
    {
        Seed::instance()(rng_);
    }

    /// \brief Clone the Particle except the RNG engines
    Particle<T> clone() const
    {
        Particle<T> particle(*this);
        particle.rng_set_.seed();
        Seed::instance()(particle.rng_);

        return particle;
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

    /// \brief Read and write access to the state collection object
    state_type &state() { return state_; }

    /// \brief Read only access to the state collection object
    const state_type &state() const { return state_; }

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

    /// \brief Get a ParticleIndex<T> object
    ParticleIndex<T> index(size_type id) { return ParticleIndex<T>(id, this); }

    /// \brief Get a ParticleIndex<T> object for the first particle
    ParticleIndex<T> begin() { return index(0); }

    /// \brief Get a ParticleIndex<T> object for one pass the last particle
    ParticleIndex<T> end() { return index(size_); }

    /// \brief Get a ParticleRange<T> object of a given range
    ParticleRange<T> range(
        size_type first, size_type last, size_type grainsize = 1)
    {
        return ParticleRange<T>(first, last, this, grainsize);
    }

    /// \brief Get a ParticleRange<T> object of all particles
    ParticleRange<T> range(size_type grainsize = 1)
    {
        return ParticleRange<T>(0, size(), this, grainsize);
    }

    private:
    static constexpr std::size_t M_ = internal::BufferSize<size_type>::value;

    size_type size_;
    state_type state_;
    weight_type weight_;
    rng_set_type rng_set_;
    rng_type rng_;

    void resize(size_type N, const size_type *idx)
    {
        size_ = N;
        state_.select(N, idx);
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
