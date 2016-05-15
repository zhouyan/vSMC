//============================================================================
// vSMC/include/vsmc/smp/backend_base.hpp
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

#ifndef VSMC_SMP_BACKEND_BASE_HPP
#define VSMC_SMP_BACKEND_BASE_HPP

#include <vsmc/internal/common.hpp>

#ifdef VSMC_CLANG
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wweak-vtables"
#endif

/// \brief Default SMP backend
/// \ingroup Config
#ifndef VSMC_SMP_BACKEND
#if VSMC_HAS_OMP
#define VSMC_SMP_BACKEND ::vsmc::BackendOMP
#elif VSMC_HAS_TBB
#define VSMC_SMP_BACKEND ::vsmc::BackendTBB
#else
#define VSMC_SMP_BACKEND ::vsmc::BackendSTD
#endif
#endif

#define VSMC_DEFINE_SMP_BACKEND_BASE_SPECIAL(Name)                            \
    Name##Base() = default;                                                   \
    Name##Base(const Name##Base<T, Derived> &) = default;                     \
    Name##Base<T, Derived> &operator=(const Name##Base<T, Derived> &) =       \
        default;                                                              \
    Name##Base(Name##Base<T, Derived> &&) = default;                          \
    Name##Base<T, Derived> &operator=(Name##Base<T, Derived> &&) = default;

#define VSMC_DEFINE_SMP_BACKEND_BASE_SPECIAL_VIRTUAL(Name)                    \
    Name##Base() = default;                                                   \
    Name##Base(const Name##Base<T, Virtual> &) = default;                     \
    Name##Base<T, Virtual> &operator=(const Name##Base<T, Virtual> &) =       \
        default;                                                              \
    Name##Base(Name##Base<T, Virtual> &&) = default;                          \
    Name##Base<T, Virtual> &operator=(Name##Base<T, Virtual> &&) = default;   \
    virtual ~Name##Base() {}

#define VSMC_DEFINE_SMP_BACKEND_SPECIAL(Impl, Name)                           \
    Name##SMP() = default;                                                    \
    Name##SMP(const Name##SMP<T, Derived, Backend##Impl> &) = default;        \
    Name##SMP<T, Derived, Backend##Impl> &operator=(                          \
        Name##SMP<T, Derived, Backend##Impl> &) = default;                    \
    Name##SMP(Name##SMP<T, Derived, Backend##Impl> &&) = default;             \
    Name##SMP<T, Derived, Backend##Impl> &operator=(                          \
        Name##SMP<T, Derived, Backend##Impl> &&) = default;

namespace vsmc
{

/// \brief SMP implementation ID for sequential
/// \ingroup SMP
class BackendSEQ;

/// \brief SMP implementation ID for the standard library
/// \ingroup SMP
class BackendSTD;

/// \brief SMP implementation ID for OpenMP
/// \ingroup SMP
class BackendOMP;

/// \brief SMP implementation ID for Intel Threading Building Blocks
/// \ingroup SMP
class BackendTBB;

/// \brief SMP default implementation ID
/// \ingroup SMP
using BackendSMP = VSMC_SMP_BACKEND;

/// \brief Template type parameter that cause the base class to use dynamic
/// dispatch
/// \ingroup SMP
class Virtual;

/// \brief Sampler<T>::eval_type
/// \ingroup SMP
template <typename T, typename = Virtual, typename = BackendSMP>
class SamplerEvalSMP;

/// \brief Monitor<T>::eval_type
/// \ingroup SMP
template <typename T, typename = Virtual, typename = BackendSMP>
class MonitorEvalSMP;

/// \brief Sampler evaluation base dispatch class
/// \ingroup SMP
template <typename T, typename Derived>
class SamplerEvalBase
{
    protected:
    VSMC_DEFINE_SMP_BACKEND_BASE_SPECIAL(SamplerEval)

    void eval_each(std::size_t iter, ParticleIndex<T> idx)
    {
        eval_dispatch(iter, idx, &Derived::eval_each);
    }

    void eval_range(std::size_t iter, const ParticleRange<T> &range)
    {
        eval_range_dispatch(iter, range, &Derived::eval_range);
    }

    void eval_first(std::size_t iter, Particle<T> &particle)
    {
        eval_first_dispatch(iter, particle, &Derived::eval_first);
    }

    void eval_last(std::size_t iter, Particle<T> &particle)
    {
        eval_last_dispatch(iter, particle, &Derived::eval_last);
    }

    private:
    // non-static non-const

    template <typename D>
    void eval_dispatch(std::size_t iter, ParticleIndex<T> idx,
        void (D::*)(std::size_t, ParticleIndex<T>))
    {
        static_cast<Derived *>(this)->eval_each(iter, idx);
    }

    template <typename D>
    void eval_range_dispatch(std::size_t iter, const ParticleRange<T> &range,
        void (D::*)(std::size_t, const ParticleRange<T> &))
    {
        static_cast<Derived *>(this)->eval_range(iter, range);
    }

    template <typename D>
    void eval_first_dispatch(std::size_t iter, Particle<T> &particle,
        void (D::*)(std::size_t, Particle<T> &))
    {
        static_cast<Derived *>(this)->eval_first(iter, particle);
    }

    template <typename D>
    void eval_last_dispatch(std::size_t iter, Particle<T> &particle,
        void (D::*)(std::size_t, Particle<T> &))
    {
        static_cast<Derived *>(this)->eval_last(iter, particle);
    }

    // non-static const

    template <typename D>
    void eval_dispatch(std::size_t iter, ParticleIndex<T> idx,
        void (D::*)(std::size_t, ParticleIndex<T>) const)
    {
        static_cast<Derived *>(this)->eval_each(iter, idx);
    }

    template <typename D>
    void eval_range_dispatch(std::size_t iter, const ParticleRange<T> &range,
        void (D::*)(std::size_t, const ParticleRange<T> &) const)
    {
        static_cast<Derived *>(this)->eval_range(iter, range);
    }

    template <typename D>
    void eval_first_dispatch(std::size_t iter, Particle<T> &particle,
        void (D::*)(std::size_t, Particle<T> &) const)
    {
        static_cast<Derived *>(this)->eval_first(iter, particle);
    }

    template <typename D>
    void eval_last_dispatch(std::size_t iter, Particle<T> &particle,
        void (D::*)(std::size_t, Particle<T> &) const)
    {
        static_cast<Derived *>(this)->eval_last(iter, particle);
    }

    // static

    void eval_dispatch(std::size_t iter, ParticleIndex<T> idx,
        void (*)(std::size_t, ParticleIndex<T>))
    {
        Derived::eval_each(iter, idx);
    }

    void eval_range_dispatch(std::size_t iter, const ParticleRange<T> &range,
        void (*)(std::size_t, const ParticleRange<T> &))
    {
        Derived::eval_range(iter, range);
    }

    void eval_first_dispatch(std::size_t iter, Particle<T> &particle,
        void (*)(std::size_t, Particle<T> &))
    {
        Derived::eval_first(iter, particle);
    }

    void eval_last_dispatch(std::size_t iter, Particle<T> &particle,
        void (*)(std::size_t, Particle<T> &))
    {
        Derived::eval_last(iter, particle);
    }

    // base

    void eval_dispatch(std::size_t, ParticleIndex<T>,
        void (SamplerEvalBase::*)(std::size_t, ParticleIndex<T>))
    {
    }

    void eval_range_dispatch(std::size_t iter, const ParticleRange<T> &range,
        void (SamplerEvalBase::*)(std::size_t, const ParticleRange<T> &))
    {
        for (auto idx : range)
            eval_each(iter, idx);
    }

    void eval_first_dispatch(std::size_t, Particle<T> &,
        void (SamplerEvalBase::*)(std::size_t, Particle<T> &))
    {
    }

    void eval_last_dispatch(std::size_t, Particle<T> &,
        void (SamplerEvalBase::*)(std::size_t, Particle<T> &))
    {
    }
}; // class SamplerEvalBase

/// \brief Mampler evaluation base dispatch class
/// \ingroup SMP
template <typename T>
class SamplerEvalBase<T, Virtual>
{
    protected:
    VSMC_DEFINE_SMP_BACKEND_BASE_SPECIAL_VIRTUAL(SamplerEval)

    virtual void eval_each(std::size_t, ParticleIndex<T>) {}

    virtual void eval_range(std::size_t iter, const ParticleRange<T> &range)
    {
        for (auto idx : range)
            eval_each(iter, idx);
    }

    virtual void eval_first(std::size_t, Particle<T> &) {}

    virtual void eval_last(std::size_t, Particle<T> &) {}
}; // class SamplerEvalBase<T, Virtual>

/// \brief Monitor evalution base dispatch class
/// \ingroup SMP
template <typename T, typename Derived>
class MonitorEvalBase
{
    protected:
    VSMC_DEFINE_SMP_BACKEND_BASE_SPECIAL(MonitorEval)

    void eval_each(
        std::size_t iter, std::size_t dim, ParticleIndex<T> idx, double *r)
    {
        eval_dispatch(iter, dim, idx, r, &Derived::eval_each);
    }

    void eval_range(std::size_t iter, std::size_t dim,
        const ParticleRange<T> &range, double *r)
    {
        eval_range_dispatch(iter, dim, range, r, &Derived::eval_range);
    }

    void eval_first(std::size_t iter, Particle<T> &particle)
    {
        eval_first_dispatch(iter, particle, &Derived::eval_first);
    }

    void eval_last(std::size_t iter, Particle<T> &particle)
    {
        eval_last_dispatch(iter, particle, &Derived::eval_last);
    }

    private:
    // non-static non-const

    template <typename D>
    void eval_dispatch(std::size_t iter, std::size_t dim, ParticleIndex<T> idx,
        double *r,
        void (D::*)(std::size_t, std::size_t, ParticleIndex<T>, double *))
    {
        static_cast<Derived *>(this)->eval_each(iter, dim, idx, r);
    }

    template <typename D>
    void eval_range_dispatch(std::size_t iter, std::size_t dim,
        const ParticleRange<T> &range, double *r,
        void (D::*)(std::size_t, std::size_t, const ParticleRange<T> &,
                                 double *))
    {
        static_cast<Derived *>(this)->eval_range(iter, dim, range, r);
    }

    template <typename D>
    void eval_first_dispatch(std::size_t iter, Particle<T> &particle,
        void (D::*)(std::size_t, Particle<T> &))
    {
        static_cast<Derived *>(this)->eval_first(iter, particle);
    }

    template <typename D>
    void eval_last_dispatch(std::size_t iter, Particle<T> &particle,
        void (D::*)(std::size_t, Particle<T> &))
    {
        static_cast<Derived *>(this)->eval_last(iter, particle);
    }

    // non-static const

    template <typename D>
    void eval_dispatch(std::size_t iter, std::size_t dim, ParticleIndex<T> idx,
        double *r, void (D::*)(std::size_t, std::size_t, ParticleIndex<T>,
                           double *) const)
    {
        static_cast<Derived *>(this)->eval_each(iter, dim, idx, r);
    }

    template <typename D>
    void eval_range_dispatch(std::size_t iter, std::size_t dim,
        const ParticleRange<T> &range, double *r,
        void (D::*)(std::size_t, std::size_t, const ParticleRange<T> &,
                                 double *) const)
    {
        static_cast<Derived *>(this)->eval_range(iter, dim, range, r);
    }

    template <typename D>
    void eval_first_dispatch(std::size_t iter, Particle<T> &particle,
        void (D::*)(std::size_t, Particle<T> &) const)
    {
        static_cast<Derived *>(this)->eval_first(iter, particle);
    }

    template <typename D>
    void eval_last_dispatch(std::size_t iter, Particle<T> &particle,
        void (D::*)(std::size_t, Particle<T> &) const)
    {
        static_cast<Derived *>(this)->eval_last(iter, particle);
    }

    // static

    void eval_dispatch(std::size_t iter, std::size_t dim, ParticleIndex<T> idx,
        double *r,
        void (*)(std::size_t, std::size_t, ParticleIndex<T>, double *))
    {
        Derived::eval_each(iter, dim, idx, r);
    }

    void eval_range_dispatch(std::size_t iter, std::size_t dim,
        const ParticleRange<T> &range, double *r,
        void (*)(std::size_t, std::size_t, const ParticleRange<T> &, double *))
    {
        Derived::eval_range(iter, dim, range, r);
    }

    void eval_first_dispatch(std::size_t iter, Particle<T> &particle,
        void (*)(std::size_t, Particle<T> &))
    {
        Derived::eval_first(iter, particle);
    }

    void eval_last_dispatch(std::size_t iter, Particle<T> &particle,
        void (*)(std::size_t, Particle<T> &))
    {
        Derived::eval_last(iter, particle);
    }

    // base

    void eval_dispatch(std::size_t, std::size_t, ParticleIndex<T>, double *,
        void (MonitorEvalBase::*)(std::size_t, std::size_t, ParticleIndex<T>,
                           double *))
    {
    }

    void eval_range_dispatch(std::size_t iter, std::size_t dim,
        const ParticleRange<T> &range, double *r,
        void (MonitorEvalBase::*)(std::size_t, std::size_t,
                                 const ParticleRange<T> &, double *))
    {
        for (auto idx : range) {
            eval_each(iter, dim, idx, r);
            r += dim;
        }
    }

    void eval_first_dispatch(std::size_t, Particle<T> &,
        void (MonitorEvalBase::*)(std::size_t, Particle<T> &))
    {
    }

    void eval_last_dispatch(std::size_t, Particle<T> &,
        void (MonitorEvalBase::*)(std::size_t, Particle<T> &))
    {
    }
}; // class MonitorBase

/// \brief Monitor evalution base dispatch class
/// \ingroup SMP
template <typename T>
class MonitorEvalBase<T, Virtual>
{
    protected:
    VSMC_DEFINE_SMP_BACKEND_BASE_SPECIAL_VIRTUAL(MonitorEval)

    virtual void eval_each(
        std::size_t, std::size_t, ParticleIndex<T>, double *)
    {
    }

    virtual void eval_range(std::size_t iter, std::size_t dim,
        const ParticleRange<T> &range, double *r)
    {
        for (auto idx : range) {
            eval_each(iter, dim, idx, r);
            r += dim;
        }
    }

    virtual void eval_first(std::size_t, Particle<T> &) {}

    virtual void eval_last(std::size_t, Particle<T> &) {}
}; // class MonitorEvalBase<T, Virtual>

} // namespace vsmc

#ifdef VSMC_CLANG
#pragma clang diagnostic pop
#endif

#endif // VSMC_SMP_BACKEND_BASE_HPP
