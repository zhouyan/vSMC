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
/// \ingroup SEQ
class BackendSEQ;

/// \brief SMP implementation ID for the standard library
/// \ingroup STD
class BackendSTD;

/// \brief SMP implementation ID for OpenMP
/// \ingroup OMP
class BackendOMP;

/// \brief SMP implementation ID for Intel Threading Building Blocks
/// \ingroup TBB
class BackendTBB;

/// \brief Template type parameter that cause the base class to use dynamic
/// dispatch
/// \ingroup SMP
class Virtual;

/// \brief Sampler<T>::eval_type
/// \ingroup SMP
template <typename T, typename = Virtual, typename = VSMC_SMP_BACKEND>
class SamplerEvalSMP;

/// \brief Monitor<T>::eval_type
/// \ingroup SMP
template <typename T, typename = Virtual, typename = VSMC_SMP_BACKEND>
class MonitorEvalSMP;

/// \brief Sampler evaluation base dispatch class
/// \ingroup SMP
template <typename T, typename Derived>
class SamplerEvalBase
{
    public:
    std::size_t eval_sp(std::size_t iter, SingleParticle<T> sp)
    {
        return eval_sp_dispatch(iter, sp, &Derived::eval_sp);
    }

    std::size_t eval_range(std::size_t iter, ParticleRange<T> range)
    {
        return eval_range_dispatch(iter, range, &Derived::eval_range);
    }

    void eval_pre(std::size_t iter, Particle<T> &particle)
    {
        eval_pre_dispatch(iter, particle, &Derived::eval_pre);
    }

    void eval_post(std::size_t iter, Particle<T> &particle)
    {
        eval_post_dispatch(iter, particle, &Derived::eval_post);
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_BASE_SPECIAL(SamplerEval)

    private:
    // non-static non-const

    template <typename D>
    std::size_t eval_sp_dispatch(std::size_t iter, SingleParticle<T> sp,
        std::size_t (D::*)(std::size_t, SingleParticle<T>))
    {
        return static_cast<Derived *>(this)->eval_sp(iter, sp);
    }

    template <typename D>
    std::size_t eval_range_dispatch(std::size_t iter, ParticleRange<T> range,
        std::size_t (D::*)(std::size_t, ParticleRange<T>))
    {
        return static_cast<Derived *>(this)->eval_range(iter, range);
    }

    template <typename D>
    void eval_pre_dispatch(std::size_t iter, Particle<T> &particle,
        void (D::*)(std::size_t, Particle<T> &))
    {
        static_cast<Derived *>(this)->eval_pre(iter, particle);
    }

    template <typename D>
    void eval_post_dispatch(std::size_t iter, Particle<T> &particle,
        void (D::*)(std::size_t, Particle<T> &))
    {
        static_cast<Derived *>(this)->eval_post(iter, particle);
    }

    // non-static const

    template <typename D>
    std::size_t eval_sp_dispatch(std::size_t iter, SingleParticle<T> sp,
        std::size_t (D::*)(std::size_t, SingleParticle<T>) const)
    {
        return static_cast<Derived *>(this)->eval_sp(iter, sp);
    }

    template <typename D>
    std::size_t eval_range_dispatch(std::size_t iter, ParticleRange<T> range,
        std::size_t (D::*)(std::size_t, ParticleRange<T>) const)
    {
        return static_cast<Derived *>(this)->eval_range(iter, range);
    }

    template <typename D>
    void eval_pre_dispatch(std::size_t iter, Particle<T> &particle,
        void (D::*)(std::size_t, Particle<T> &) const)
    {
        static_cast<Derived *>(this)->eval_pre(iter, particle);
    }

    template <typename D>
    void eval_post_dispatch(std::size_t iter, Particle<T> &particle,
        void (D::*)(std::size_t, Particle<T> &) const)
    {
        static_cast<Derived *>(this)->eval_post(iter, particle);
    }

    // static

    std::size_t eval_sp_dispatch(std::size_t iter, SingleParticle<T> sp,
        std::size_t (*)(std::size_t, SingleParticle<T>))
    {
        return Derived::eval_sp(iter, sp);
    }

    std::size_t eval_range_dispatch(std::size_t iter, ParticleRange<T> range,
        std::size_t (*)(std::size_t, ParticleRange<T>))
    {
        return Derived::eval_range(iter, range);
    }

    void eval_pre_dispatch(std::size_t iter, Particle<T> &particle,
        void (*)(std::size_t, Particle<T> &))
    {
        Derived::eval_pre(iter, particle);
    }

    void eval_post_dispatch(std::size_t iter, Particle<T> &particle,
        void (*)(std::size_t, Particle<T> &))
    {
        Derived::eval_post(iter, particle);
    }

    // base

    std::size_t eval_sp_dispatch(std::size_t, SingleParticle<T>,
        std::size_t (SamplerEvalBase::*)(std::size_t, SingleParticle<T>))
    {
        return 0;
    }

    std::size_t eval_range_dispatch(std::size_t iter, ParticleRange<T> range,
        std::size_t (SamplerEvalBase::*)(std::size_t, ParticleRange<T>))
    {
        using size_type = typename Particle<T>::size_type;

        std::size_t accept = 0;
        for (size_type i = range.begin(); i != range.end(); ++i)
            accept += eval_sp(iter, range.particle().sp(i));

        return accept;
    }

    void eval_pre_dispatch(std::size_t, Particle<T> &,
        void (SamplerEvalBase::*)(std::size_t, Particle<T> &))
    {
    }

    void eval_post_dispatch(std::size_t, Particle<T> &,
        void (SamplerEvalBase::*)(std::size_t, Particle<T> &))
    {
    }
}; // class SamplerEvalBase

/// \brief Mampler evaluation base dispatch class
/// \ingroup SMP
template <typename T>
class SamplerEvalBase<T, Virtual>
{
    public:
    virtual std::size_t eval_sp(std::size_t, SingleParticle<T>) { return 0; }

    virtual std::size_t eval_range(std::size_t iter, ParticleRange<T> range)
    {
        using size_type = typename Particle<T>::size_type;

        std::size_t accept = 0;
        for (size_type i = range.begin(); i != range.end(); ++i)
            accept += eval_sp(iter, range.particle().sp(i));

        return accept;
    }

    virtual void eval_pre(std::size_t, Particle<T> &) {}

    virtual void eval_post(std::size_t, Particle<T> &) {}

    protected:
    VSMC_DEFINE_SMP_BACKEND_BASE_SPECIAL_VIRTUAL(SamplerEval)
}; // class SamplerEvalBase<T, Virtual>

/// \brief Monitor evalution base dispatch class
/// \ingroup SMP
template <typename T, typename Derived>
class MonitorEvalBase
{
    public:
    void eval_sp(
        std::size_t iter, std::size_t dim, SingleParticle<T> sp, double *r)
    {
        eval_sp_dispatch(iter, dim, sp, r, &Derived::eval_sp);
    }

    void eval_range(
        std::size_t iter, std::size_t dim, ParticleRange<T> range, double *r)
    {
        eval_range_dispatch(iter, dim, range, r, &Derived::eval_range);
    }

    void eval_pre(std::size_t iter, Particle<T> &particle)
    {
        eval_pre_dispatch(iter, particle, &Derived::eval_pre);
    }

    void eval_post(std::size_t iter, Particle<T> &particle)
    {
        eval_post_dispatch(iter, particle, &Derived::eval_post);
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_BASE_SPECIAL(MonitorEval)

    private:
    // non-static non-const

    template <typename D>
    void eval_sp_dispatch(std::size_t iter, std::size_t dim,
        SingleParticle<T> sp, double *r,
        void (D::*)(std::size_t, std::size_t, SingleParticle<T>, double *))
    {
        static_cast<Derived *>(this)->eval_sp(iter, dim, sp, r);
    }

    template <typename D>
    void eval_range_dispatch(std::size_t iter, std::size_t dim,
        ParticleRange<T> range, double *r,
        void (D::*)(std::size_t, std::size_t, ParticleRange<T>, double *))
    {
        static_cast<Derived *>(this)->eval_range(iter, dim, range, r);
    }

    template <typename D>
    void eval_pre_dispatch(std::size_t iter, Particle<T> &particle,
        void (D::*)(std::size_t, Particle<T> &))
    {
        static_cast<Derived *>(this)->eval_pre(iter, particle);
    }

    template <typename D>
    void eval_post_dispatch(std::size_t iter, Particle<T> &particle,
        void (D::*)(std::size_t, Particle<T> &))
    {
        static_cast<Derived *>(this)->eval_post(iter, particle);
    }

    // non-static const

    template <typename D>
    void eval_sp_dispatch(std::size_t iter, std::size_t dim,
        SingleParticle<T> sp, double *r,
        void (D::*)(std::size_t, std::size_t, SingleParticle<T>, double *)
            const)
    {
        static_cast<Derived *>(this)->eval_sp(iter, dim, sp, r);
    }

    template <typename D>
    void eval_range_dispatch(std::size_t iter, std::size_t dim,
        ParticleRange<T> range, double *r,
        void (D::*)(std::size_t, std::size_t, ParticleRange<T>, double *)
            const)
    {
        static_cast<Derived *>(this)->eval_range(iter, dim, range, r);
    }

    template <typename D>
    void eval_pre_dispatch(std::size_t iter, Particle<T> &particle,
        void (D::*)(std::size_t, Particle<T> &) const)
    {
        static_cast<Derived *>(this)->eval_pre(iter, particle);
    }

    template <typename D>
    void eval_post_dispatch(std::size_t iter, Particle<T> &particle,
        void (D::*)(std::size_t, Particle<T> &) const)
    {
        static_cast<Derived *>(this)->eval_post(iter, particle);
    }

    // static

    void eval_sp_dispatch(std::size_t iter, std::size_t dim,
        SingleParticle<T> sp, double *r,
        void (*)(std::size_t, std::size_t, SingleParticle<T>, double *))
    {
        Derived::eval_sp(iter, dim, sp, r);
    }

    void eval_range_dispatch(std::size_t iter, std::size_t dim,
        ParticleRange<T> range, double *r,
        void (*)(std::size_t, std::size_t, ParticleRange<T>, double *))
    {
        Derived::eval_range(iter, dim, range, r);
    }

    void eval_pre_dispatch(std::size_t iter, Particle<T> &particle,
        void (*)(std::size_t, Particle<T> &))
    {
        Derived::eval_pre(iter, particle);
    }

    void eval_post_dispatch(std::size_t iter, Particle<T> &particle,
        void (*)(std::size_t, Particle<T> &))
    {
        Derived::eval_post(iter, particle);
    }

    // base

    void eval_sp_dispatch(std::size_t, std::size_t, SingleParticle<T>,
        double *, void (MonitorEvalBase::*)(std::size_t, std::size_t,
                              SingleParticle<T>, double *))
    {
    }

    void eval_range_dispatch(std::size_t iter, std::size_t dim,
        ParticleRange<T> range, double *r,
        void (MonitorEvalBase::*)(std::size_t, std::size_t, ParticleRange<T>,
                                 double *))
    {
        using size_type = typename Particle<T>::size_type;

        for (size_type i = range.begin(); i != range.end(); ++i, r += dim)
            eval_sp(iter, dim, range.particle().sp(i), r);
    }

    void eval_pre_dispatch(std::size_t, Particle<T> &,
        void (MonitorEvalBase::*)(std::size_t, Particle<T> &))
    {
    }

    void eval_post_dispatch(std::size_t, Particle<T> &,
        void (MonitorEvalBase::*)(std::size_t, Particle<T> &))
    {
    }
}; // class MonitorBase

/// \brief Monitor evalution base dispatch class
/// \ingroup SMP
template <typename T>
class MonitorEvalBase<T, Virtual>
{
    public:
    virtual void eval_sp(std::size_t, std::size_t, SingleParticle<T>, double *)
    {
    }

    virtual void eval_range(
        std::size_t iter, std::size_t dim, ParticleRange<T> range, double *r)
    {
        using size_type = typename Particle<T>::size_type;

        for (size_type i = range.begin(); i != range.end(); ++i, r += dim)
            eval_sp(iter, dim, range.particle().sp(i), r);
    }

    virtual void eval_pre(std::size_t, Particle<T> &) {}

    virtual void eval_post(std::size_t, Particle<T> &) {}

    protected:
    VSMC_DEFINE_SMP_BACKEND_BASE_SPECIAL_VIRTUAL(MonitorEval)
}; // class MonitorEvalBase<T, Virtual>

} // namespace vsmc

#ifdef VSMC_CLANG
#pragma clang diagnostic pop
#endif

#endif // VSMC_SMP_BACKEND_BASE_HPP
