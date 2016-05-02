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
    Name##SMP(const Name##SMP<Backend##Impl, T, Derived> &) = default;        \
    Name##SMP<Backend##Impl, T, Derived> &operator=(                          \
        Name##SMP<Backend##Impl, T, Derived> &) = default;                    \
    Name##SMP(Name##SMP<Backend##Impl, T, Derived> &&) = default;             \
    Name##SMP<Backend##Impl, T, Derived> &operator=(                          \
        Name##SMP<Backend##Impl, T, Derived> &&) = default;

namespace vsmc
{

/// \brief Template type parameter that cause the base class to use dynamic
/// dispatch
/// \ingroup SMP
class Virtual;

/// \brief Sampler<T>::init_type
/// \ingroup SMP
template <typename Backend, typename T, typename = Virtual>
class InitializeSMP;

/// \brief Sampler<T>::move_type
/// \ingroup SMP
template <typename Backend, typename T, typename = Virtual>
class MoveSMP;

/// \brief Monitor<T>::eval_type
/// \ingroup SMP
template <typename Backend, typename T, typename = Virtual>
class MonitorEvalSMP;

/// \brief Initialize base dispatch class
/// \ingroup SMP
template <typename T, typename Derived>
class InitializeBase
{
    public:
    std::size_t eval_sp(SingleParticle<T> sp)
    {
        return eval_sp_dispatch(sp, &Derived::eval_sp);
    }

    void eval_param(Particle<T> &particle, void *param)
    {
        eval_param_dispatch(particle, param, &Derived::eval_param);
    }

    void eval_pre(Particle<T> &particle)
    {
        eval_pre_dispatch(particle, &Derived::eval_pre);
    }

    void eval_post(Particle<T> &particle)
    {
        eval_post_dispatch(particle, &Derived::eval_post);
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_BASE_SPECIAL(Initialize)

    private:
    // non-static non-const

    template <typename D>
    std::size_t eval_sp_dispatch(
        SingleParticle<T> sp, std::size_t (D::*)(SingleParticle<T>))
    {
        return static_cast<Derived *>(this)->eval_sp(sp);
    }

    template <typename D>
    void eval_param_dispatch(
        Particle<T> &particle, void *param, void (D::*)(Particle<T> &, void *))
    {
        static_cast<Derived *>(this)->eval_param(particle, param);
    }

    template <typename D>
    void eval_pre_dispatch(Particle<T> &particle, void (D::*)(Particle<T> &))
    {
        static_cast<Derived *>(this)->eval_pre(particle);
    }

    template <typename D>
    void eval_post_dispatch(Particle<T> &particle, void (D::*)(Particle<T> &))
    {
        static_cast<Derived *>(this)->eval_post(particle);
    }

    // non-static const

    template <typename D>
    std::size_t eval_sp_dispatch(
        SingleParticle<T> sp, std::size_t (D::*)(SingleParticle<T>) const)
    {
        return static_cast<Derived *>(this)->eval_sp(sp);
    }

    template <typename D>
    void eval_param_dispatch(Particle<T> &particle, void *param,
        void (D::*)(Particle<T> &, void *) const)
    {
        static_cast<Derived *>(this)->eval_param(particle, param);
    }

    template <typename D>
    void eval_pre_dispatch(
        Particle<T> &particle, void (D::*)(Particle<T> &) const)
    {
        static_cast<Derived *>(this)->eval_pre(particle);
    }

    template <typename D>
    void eval_post_dispatch(
        Particle<T> &particle, void (D::*)(Particle<T> &) const)
    {
        static_cast<Derived *>(this)->eval_post(particle);
    }

    // static

    std::size_t eval_sp_dispatch(
        SingleParticle<T> sp, std::size_t (*)(SingleParticle<T>))
    {
        return Derived::eval_sp(sp);
    }

    void eval_param_dispatch(
        Particle<T> &particle, void *param, void (*)(Particle<T> &, void *))
    {
        Derived::eval_param(particle, param);
    }

    void eval_pre_dispatch(Particle<T> &particle, void (*)(Particle<T> &))
    {
        Derived::eval_pre(particle);
    }

    void eval_post_dispatch(Particle<T> &particle, void (*)(Particle<T> &))
    {
        Derived::eval_post(particle);
    }

    // base

    std::size_t eval_sp_dispatch(
        SingleParticle<T>, std::size_t (InitializeBase::*)(SingleParticle<T>))
    {
        return 0;
    }

    void eval_param_dispatch(
        Particle<T> &, void *, void (InitializeBase::*)(Particle<T> &, void *))
    {
    }

    void eval_pre_dispatch(
        Particle<T> &, void (InitializeBase::*)(Particle<T> &))
    {
    }

    void eval_post_dispatch(
        Particle<T> &, void (InitializeBase::*)(Particle<T> &))
    {
    }
}; // class InitializeBase

/// \brief Initilaize base dispatch class
/// \ingroup SMP
template <typename T>
class InitializeBase<T, Virtual>
{
    public:
    virtual std::size_t eval_sp(SingleParticle<T>) { return 0; }
    virtual void eval_param(Particle<T> &, void *) {}
    virtual void eval_pre(Particle<T> &) {}
    virtual void eval_post(Particle<T> &) {}

    protected:
    VSMC_DEFINE_SMP_BACKEND_BASE_SPECIAL_VIRTUAL(Initialize)
}; // class InitializeBase<T, Virtual>

/// \brief Move base dispatch class
/// \ingroup SMP
template <typename T, typename Derived>
class MoveBase
{
    public:
    std::size_t eval_sp(std::size_t iter, SingleParticle<T> sp)
    {
        return eval_sp_dispatch(iter, sp, &Derived::eval_sp);
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
    VSMC_DEFINE_SMP_BACKEND_BASE_SPECIAL(Move)

    private:
    // non-static non-const

    template <typename D>
    std::size_t eval_sp_dispatch(std::size_t iter, SingleParticle<T> sp,
        std::size_t (D::*)(std::size_t, SingleParticle<T>))
    {
        return static_cast<Derived *>(this)->eval_sp(iter, sp);
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
        std::size_t (MoveBase::*)(std::size_t, SingleParticle<T>))
    {
        return 0;
    }

    void eval_pre_dispatch(std::size_t, Particle<T> &,
        void (MoveBase::*)(std::size_t, Particle<T> &))
    {
    }

    void eval_post_dispatch(std::size_t, Particle<T> &,
        void (MoveBase::*)(std::size_t, Particle<T> &))
    {
    }
}; // class MoveBase

/// \brief Move base dispatch class
/// \ingroup SMP
template <typename T>
class MoveBase<T, Virtual>
{
    public:
    virtual std::size_t eval_sp(std::size_t, SingleParticle<T>) { return 0; }
    virtual void eval_pre(std::size_t, Particle<T> &) {}
    virtual void eval_post(std::size_t, Particle<T> &) {}

    protected:
    VSMC_DEFINE_SMP_BACKEND_BASE_SPECIAL_VIRTUAL(Move)
}; // class MoveBase<T, Virtual>

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
