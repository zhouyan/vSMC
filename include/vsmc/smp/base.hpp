#ifndef VSMC_SMP_BASE_HPP
#define VSMC_SMP_BASE_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/weight.hpp>
#include <vsmc/core/normalizing_constant.hpp>

#define VSMC_DEFINE_SMP_IMPL_COPY_BASE(Impl, Name)                            \
Name##Impl () {}                                                              \
Name##Impl (const Name##Impl<T, Derived> &other) :                            \
        Name##Base<T, Derived>(other) {}                                      \
Name##Impl<T, Derived> &operator=                                             \
        (const Name##Impl<T, Derived> &other)                                 \
{                                                                             \
    if (this != &other) Name##Base<T, Derived>::operator=(other);             \
    return *this;                                                             \
}                                                                             \
~Name##Impl () {}

namespace vsmc {

/// \brief Initialize base dispatch class
/// \ingroup SMP
template <typename T, typename Derived>
class InitializeBase
{
    public :

    std::size_t initialize_state (SingleParticle<T> sp)
    {return initialize_state_dispatch(sp, &Derived::initialize_state);}

    void initialize_param (Particle<T> &particle, void *param)
    {initialize_param_dispatch(particle, param, &Derived::initialize_param);}

    void pre_processor (Particle<T> &particle)
    {pre_processor_dispatch(particle, &Derived::pre_processor);}

    void post_processor (Particle<T> &particle)
    {post_processor_dispatch(particle, &Derived::post_processor);}

    protected :

    InitializeBase () {}
    InitializeBase (const InitializeBase<T, Derived> &) {}
    InitializeBase<T, Derived> &operator=
        (const InitializeBase<T, Derived> &) {return *this;}
    VSMC_SMP_BASE_DESTRUCTOR_PREFIX ~InitializeBase () {}

    private :

    // non-static non-const

    template <typename D>
    std::size_t initialize_state_dispatch (SingleParticle<T> sp,
            std::size_t (D::*) (SingleParticle<T>))
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(InitializeBase);
        return static_cast<Derived *>(this)->initialize_state(sp);
    }

    template <typename D>
    void initialize_param_dispatch (Particle<T> &particle, void *param,
            void (D::*) (Particle<T> &, void *))
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(InitializeBase);
        static_cast<Derived *>(this)->initialize_param(particle, param);
    }

    template <typename D>
    void pre_processor_dispatch (Particle<T> &particle,
            void (D::*) (Particle<T> &))
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(InitializeBase);
        static_cast<Derived *>(this)->pre_processor(particle);
    }

    template <typename D>
    void post_processor_dispatch (Particle<T> &particle,
            void (D::*) (Particle<T> &))
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(InitializeBase);
        static_cast<Derived *>(this)->post_processor(particle);
    }

    // non-static const

    template <typename D>
    std::size_t initialize_state_dispatch (SingleParticle<T> sp,
            std::size_t (D::*) (SingleParticle<T>) const)
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(InitializeBase);
        return static_cast<Derived *>(this)->initialize_state(sp);
    }

    template <typename D>
    void initialize_param_dispatch (Particle<T> &particle, void *param,
            void (D::*) (Particle<T> &, void *) const)
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(InitializeBase);
        static_cast<Derived *>(this)->initialize_param(particle, param);
    }

    template <typename D>
    void pre_processor_dispatch (Particle<T> &particle,
            void (D::*) (Particle<T> &) const)
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(InitializeBase);
        static_cast<Derived *>(this)->pre_processor(particle);
    }

    template <typename D>
    void post_processor_dispatch (Particle<T> &particle,
            void (D::*) (Particle<T> &) const)
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(InitializeBase);
        static_cast<Derived *>(this)->post_processor(particle);
    }

    // static

    std::size_t initialize_state_dispatch (SingleParticle<T> sp,
            std::size_t (*) (SingleParticle<T>))
    {return Derived::initialize_state(sp);}

    void initialize_param_dispatch (Particle<T> &particle, void *param,
            void (*) (Particle<T> &, void *))
    {Derived::initialize_param(particle, param);}

    void pre_processor_dispatch (Particle<T> &particle,
            void (*) (Particle<T> &))
    {Derived::pre_processor(particle);}

    void post_processor_dispatch (Particle<T> &particle,
            void (*) (Particle<T> &))
    {Derived::post_processor(particle);}

    // base

    std::size_t initialize_state_dispatch (SingleParticle<T>,
            std::size_t (InitializeBase::*) (SingleParticle<T>))
    {VSMC_STATIC_ASSERT_NO_IMPL(initialize_state); return 0;}

    void initialize_param_dispatch (Particle<T> &, void *,
            void (InitializeBase::*) (Particle<T> &, void *)) {}

    void pre_processor_dispatch (Particle<T> &,
            void (InitializeBase::*) (Particle<T> &)) {}

    void post_processor_dispatch (Particle<T> &,
            void (InitializeBase::*) (Particle<T> &)) {}
}; // class InitializeBase

/// \brief Initilaize base dispatch class
/// \ingroup SMP
template <typename T>
class InitializeBase<T, Virtual>
{
    public :

    virtual std::size_t initialize_state (SingleParticle<T>) = 0;
    virtual void initialize_param (Particle<T> &, void *) {}
    virtual void pre_processor (Particle<T> &) {}
    virtual void post_processor (Particle<T> &) {}

    protected :

    InitializeBase () {}
    InitializeBase (const InitializeBase<T, Virtual> &) {}
    InitializeBase<T, Virtual> &operator=
        (const InitializeBase<T, Virtual> &) {return *this;}
    virtual ~InitializeBase () {}
}; // class InitializeBase<T, Virtual>

/// \brief Move base dispatch class
/// \ingroup SMP
template <typename T, typename Derived>
class MoveBase
{
    public :

    std::size_t move_state (std::size_t iter, SingleParticle<T> sp)
    {return move_state_dispatch(iter, sp, &Derived::move_state);}

    void pre_processor (std::size_t iter, Particle<T> &particle)
    {pre_processor_dispatch(iter, particle, &Derived::pre_processor);}

    void post_processor (std::size_t iter, Particle<T> &particle)
    {post_processor_dispatch(iter, particle, &Derived::post_processor);}

    protected :

    MoveBase () {}
    MoveBase (const MoveBase<T, Derived> &) {}
    MoveBase<T, Derived> &operator=
        (const MoveBase<T, Derived> &) {return *this;}
    VSMC_SMP_BASE_DESTRUCTOR_PREFIX ~MoveBase () {}

    private :

    // non-static non-const

    template <typename D>
    std::size_t move_state_dispatch (std::size_t iter, SingleParticle<T> sp,
            std::size_t (D::*) (std::size_t, SingleParticle<T>))
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(MoveBase);
        return static_cast<Derived *>(this)->move_state(iter, sp);
    }

    template <typename D>
    void pre_processor_dispatch (std::size_t iter, Particle<T> &particle,
            void (D::*) (std::size_t, Particle<T> &))
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(MoveBase);
        static_cast<Derived *>(this)->pre_processor(iter, particle);
    }

    template <typename D>
    void post_processor_dispatch (std::size_t iter, Particle<T> &particle,
            void (D::*) (std::size_t, Particle<T> &))
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(MoveBase);
        static_cast<Derived *>(this)->post_processor(iter, particle);
    }

    // non-static const

    template <typename D>
    std::size_t move_state_dispatch (std::size_t iter, SingleParticle<T> sp,
            std::size_t (D::*) (std::size_t, SingleParticle<T>) const)
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(MoveBase);
        return static_cast<Derived *>(this)->move_state(iter, sp);
    }

    template <typename D>
    void pre_processor_dispatch (std::size_t iter, Particle<T> &particle,
            void (D::*) (std::size_t, Particle<T> &) const)
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(MoveBase);
        static_cast<Derived *>(this)->pre_processor(iter, particle);
    }

    template <typename D>
    void post_processor_dispatch (std::size_t iter, Particle<T> &particle,
            void (D::*) (std::size_t, Particle<T> &) const)
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(MoveBase);
        static_cast<Derived *>(this)->post_processor(iter, particle);
    }

    // static

    std::size_t move_state_dispatch (std::size_t iter, SingleParticle<T> sp,
            std::size_t (*) (std::size_t, SingleParticle<T>))
    {return Derived::move_state(iter, sp);}

    void pre_processor_dispatch (std::size_t iter, Particle<T> &particle,
            void (*) (std::size_t, Particle<T> &))
    {Derived::pre_processor(iter, particle);}

    void post_processor_dispatch (std::size_t iter, Particle<T> &particle,
            void (*) (std::size_t, Particle<T> &))
    {Derived::post_processor(iter, particle);}

    // base

    std::size_t move_state_dispatch (std::size_t, SingleParticle<T>,
            std::size_t (MoveBase::*) (std::size_t, SingleParticle<T>))
    {VSMC_STATIC_ASSERT_NO_IMPL(move_state); return 0;}

    void pre_processor_dispatch (std::size_t, Particle<T> &,
            void (MoveBase::*) (std::size_t, Particle<T> &)) {}

    void post_processor_dispatch (std::size_t, Particle<T> &,
            void (MoveBase::*) (std::size_t, Particle<T> &)) {}
}; // class MoveBase

/// \brief Move base dispatch class
/// \ingroup SMP
template <typename T>
class MoveBase<T, Virtual>
{
    public :

    virtual std::size_t move_state (std::size_t, SingleParticle<T>) = 0;
    virtual void pre_processor (std::size_t, Particle<T> &) {}
    virtual void post_processor (std::size_t, Particle<T> &) {}

    protected :

    MoveBase () {}
    MoveBase (const MoveBase<T, Virtual> &) {}
    MoveBase<T, Virtual> &operator=
        (const MoveBase<T, Virtual> &) {return *this;}
    virtual ~MoveBase () {}
}; // class MoveBase<T, Virtual>

/// \brief Monitor evalution base dispatch class
/// \ingroup SMP
template <typename T, typename Derived>
class MonitorEvalBase
{
    public :

    void monitor_state (std::size_t iter, std::size_t dim,
            ConstSingleParticle<T> csp, double *res)
    {monitor_state_dispatch(iter, dim, csp, res, &Derived::monitor_state);}

    void pre_processor (std::size_t iter, const Particle<T> &particle)
    {pre_processor_dispatch(iter, particle, &Derived::pre_processor);}

    void post_processor (std::size_t iter, const Particle<T> &particle)
    {post_processor_dispatch(iter, particle, &Derived::post_processor);}

    protected :

    MonitorEvalBase () {}
    MonitorEvalBase (const MonitorEvalBase<T, Derived> &) {}
    MonitorEvalBase<T, Derived> &operator=
        (const MonitorEvalBase<T, Derived> &) {return *this;}
    VSMC_SMP_BASE_DESTRUCTOR_PREFIX ~MonitorEvalBase () {}

    private :

    // non-static non-const

    template <typename D>
    void monitor_state_dispatch (std::size_t iter, std::size_t dim,
            ConstSingleParticle<T> csp, double *res,
            void (D::*) (std::size_t, std::size_t, ConstSingleParticle<T>,
                double *))
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(MonitorEvalBase);
        static_cast<Derived *>(this)->monitor_state(iter, dim, csp, res);
    }

    template <typename D>
    void pre_processor_dispatch (std::size_t iter, const Particle<T> &particle,
            void (D::*) (std::size_t, const Particle<T> &))
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(MonitorEvalBase);
        static_cast<Derived *>(this)->pre_processor(iter, particle);
    }

    template <typename D>
    void post_processor_dispatch (std::size_t iter,
            const Particle<T> &particle,
            void (D::*) (std::size_t, const Particle<T> &))
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(MonitorEvalBase);
        static_cast<Derived *>(this)->post_processor(iter, particle);
    }

    // non-static const

    template <typename D>
    void monitor_state_dispatch (std::size_t iter, std::size_t dim,
            ConstSingleParticle<T> csp, double *res,
            void (D::*) (std::size_t, std::size_t, ConstSingleParticle<T>,
                double *) const)
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(MonitorEvalBase);
        static_cast<Derived *>(this)->monitor_state(iter, dim, csp, res);
    }

    template <typename D>
    void pre_processor_dispatch (std::size_t iter, const Particle<T> &particle,
            void (D::*) (std::size_t, const Particle<T> &) const)
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(MonitorEvalBase);
        static_cast<Derived *>(this)->pre_processor(iter, particle);
    }

    template <typename D>
    void post_processor_dispatch (std::size_t iter,
            const Particle<T> &particle,
            void (D::*) (std::size_t, const Particle<T> &) const)
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(MonitorEvalBase);
        static_cast<Derived *>(this)->post_processor(iter, particle);
    }

    // static

    void monitor_state_dispatch (std::size_t iter, std::size_t dim,
            ConstSingleParticle<T> csp, double *res,
            void (*) (std::size_t, std::size_t, ConstSingleParticle<T>,
                double *))
    {Derived::monitor_state(iter, dim, csp, res);}

    void pre_processor_dispatch (std::size_t iter, const Particle<T> &particle,
            void (*) (std::size_t, const Particle<T> &))
    {Derived::pre_processor(iter, particle);}

    void post_processor_dispatch (std::size_t iter,
            const Particle<T> &particle,
            void (*) (std::size_t, const Particle<T> &))
    {Derived::post_processor(iter, particle);}

    // base

    void monitor_state_dispatch (std::size_t, std::size_t dim,
            ConstSingleParticle<T>, double *res,
            void (MonitorEvalBase::*)
            (std::size_t, std::size_t, ConstSingleParticle<T>, double *))
    {VSMC_STATIC_ASSERT_NO_IMPL(monitor_state);}

    void pre_processor_dispatch (std::size_t, const Particle<T> &,
            void (MonitorEvalBase::*) (std::size_t, const Particle<T> &)) {}

    void post_processor_dispatch (std::size_t, const Particle<T> &,
            void (MonitorEvalBase::*) (std::size_t, const Particle<T> &)) {}
}; // class MonitorBase

/// \brief Monitor evalution base dispatch class
/// \ingroup SMP
template <typename T>
class MonitorEvalBase<T, Virtual>
{
    public :

    virtual void monitor_state (std::size_t, std::size_t,
            ConstSingleParticle<T>, double *) = 0;
    virtual void pre_processor (std::size_t, const Particle<T> &) {}
    virtual void post_processor (std::size_t, const Particle<T> &) {}

    protected :

    MonitorEvalBase () {}
    MonitorEvalBase (const MonitorEvalBase<T, Virtual> &) {}
    MonitorEvalBase<T, Virtual> &operator=
        (const MonitorEvalBase<T, Virtual> &) {return *this;}
    virtual ~MonitorEvalBase () {}
}; // class MonitorEvalBase<T, Virtual>

/// \brief Path evalution base dispatch class
/// \ingroup SMP
template <typename T, typename Derived>
class PathEvalBase
{
    public :

    double path_state (std::size_t iter, ConstSingleParticle<T> csp)
    {return path_state_dispatch(iter, csp, &Derived::path_state);}

    double path_grid (std::size_t iter, const Particle<T> &particle)
    {return path_grid_dispatch(iter, particle, &Derived::path_grid);}

    void pre_processor (std::size_t iter, const Particle<T> &particle)
    {pre_processor_dispatch(iter, particle, &Derived::pre_processor);}

    void post_processor (std::size_t iter, const Particle<T> &particle)
    {post_processor_dispatch(iter, particle, &Derived::post_processor);}

    protected :

    PathEvalBase () {}
    PathEvalBase (const PathEvalBase<T, Derived> &) {}
    PathEvalBase<T, Derived> &operator=
        (const PathEvalBase<T, Derived> &) {return *this;}
    VSMC_SMP_BASE_DESTRUCTOR_PREFIX ~PathEvalBase () {}

    private :

    // non-static non-const

    template <typename D>
    double path_state_dispatch (std::size_t iter, ConstSingleParticle<T> csp,
            double (D::*) (std::size_t, ConstSingleParticle<T>))
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(PathEvalBase);
        return static_cast<Derived *>(this)->path_state(iter, csp);
    }

    template <typename D>
    double path_grid_dispatch (std::size_t iter, const Particle<T> &particle,
            double (D::*) (std::size_t, const Particle<T> &))
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(PathEvalBase);
        return static_cast<Derived *>(this)->path_grid(iter, particle);
    }

    template <typename D>
    void pre_processor_dispatch (std::size_t iter, const Particle<T> &particle,
            void (D::*) (std::size_t, const Particle<T> &))
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(PathEvalBase);
        static_cast<Derived *>(this)->pre_processor(iter, particle);
    }

    template <typename D>
    void post_processor_dispatch (std::size_t iter,
            const Particle<T> &particle,
            void (D::*) (std::size_t, const Particle<T> &))
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(PathEvalBase);
        static_cast<Derived *>(this)->post_processor(iter, particle);
    }

    // non-static const

    template <typename D>
    double path_state_dispatch (std::size_t iter, ConstSingleParticle<T> csp,
            double (D::*) (std::size_t, ConstSingleParticle<T>) const)
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(PathEvalBase);
        return static_cast<Derived *>(this)->path_state(iter, csp);
    }

    template <typename D>
    double path_grid_dispatch (std::size_t iter, const Particle<T> &particle,
            double (D::*) (std::size_t, const Particle<T> &) const)
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(PathEvalBase);
        return static_cast<Derived *>(this)->path_grid(iter, particle);
    }

    template <typename D>
    void pre_processor_dispatch (std::size_t iter, const Particle<T> &particle,
            void (D::*) (std::size_t, const Particle<T> &) const)
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(PathEvalBase);
        static_cast<Derived *>(this)->pre_processor(iter, particle);
    }

    template <typename D>
    void post_processor_dispatch (std::size_t iter,
            const Particle<T> &particle,
            void (D::*) (std::size_t, const Particle<T> &) const)
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(PathEvalBase);
        static_cast<Derived *>(this)->post_processor(iter, particle);
    }

    // static

    double path_state_dispatch (std::size_t iter, ConstSingleParticle<T> csp,
            double (*) (std::size_t, ConstSingleParticle<T>))
    {return Derived::path_state(iter, csp);}

    double path_grid_dispatch (std::size_t iter, const Particle<T> &particle,
            double (*) (std::size_t, const Particle<T> &))
    {return Derived::path_grid(iter, particle);}

    void pre_processor_dispatch (std::size_t iter, const Particle<T> &particle,
            void (*) (std::size_t, const Particle<T> &))
    {Derived::pre_processor(iter, particle);}

    void post_processor_dispatch (std::size_t iter,
            const Particle<T> &particle,
            void (*) (std::size_t, const Particle<T> &))
    {Derived::post_processor(iter, particle);}

    // base

    double path_state_dispatch (std::size_t, ConstSingleParticle<T>,
            double (PathEvalBase::*) (std::size_t, ConstSingleParticle<T>))
    {VSMC_STATIC_ASSERT_NO_IMPL(path_state); return 0;}

    double path_grid_dispatch (std::size_t, const Particle<T> &,
            double (PathEvalBase::*) (std::size_t, const Particle<T> &))
    {VSMC_STATIC_ASSERT_NO_IMPL(path_grid); return 0;}

    void pre_processor_dispatch (std::size_t, const Particle<T> &,
            void (PathEvalBase::*) (std::size_t, const Particle<T> &)) {}

    void post_processor_dispatch (std::size_t, const Particle<T> &,
            void (PathEvalBase::*) (std::size_t, const Particle<T> &)) {}
}; // class PathEvalBase

/// \brief Path evalution base dispatch class
/// \ingroup SMP
template <typename T>
class PathEvalBase<T, Virtual>
{
    public :

    virtual double path_state (std::size_t, ConstSingleParticle<T>) = 0;
    virtual double path_grid (std::size_t, const Particle<T> &) = 0;
    virtual void pre_processor (std::size_t, const Particle<T> &) {}
    virtual void post_processor (std::size_t, const Particle<T> &) {}

    protected :

    PathEvalBase () {}
    PathEvalBase (const PathEvalBase<T, Virtual> &) {}
    PathEvalBase<T, Virtual> &operator=
        (const PathEvalBase<T, Virtual> &) {return *this;}
    virtual ~PathEvalBase () {}
}; // class PathEval<T, Virtual>

} // namespace vsmc

#endif // VSMC_SMP_BASE_HPP
