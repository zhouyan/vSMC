#ifndef VSMC_SMP_BASE_HPP
#define VSMC_SMP_BASE_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

namespace internal {

template <std::size_t Dim>
class StateBaseDim
{
    public :

    static VSMC_CONSTEXPR std::size_t dim () {return Dim;}
};

template <>
class StateBaseDim<Dynamic>
{
    public :

    StateBaseDim () : dim_(Dynamic) {}

    std::size_t dim () const {return dim_;}

    void resize_dim (std::size_t dim) {dim_ = dim;}

    private :

    std::size_t dim_;
};

} // namespace vsmc::internal

/// \brief Particle::value_type subtype
/// \ingroup Base
template <std::size_t Dim, typename T>
class StateBase : public internal::StateBaseDim<Dim>
{
    public :

    typedef VSMC_SIZE_TYPE size_type;
    typedef T state_type;
    explicit StateBase (size_type N) : size_(N), state_(N * Dim) {}

    template <typename IntType>
    void copy (size_type N, const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_STATE_COPY_SIZE_MISMATCH(Base);

        for (size_type to = 0; to != N; ++to)
            this->copy_particle(copy_from[to], to);
    }

    void resize_dim (std::size_t dim)
    {
        VSMC_STATIC_ASSERT_DYNAMIC_DIM_RESIZE(Base);

        internal::StateBaseDim<Dim>::resize_dim(dim);
        state_.resize(dim * size_);
    }

    size_type size () const
    {
        return size_;
    }

    state_type &state (size_type id, std::size_t pos)
    {
        return state_[id * this->dim() + pos];
    }

    const state_type &state (size_type id, std::size_t pos) const
    {
        return state_[id * this->dim() + pos];
    }

    template <typename OutputIter>
    OutputIter read_state (std::size_t pos, OutputIter first) const
    {
        const T *siter = &state_[pos];
        for (size_type i = 0; i != size_; ++i, ++first, siter += this->dim())
            *first = *siter;

        return first;
    }

    template <typename OutputIter>
    void read_state_matrix (OutputIter *first) const
    {
        for (std::size_t d = 0; d != this->dim(); ++d)
            read_state(d, first[d]);
    }

    template <typename OutputIter>
    OutputIter read_state_matrix (MatrixOrder order, OutputIter first) const
    {
        VSMC_RUNTIME_ASSERT_MATRIX_ORDER(order, StateBase::read_state_matrix);

        if (order == ColMajor)
            for (std::size_t d = 0; d != this->dim(); ++d)
                first = read_state(d, first);

        if (order == RowMajor)
            for (std::size_t i = 0; i != state_.size(); ++i, ++first)
                *first = state_[i];

        return first;
    }

    template <typename OutputStream>
    OutputStream &print (OutputStream &os, std::size_t iter = 0,
            char sepchar = ' ', char eolchar = '\n') const
    {
        for (size_type i = 0; i != size_; ++i) {
            os << iter << sepchar;
            for (std::size_t d = 0; d != this->dim() - 1; ++d)
                os << state(i, d) << sepchar;
            os << state(i, this->dim() - 1) << eolchar;
        }

        return os;
    }

    protected :

    void copy_particle (size_type from, size_type to)
    {
        if (from != to)
            for (std::size_t d = 0; d != this->dim(); ++d)
                state(to, d) = state(from, d);
    }

    private :

    size_type size_;
    std::vector<T> state_;
}; // class StateBase

/// \brief Base Initialize class
/// \ingroup Base
template <typename T, typename Derived>
class InitializeBase
{
    protected :

    InitializeBase () {}
    InitializeBase (const InitializeBase<T, Derived> &) {}
    InitializeBase<T, Derived> &operator=
        (const InitializeBase<T, Derived> &) {return *this;}
    VSMC_SMP_BASE_DESTRUCTOR_PREFIX ~InitializeBase () {}

    std::size_t initialize_state (SingleParticle<T> sp)
    {
        return initialize_state_dispatch(sp, &Derived::initialize_state);
    }

    void initialize_param (Particle<T> &particle, void *param)
    {
        initialize_param_dispatch(particle, param, &Derived::initialize_param);
    }

    void pre_processor (Particle<T> &particle)
    {
        pre_processor_dispatch(particle, &Derived::pre_processor);
    }

    void post_processor (Particle<T> &particle)
    {
        post_processor_dispatch(particle, &Derived::post_processor);
    }

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
    {
        return Derived::initialize_state(sp);
    }

    void initialize_param_dispatch (Particle<T> &particle, void *param,
            void (*) (Particle<T> &, void *))
    {
        Derived::initialize_param(particle, param);
    }

    void pre_processor_dispatch (Particle<T> &particle,
            void (*) (Particle<T> &))
    {
        Derived::pre_processor(particle);
    }

    void post_processor_dispatch (Particle<T> &particle,
            void (*) (Particle<T> &))
    {
        Derived::post_processor(particle);
    }

    // base

    std::size_t initialize_state_dispatch (SingleParticle<T>,
            std::size_t (InitializeBase::*) (SingleParticle<T>))
    { VSMC_STATIC_ASSERT_NO_IMPL(initialize_state); return 0;}

    void initialize_param_dispatch (Particle<T> &, void *,
            void (InitializeBase::*) (Particle<T> &, void *)) {}

    void pre_processor_dispatch (Particle<T> &,
            void (InitializeBase::*) (Particle<T> &)) {}

    void post_processor_dispatch (Particle<T> &,
            void (InitializeBase::*) (Particle<T> &)) {}
}; // class InitializeBase

/// \brief Base Initialize class with virtual interface
/// \ingroup Base
template <typename T>
class InitializeBase<T, VBase>
{
    protected :

    InitializeBase () {}
    InitializeBase (const InitializeBase<T, VBase> &) {}
    InitializeBase<T, VBase> &operator=
        (const InitializeBase<T, VBase> &) {return *this;}
    virtual ~InitializeBase () {}
    virtual std::size_t initialize_state (SingleParticle<T>) = 0;
    virtual void initialize_param (Particle<T> &, void *) {}
    virtual void pre_processor (Particle<T> &) {}
    virtual void post_processor (Particle<T> &) {}
}; // class InitializeBase<T, VBase>

/// \brief Base Move class
/// \ingroup Base
template <typename T, typename Derived>
class MoveBase
{
    protected :

    MoveBase () {}
    MoveBase (const MoveBase<T, Derived> &) {}
    MoveBase<T, Derived> &operator=
        (const MoveBase<T, Derived> &) {return *this;}
    VSMC_SMP_BASE_DESTRUCTOR_PREFIX ~MoveBase () {}

    std::size_t move_state (std::size_t iter, SingleParticle<T> sp)
    {
        return move_state_dispatch(iter, sp, &Derived::move_state);
    }

    void pre_processor (std::size_t iter, Particle<T> &particle)
    {
        pre_processor_dispatch(iter, particle, &Derived::pre_processor);
    }

    void post_processor (std::size_t iter, Particle<T> &particle)
    {
        post_processor_dispatch(iter, particle, &Derived::post_processor);
    }

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
    {
        return Derived::move_state(iter, sp);
    }

    void pre_processor_dispatch (std::size_t iter, Particle<T> &particle,
            void (*) (std::size_t, Particle<T> &))
    {
        Derived::pre_processor(iter, particle);
    }

    void post_processor_dispatch (std::size_t iter, Particle<T> &particle,
            void (*) (std::size_t, Particle<T> &))
    {
        Derived::post_processor(iter, particle);
    }

    // base

    std::size_t move_state_dispatch (std::size_t, SingleParticle<T>,
            std::size_t (MoveBase::*) (std::size_t, SingleParticle<T>))
    { VSMC_STATIC_ASSERT_NO_IMPL(move_state); return 0;}

    void pre_processor_dispatch (std::size_t, Particle<T> &,
            void (MoveBase::*) (std::size_t, Particle<T> &)) {}

    void post_processor_dispatch (std::size_t, Particle<T> &,
            void (MoveBase::*) (std::size_t, Particle<T> &)) {}
}; // class MoveBase

/// \brief Base Move class with virtual interface
/// \ingroup Base
template <typename T>
class MoveBase<T, VBase>
{
    protected :

    MoveBase () {}
    MoveBase (const MoveBase<T, VBase> &) {}
    MoveBase<T, VBase> &operator=
        (const MoveBase<T, VBase> &) {return *this;}
    virtual ~MoveBase () {}
    virtual std::size_t move_state (std::size_t, SingleParticle<T>) = 0;
    virtual void pre_processor (std::size_t, Particle<T> &) {}
    virtual void post_processor (std::size_t, Particle<T> &) {}
}; // class MoveBase<T, VBase>

/// \brief Base Monitor evaluation class
/// \ingroup Base
template <typename T, typename Derived>
class MonitorEvalBase
{
    protected :

    MonitorEvalBase () {}
    MonitorEvalBase (const MonitorEvalBase<T, Derived> &) {}
    MonitorEvalBase<T, Derived> &operator=
        (const MonitorEvalBase<T, Derived> &) {return *this;}
    VSMC_SMP_BASE_DESTRUCTOR_PREFIX ~MonitorEvalBase () {}

    void monitor_state (std::size_t iter, std::size_t dim,
            ConstSingleParticle<T> csp, double *res)
    {
        monitor_state_dispatch(iter, dim, csp, res, &Derived::monitor_state);
    }

    void pre_processor (std::size_t iter, const Particle<T> &particle)
    {
        pre_processor_dispatch(iter, particle, &Derived::pre_processor);
    }

    void post_processor (std::size_t iter, const Particle<T> &particle)
    {
        post_processor_dispatch(iter, particle, &Derived::post_processor);
    }

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
    {
        Derived::monitor_state(iter, dim, csp, res);
    }

    void pre_processor_dispatch (std::size_t iter, const Particle<T> &particle,
            void (*) (std::size_t, const Particle<T> &))
    {
        Derived::pre_processor(iter, particle);
    }

    void post_processor_dispatch (std::size_t iter,
            const Particle<T> &particle,
            void (*) (std::size_t, const Particle<T> &))
    {
        Derived::post_processor(iter, particle);
    }

    // base

    void monitor_state_dispatch (std::size_t, std::size_t dim,
            ConstSingleParticle<T>, double *res,
            void (MonitorEvalBase::*)
            (std::size_t, std::size_t, ConstSingleParticle<T>, double *))
    { VSMC_STATIC_ASSERT_NO_IMPL(monitor_state); }

    void pre_processor_dispatch (std::size_t, const Particle<T> &,
            void (MonitorEvalBase::*) (std::size_t, const Particle<T> &)) {}

    void post_processor_dispatch (std::size_t, const Particle<T> &,
            void (MonitorEvalBase::*) (std::size_t, const Particle<T> &)) {}
}; // class MonitorBase

/// \brief Base Monitor evaluation class with virtual interface
/// \ingroup Base
template <typename T>
class MonitorEvalBase<T, VBase>
{
    protected :

    MonitorEvalBase () {}
    MonitorEvalBase (const MonitorEvalBase<T, VBase> &) {}
    MonitorEvalBase<T, VBase> &operator=
        (const MonitorEvalBase<T, VBase> &) {return *this;}
    virtual ~MonitorEvalBase () {}
    virtual void monitor_state (std::size_t, std::size_t,
            ConstSingleParticle<T>, double *) = 0;
    virtual void pre_processor (std::size_t, const Particle<T> &) {}
    virtual void post_processor (std::size_t, const Particle<T> &) {}
}; // class MonitorEvalBase<T, VBase>

/// \brief Base Path evaluation class
/// \ingroup Base
template <typename T, typename Derived>
class PathEvalBase
{
    protected :

    PathEvalBase () {}
    PathEvalBase (const PathEvalBase<T, Derived> &) {}
    PathEvalBase<T, Derived> &operator=
        (const PathEvalBase<T, Derived> &) {return *this;}
    VSMC_SMP_BASE_DESTRUCTOR_PREFIX ~PathEvalBase () {}

    double path_state (std::size_t iter, ConstSingleParticle<T> csp)
    {
        return path_state_dispatch(iter, csp, &Derived::path_state);
    }

    double path_grid (std::size_t iter, const Particle<T> &particle)
    {
        return path_grid_dispatch(iter, particle, &Derived::path_grid);
    }

    void pre_processor (std::size_t iter, const Particle<T> &particle)
    {
        pre_processor_dispatch(iter, particle, &Derived::pre_processor);
    }

    void post_processor (std::size_t iter, const Particle<T> &particle)
    {
        post_processor_dispatch(iter, particle, &Derived::post_processor);
    }

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
    {
        return Derived::path_state(iter, csp);
    }

    double path_grid_dispatch (std::size_t iter, const Particle<T> &particle,
            double (*) (std::size_t, const Particle<T> &))
    {
        return Derived::path_grid(iter, particle);
    }

    void pre_processor_dispatch (std::size_t iter, const Particle<T> &particle,
            void (*) (std::size_t, const Particle<T> &))
    {
        Derived::pre_processor(iter, particle);
    }

    void post_processor_dispatch (std::size_t iter,
            const Particle<T> &particle,
            void (*) (std::size_t, const Particle<T> &))
    {
        Derived::post_processor(iter, particle);
    }

    // base

    double path_state_dispatch (std::size_t, ConstSingleParticle<T>,
            double (PathEvalBase::*) (std::size_t, ConstSingleParticle<T>))
    { VSMC_STATIC_ASSERT_NO_IMPL(path_state); return 0; }

    double path_grid_dispatch (std::size_t, const Particle<T> &,
            double (PathEvalBase::*) (std::size_t, const Particle<T> &))
    { VSMC_STATIC_ASSERT_NO_IMPL(path_grid); return 0; }

    void pre_processor_dispatch (std::size_t, const Particle<T> &,
            void (PathEvalBase::*) (std::size_t, const Particle<T> &)) {}

    void post_processor_dispatch (std::size_t, const Particle<T> &,
            void (PathEvalBase::*) (std::size_t, const Particle<T> &)) {}
}; // class PathEvalBase

/// \brief Base Path evaluation class with virtual interface
/// \ingroup Base
template <typename T>
class PathEvalBase<T, VBase>
{
    protected :

    PathEvalBase () {}
    PathEvalBase (const PathEvalBase<T, VBase> &) {}
    PathEvalBase<T, VBase> &operator=
        (const PathEvalBase<T, VBase> &) {return *this;}
    virtual ~PathEvalBase () {}
    virtual double path_state (std::size_t, ConstSingleParticle<T>) = 0;
    virtual double path_grid (std::size_t, const Particle<T> &) = 0;
    virtual void pre_processor (std::size_t, const Particle<T> &) {}
    virtual void post_processor (std::size_t, const Particle<T> &) {}
}; // class PathEval<T, VBase>

} // namespace vsmc

#endif // VSMC_SMP_BASE_HPP
