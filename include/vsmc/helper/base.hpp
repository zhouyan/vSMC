#ifndef VSMC_HELPER_BASE_HPP
#define VSMC_HELPER_BASE_HPP

#include <vsmc/internal/common.hpp>

#define VSMC_RUNTIME_ASSERT_DERIVED_BASE(basename) \
    VSMC_RUNTIME_ASSERT((dynamic_cast<Derived *>(this)), ( \
                "YOU DERIVED FROM " #basename \
                " WITH INCORRECT **Derived** TEMPLATE PARAMTER")); \

#ifdef NDEBUG
#define VSMC_HELPER_BASE_DESTRUCTOR_PREFIX
#else
#define VSMC_HELPER_BASE_DESTRUCTOR_PREFIX virtual
#endif

namespace vsmc { namespace traits {

template <template <unsigned, typename> class State, typename D>
class IsBaseOfState
{
    private :

    struct char2 {char c1; char c2;};
    typedef typename cxx11::remove_cv<D>::type derived_type;

    template <unsigned Dim, typename T>
    static char test (State<Dim, T> *);
    static char2 test (...);

    public :

   enum {value = sizeof(test(static_cast<derived_type *>(0))) == sizeof(char)};
};

} // namespace vsmc::traits

/// \brief Particle::value_type subtype
/// \ingroup Base
template <unsigned Dim, typename T>
class StateBase
{
    public :

    typedef VSMC_SIZE_TYPE size_type;
    typedef T state_type;
    explicit StateBase (size_type N) : size_(N), dim_(Dim), state_(N * Dim) {}

    template <typename IntType>
    void copy (const IntType *copy_from)
    {
        for (size_type to = 0; to != size_; ++to)
            this->copy_particle(copy_from[to], to);
    }

    unsigned dim () const
    {
        return dim_;
    }

    void resize_dim (unsigned dim)
    {
        VSMC_STATIC_ASSERT((Dim == Dynamic),
                USE_METHOD_resize_dim_WITH_A_FIXED_SIZE_StateBase_OBJECT);

        state_.resize(dim * size_);
        dim_ = dim;
    }

    size_type size () const
    {
        return size_;
    }

    state_type &state (size_type id, unsigned pos)
    {
        return state_[id * dim_ + pos];
    }

    const state_type &state (size_type id, unsigned pos) const
    {
        return state_[id * dim_ + pos];
    }

    state_type *state (size_type id)
    {
        return &state_[id * dim_];
    }

    const state_type *state (size_type id) const
    {
        return &state_[id * dim_];
    }

    template <typename OutputIter>
    OutputIter read_state (unsigned pos, OutputIter first) const
    {
        const T *siter = &state_[pos];
        for (size_type i = 0; i != size_; ++i, ++first, siter += dim_)
            *first = *siter;

        return first;
    }

    template <typename OutputIter>
    void read_state_matrix (OutputIter *first) const
    {
        for (unsigned d = 0; d != dim_; ++d)
            read_state(d, first[d]);
    }

    template <typename OutputIter>
    OutputIter read_state_matrix (MatrixOrder order, OutputIter first) const
    {
        VSMC_RUNTIME_ASSERT((order == ColumnMajor || order == RowMajor),
                "CALL **StateBase::read_state_matrix** with and INVALID "
                "MatrixOrder");

        if (order == ColumnMajor)
            for (unsigned d = 0; d != dim_; ++d)
                first = read_state(d, first);

        if (order == RowMajor)
            first = std::copy(state_.begin(), state_.end(), first);

        return first;
    }

    protected :

    void copy_particle (size_type from, size_type to)
    {
        if (from != to)
            std::copy(state(from), state(from) + dim_, state(to));
    }

    private :

    size_type size_;
    unsigned dim_;
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
    VSMC_HELPER_BASE_DESTRUCTOR_PREFIX ~InitializeBase () {}

    unsigned initialize_state (SingleParticle<T> part)
    {
        return initialize_state_dispatch(part, &Derived::initialize_state);
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

    template <typename D>
    unsigned initialize_state_dispatch (SingleParticle<T> part,
            unsigned (D::*) (SingleParticle<T>))
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(InitializeBase);
        return static_cast<Derived *>(this)->initialize_state(part);
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

    unsigned initialize_state_dispatch (SingleParticle<T> part,
            unsigned (*) (SingleParticle<T>))
    {
        return Derived::initialize_state(part);
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

    unsigned initialize_state_dispatch (SingleParticle<T>,
            unsigned (InitializeBase::*) (SingleParticle<T>)) {return 0;}

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
    virtual unsigned initialize_state (SingleParticle<T>) {return 0;}
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
    VSMC_HELPER_BASE_DESTRUCTOR_PREFIX ~MoveBase () {}

    unsigned move_state (unsigned iter, SingleParticle<T> part)
    {
        return move_state_dispatch(iter, part, &Derived::move_state);
    }

    void pre_processor (unsigned iter, Particle<T> &particle)
    {
        pre_processor_dispatch(iter, particle, &Derived::pre_processor);
    }

    void post_processor (unsigned iter, Particle<T> &particle)
    {
        post_processor_dispatch(iter, particle, &Derived::post_processor);
    }

    private :

    template <typename D>
    unsigned move_state_dispatch (unsigned iter, SingleParticle<T> part,
            unsigned (D::*) (unsigned, SingleParticle<T>))
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(MoveBase);
        return static_cast<Derived *>(this)->move_state(iter, part);
    }

    template <typename D>
    void pre_processor_dispatch (unsigned iter, Particle<T> &particle,
            void (D::*) (unsigned, Particle<T> &))
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(MoveBase);
        static_cast<Derived *>(this)->pre_processor(iter, particle);
    }

    template <typename D>
    void post_processor_dispatch (unsigned iter, Particle<T> &particle,
            void (D::*) (unsigned, Particle<T> &))
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(MoveBase);
        static_cast<Derived *>(this)->post_processor(iter, particle);
    }

    unsigned move_state_dispatch (unsigned iter, SingleParticle<T> part,
            unsigned (*) (unsigned, SingleParticle<T>))
    {
        return Derived::move_state(iter, part);
    }

    void pre_processor_dispatch (unsigned iter, Particle<T> &particle,
            void (*) (unsigned, Particle<T> &))
    {
        Derived::pre_processor(iter, particle);
    }

    void post_processor_dispatch (unsigned iter, Particle<T> &particle,
            void (*) (unsigned, Particle<T> &))
    {
        Derived::post_processor(iter, particle);
    }

    unsigned move_state_dispatch (unsigned, SingleParticle<T>,
            unsigned (MoveBase::*) (unsigned, SingleParticle<T>)) {return 0;}

    void pre_processor_dispatch (unsigned, Particle<T> &,
            void (MoveBase::*) (unsigned, Particle<T> &)) {}

    void post_processor_dispatch (unsigned, Particle<T> &,
            void (MoveBase::*) (unsigned, Particle<T> &)) {}
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
    virtual unsigned move_state (unsigned, SingleParticle<T>) {return 0;}
    virtual void pre_processor (unsigned, Particle<T> &) {}
    virtual void post_processor (unsigned, Particle<T> &) {}
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
    VSMC_HELPER_BASE_DESTRUCTOR_PREFIX ~MonitorEvalBase () {}

    void monitor_state (unsigned iter, unsigned dim,
            ConstSingleParticle<T> part, double *res)
    {
        monitor_state_dispatch(iter, dim, part, res, &Derived::monitor_state);
    }

    void pre_processor (unsigned iter, const Particle<T> &particle)
    {
        pre_processor_dispatch(iter, particle, &Derived::pre_processor);
    }

    void post_processor (unsigned iter, const Particle<T> &particle)
    {
        post_processor_dispatch(iter, particle, &Derived::post_processor);
    }

    private :

    template <typename D>
    void monitor_state_dispatch (unsigned iter, unsigned dim,
            ConstSingleParticle<T> part, double *res,
            void (D::*) (unsigned, unsigned, ConstSingleParticle<T>, double *))
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(MonitorEvalBase);
        static_cast<Derived *>(this)->monitor_state(iter, dim, part, res);
    }

    template <typename D>
    void pre_processor_dispatch (unsigned iter, const Particle<T> &particle,
            void (D::*) (unsigned, const Particle<T> &))
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(MonitorEvalBase);
        static_cast<Derived *>(this)->pre_processor(iter, particle);
    }

    template <typename D>
    void post_processor_dispatch (unsigned iter, const Particle<T> &particle,
            void (D::*) (unsigned, const Particle<T> &))
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(MonitorEvalBase);
        static_cast<Derived *>(this)->post_processor(iter, particle);
    }

    void monitor_state_dispatch (unsigned iter, unsigned dim,
            ConstSingleParticle<T> part, double *res,
            void (*) (unsigned, unsigned, ConstSingleParticle<T>, double *))
    {
        Derived::monitor_state(iter, dim, part, res);
    }

    void pre_processor_dispatch (unsigned iter, const Particle<T> &particle,
            void (*) (unsigned, const Particle<T> &))
    {
        Derived::pre_processor(iter, particle);
    }

    void post_processor_dispatch (unsigned iter, const Particle<T> &particle,
            void (*) (unsigned, const Particle<T> &))
    {
        Derived::post_processor(iter, particle);
    }

    void monitor_state_dispatch (unsigned, unsigned dim,
            ConstSingleParticle<T>, double *res,
            void (MonitorEvalBase::*)
            (unsigned, unsigned, ConstSingleParticle<T>, double *)) {}

    void pre_processor_dispatch (unsigned, const Particle<T> &,
            void (MonitorEvalBase::*) (unsigned, const Particle<T> &)) {}

    void post_processor_dispatch (unsigned, const Particle<T> &,
            void (MonitorEvalBase::*) (unsigned, const Particle<T> &)) {}
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
    virtual void monitor_state (unsigned, unsigned, ConstSingleParticle<T>,
            double *) {}
    virtual void pre_processor (unsigned, const Particle<T> &) {}
    virtual void post_processor (unsigned, const Particle<T> &) {}
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
    VSMC_HELPER_BASE_DESTRUCTOR_PREFIX ~PathEvalBase () {}

    double path_state (unsigned iter, ConstSingleParticle<T> part)
    {
        return path_state_dispatch(iter, part, &Derived::path_state);
    }

    double path_width (unsigned iter, const Particle<T> &particle)
    {
        return path_width_dispatch(iter, particle, &Derived::path_width);
    }

    void pre_processor (unsigned iter, const Particle<T> &particle)
    {
        pre_processor_dispatch(iter, particle, &Derived::pre_processor);
    }

    void post_processor (unsigned iter, const Particle<T> &particle)
    {
        post_processor_dispatch(iter, particle, &Derived::post_processor);
    }

    private :

    template <typename D>
    double path_state_dispatch (unsigned iter, ConstSingleParticle<T> part,
            double (D::*) (unsigned, ConstSingleParticle<T>))
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(PathEvalBase);
        return static_cast<Derived *>(this)->path_state(iter, part);
    }

    template <typename D>
    double path_width_dispatch (unsigned iter, const Particle<T> &particle,
            double (D::*) (unsigned, const Particle<T> &))
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(PathEvalBase);
        return static_cast<Derived *>(this)->path_width(iter, particle);
    }

    template <typename D>
    void pre_processor_dispatch (unsigned iter, const Particle<T> &particle,
            void (D::*) (unsigned, const Particle<T> &))
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(PathEvalBase);
        static_cast<Derived *>(this)->pre_processor(iter, particle);
    }

    template <typename D>
    void post_processor_dispatch (unsigned iter, const Particle<T> &particle,
            void (D::*) (unsigned, const Particle<T> &))
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(PathEvalBase);
        static_cast<Derived *>(this)->post_processor(iter, particle);
    }

    double path_state_dispatch (unsigned iter, ConstSingleParticle<T> part,
            double (*) (unsigned, ConstSingleParticle<T>))
    {
        return Derived::path_state(iter, part);
    }

    double path_width_dispatch (unsigned iter, const Particle<T> &particle,
            double (*) (unsigned, const Particle<T> &))
    {
        return Derived::path_width(iter, particle);
    }

    void pre_processor_dispatch (unsigned iter, const Particle<T> &particle,
            void (*) (unsigned, const Particle<T> &))
    {
        Derived::pre_processor(iter, particle);
    }

    void post_processor_dispatch (unsigned iter, const Particle<T> &particle,
            void (*) (unsigned, const Particle<T> &))
    {
        Derived::post_processor(iter, particle);
    }

    double path_state_dispatch (unsigned, ConstSingleParticle<T>,
            double (PathEvalBase::*) (unsigned, ConstSingleParticle<T>))
    {return 0;}

    double path_width_dispatch (unsigned, const Particle<T> &,
            double (PathEvalBase::*) (unsigned, const Particle<T> &))
    {return 0;}

    void pre_processor_dispatch (unsigned, const Particle<T> &,
            void (PathEvalBase::*) (unsigned, const Particle<T> &)) {}

    void post_processor_dispatch (unsigned, const Particle<T> &,
            void (PathEvalBase::*) (unsigned, const Particle<T> &)) {}
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
    virtual double path_state (unsigned, ConstSingleParticle<T>) {return 0;}
    virtual double path_width (unsigned, const Particle<T> &) {return 0;}
    virtual void pre_processor (unsigned, const Particle<T> &) {}
    virtual void post_processor (unsigned, const Particle<T> &) {}
}; // class PathEval<T, VBase>

} // namespace vsmc

#endif // VSMC_HELPER_BASE_HPP
