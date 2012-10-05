#ifndef VSMC_HELPER_BASE_HPP
#define VSMC_HELPER_BASE_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/helper/timer/null_timer.hpp>

#define VSMC_RUNTIME_ASSERT_DERIVED_BASE(basename) \
    VSMC_RUNTIME_ASSERT((dynamic_cast<Derived *>(this)), ( \
                "YOU DERIVED FROM " #basename \
                " WITH INCORRECT **Derived** TEMPLATE PARAMTER")); \

#define VSMC_STATIC_ASSERT_STATE_TYPE(base, derived, user) \
    VSMC_STATIC_ASSERT((vsmc::IsBaseOfState<base, derived>::value), \
            USE_##user##_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_##base)

#ifdef NDEBUG
#define VSMC_VIRTUAL_BASE_DESTRUCTOR
#else
#define VSMC_VIRTUAL_BASE_DESTRUCTOR virtual
#endif

namespace vsmc {

template <template <unsigned, typename, typename> class State, typename D>
class IsBaseOfState
{
    private :

    struct char2 {char c1; char c2;};
    typedef typename cxx11::remove_cv<D>::type derived_type;

    template <unsigned Dim, typename T, typename Timer>
    static char test (State<Dim, T, Timer> *);
    static char2 test (...);

    public :

    static const bool value =
        sizeof(test(static_cast<derived_type *>(0))) == sizeof(char);
};

/// \brief Particle::value_type subtype
/// \ingroup Helper
///
/// \tparam Dim The dimension of the state parameter vector
/// \tparam T The type of the value of the state parameter vector
template <unsigned Dim, typename T, typename Timer>
class StateBase
{
    public :

    /// The type of the number of particles
    typedef VSMC_SIZE_TYPE size_type;

    /// The type of state parameters
    typedef T state_type;

    /// The type of the timer
    typedef Timer timer_type;

    /// The dimension of the problem
    unsigned dim ()
    {
        return dim_;
    }

    /// Resize the dimension of the problem
    void resize_dim (unsigned dim)
    {
        VSMC_STATIC_ASSERT((Dim == Dynamic),
                USE_METHOD_resize_dim_WITH_A_FIXED_SIZE_StateBase_OBJECT);

        state_.resize(dim * size_);
        dim_ = dim;
    }

    /// The number of particles
    size_type size () const
    {
        return size_;
    }

    /// The timer
    const timer_type &timer () const
    {
        return timer_;
    }

    /// \brief Read and write access to a signle particle state
    ///
    /// \param id The position of the particle
    /// \param pos The position of the parameter in the state array
    ///
    /// \return A reference to the parameter at position pos of the states
    /// array of the particle at position id
    state_type &state (size_type id, unsigned pos)
    {
        return state_[id * dim_ + pos];
    }

    /// Read only access to a signle particle state
    const state_type &state (size_type id, unsigned pos) const
    {
        return state_[id * dim_ + pos];
    }

    /// \brief Read and write access to the array of a single particle states
    ///
    /// \param id The position of the particle, 0 to size() - 1
    state_type *state (size_type id)
    {
        return &state_[id * dim_];
    }

    /// Read only access to the array of a single particle states
    const state_type *state (size_type id) const
    {
        return &state_[id * dim_];
    }

    protected :

    explicit StateBase (size_type N) : size_(N), dim_(Dim), state_(N * Dim) {}

    void copy_particle (size_type from, size_type to)
    {
        if (from != to)
            std::copy(state(from), state(from) + dim_, state(to));
    }

    private :

    size_type size_;
    unsigned dim_;
    std::vector<T> state_;
    timer_type timer_;
}; // class StateBase

/// \brief A const variant to SingleParticle
/// \ingroup Helper
///
/// \tparam T A subtype of StateBase
template <typename T>
class ConstSingleParticle
{
    public :

    typedef T value_type;
    typedef typename T::state_type state_type;
    typedef typename Particle<T>::size_type size_type;
    typedef typename Particle<T>::rng_type rng_type;

    ConstSingleParticle (size_type id, const Particle<T> *particle) :
        id_(id), particle_(particle)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateBase, T, ConstSingleParticle);
        VSMC_RUNTIME_ASSERT(particle_,
                ("A **ConstSingleParticle** object "
                 "is contructed with 0 **Particle** pointer"));
        VSMC_RUNTIME_ASSERT((id_ >= 0 && id_ < particle_->size()),
                ("A **ConstSignleParticle** object "
                 "is contructed with an out of range id"));
    }

    ConstSingleParticle (const ConstSingleParticle<T> &other) :
        id_(other.id_), particle_(other.particle_) {}

    ConstSingleParticle (const SingleParticle<T> &other) :
        id_(other.id_), particle_(other.particle_) {}

    const ConstSingleParticle &operator= (const ConstSingleParticle<T> &other)
    {
        id_ = other.id_;
        particle_ = other.particle_;
    }

    size_type id () const
    {
        return id_;
    }

    const state_type &state (unsigned pos) const
    {
        return particle_->value().state(id_, pos);
    }

    const state_type *state () const
    {
        return particle_->value().state(id_);
    }

    double weight () const
    {
        return particle_->weight()[id_];
    }

    double log_weight () const
    {
        return particle_->log_weight()[id_];
    }

    const Particle<T> &particle () const
    {
        return *particle_;
    }

    private :

    const size_type id_;
    const Particle<T> *const particle_;
}; // class ConstSingleParticle

/// \brief A thin wrapper over a complete Particle
/// \ingroup Helper
///
/// \tparam T A subtype of StateBase
template <typename T>
class SingleParticle
{
    public :

    /// The type of the particle values
    typedef T value_type;

    /// The type of the state parameters
    typedef typename T::state_type state_type;

    /// The type of the number of particles
    typedef typename Particle<T>::size_type size_type;

    /// The type of RNG engine
    typedef typename Particle<T>::rng_type rng_type;

    /// \brief Construct a SingleParticle with a given id and a particle set
    ///
    /// \param id The id of the particle, start with zero
    /// \param particle A pointer to the particle set
    ///
    /// \note particle cannot be a const pointer. Unless one explicitly cast
    /// out the constness of the pointer, otherwise this shall results in a
    /// compile time error. vsmc itself will never do this. When one need
    /// access to a const particle set, use the ConstSingleParticle variant
    /// which is exaclty for this purpose, which does not have the mutator like
    /// rng() and write access to states.
    SingleParticle (size_type id, Particle<T> *particle) :
        id_(id), particle_(particle)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateBase, T, SingleParticle);
        VSMC_RUNTIME_ASSERT(particle_,
                ("A **SingleParticle** object "
                 "is contructed with 0 **Particle** pointer"));
        VSMC_RUNTIME_ASSERT((id_ >= 0 && id_ < particle_->size()),
                ("A **SignleParticle** object "
                 "is contructed with an out of range id"));
    }

    SingleParticle (const SingleParticle<T> &other) :
        id_(other.id_), particle_(other.particle_) {}

    const SingleParticle &operator= (const SingleParticle<T> &other)
    {
        id_ = other.id_;
        particle_ = other.particle_;
    }

    /// The id of the particle
    size_type id () const
    {
        return id_;
    }

    /// \brief Read and write access to a single parameter
    ///
    /// \param pos The position of the parameter
    state_type &state (unsigned pos)
    {
        return particle_->value().state(id_, pos);
    }

    /// \brief Read only access to a single parameter
    ///
    /// \param pos The position of the parameter
    const state_type &state (unsigned pos) const
    {
        return particle_->value().state(id_, pos);
    }

    /// Read and write access to all parameters through pointer
    state_type *state ()
    {
        return particle_->value().state(id_);
    }

    /// Read only access to all parameters through pointer
    const state_type *state () const
    {
        return particle_->value().state(id_);
    }

    /// The weight of this particle
    double weight () const
    {
        return particle_->weight()[id_];
    }

    /// The log weight of this particle
    double log_weight () const
    {
        return particle_->log_weight()[id_];
    }

    /// Read only access to all particles
    const Particle<T> &particle () const
    {
        return *particle_;
    }

    /// A unique RNG engine for this particle
    rng_type &rng ()
    {
        return particle_->rng(id_);
    }

    operator ConstSingleParticle<T>()
    {
        return ConstSingleParticle<T>(id_, particle_);
    }

    private :

    const size_type id_;
    Particle<T> *const particle_;
}; // class SingleParticle

/// \brief Base Initialize class
/// \ingroup Helper
///
/// \tparam T Particle::value_type
/// \tparam Derived InitializeBase<T, Derived> subclass or a subtype with all
/// and only static member functions
template <typename T, typename Derived>
class InitializeBase
{
    protected :

    InitializeBase () {}
    InitializeBase (const InitializeBase<T, Derived> &) {}
    const InitializeBase<T, Derived> &operator=
        (const InitializeBase<T, Derived> &) {return *this;}
    VSMC_VIRTUAL_BASE_DESTRUCTOR ~InitializeBase () {}

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

    unsigned initialize_state_dispatch (SingleParticle<T> part,
            unsigned (InitializeBase::*) (SingleParticle<T>)) {return 0;}

    void initialize_param_dispatch (Particle<T> &particle, void *param,
            void (InitializeBase::*) (Particle<T> &, void *)) {}

    void pre_processor_dispatch (Particle<T> &particle,
            void (InitializeBase::*) (Particle<T> &)) {}

    void post_processor_dispatch (Particle<T> &particle,
            void (InitializeBase::*) (Particle<T> &)) {}
}; // class InitializeBase

/// \brief Base Initialize class with virtual interface
/// \ingroup Helper
///
/// \tparam T Particle::value_type
template <typename T>
class InitializeBase<T, VBase>
{
    public :

    virtual unsigned initialize_state (SingleParticle<T>) {return 0;}
    virtual void initialize_param (Particle<T> &, void *) {}
    virtual void post_processor (Particle<T> &) {}
    virtual void pre_processor (Particle<T> &) {}

    protected :

    InitializeBase () {}
    InitializeBase (const InitializeBase<T, VBase> &) {}
    const InitializeBase<T, VBase> &operator=
        (const InitializeBase<T, VBase> &) {return *this;}
    virtual ~InitializeBase () {}
}; // class InitializeBase<T, VBase>

/// \brief Base Move class
/// \ingroup Helper
///
/// \tparam T Particle::value_type
/// \tparam Derived MoveBase<T, Derived> subclass or a subtype with all
/// and only static member functions
template <typename T, typename Derived>
class MoveBase
{
    protected :

    MoveBase () {}
    MoveBase (const MoveBase<T, Derived> &) {}
    const MoveBase<T, Derived> &operator=
        (const MoveBase<T, Derived> &) {return *this;}
    VSMC_VIRTUAL_BASE_DESTRUCTOR ~MoveBase () {}

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

    void pre_processor_dispatch (unsigned iter, Particle<T> &,
            void (MoveBase::*) (unsigned, Particle<T> &)) {}

    void post_processor_dispatch (unsigned iter, Particle<T> &,
            void (MoveBase::*) (unsigned, Particle<T> &)) {}
}; // class MoveBase

/// \brief Base Move class with virtual interface
/// \ingroup Helper
///
/// \tparam T Particle::value_type
template <typename T>
class MoveBase<T, VBase>
{
    public :

    virtual unsigned move_state (unsigned, SingleParticle<T>) {return 0;}
    virtual void post_processor (unsigned, Particle<T> &) {}
    virtual void pre_processor (unsigned, Particle<T> &) {}

    protected :

    MoveBase () {}
    MoveBase (const MoveBase<T, VBase> &) {}
    const MoveBase<T, VBase> &operator=
        (const MoveBase<T, VBase> &) {return *this;}
    virtual ~MoveBase () {}
}; // class MoveBase<T, VBase>

/// \brief Base Monitor evaluation class
/// \ingroup Helper
///
/// \tparam T Particle::value_type
/// \tparam Derived MonitorBase<T, Derived> subclass or a subtype with all
/// and only static member functions
template <typename T, typename Derived>
class MonitorEvalBase
{
    protected :

    MonitorEvalBase () {}
    MonitorEvalBase (const MonitorEvalBase<T, Derived> &) {}
    const MonitorEvalBase<T, Derived> &operator=
        (const MonitorEvalBase<T, Derived> &) {return *this;}
    VSMC_VIRTUAL_BASE_DESTRUCTOR ~MonitorEvalBase () {}

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

    void pre_processor_dispatch (unsigned iter, const Particle<T> &,
            void (MonitorEvalBase::*) (unsigned, const Particle<T> &)) {}

    void post_processor_dispatch (unsigned iter, const Particle<T> &,
            void (MonitorEvalBase::*) (unsigned, const Particle<T> &)) {}
}; // class MonitorBase

/// \brief Base Monitor evaluation class with virtual interface
/// \ingroup Helper
///
/// \tparam T Particle::value_type
template <typename T>
class MonitorEvalBase<T, VBase>
{
    public :

    virtual void monitor_state (unsigned, unsigned, ConstSingleParticle<T>,
            double *) {}
    virtual void post_processor (unsigned, const Particle<T> &) {}
    virtual void pre_processor (unsigned, const Particle<T> &) {}

    protected :

    MonitorEvalBase () {}
    MonitorEvalBase (const MonitorEvalBase<T, VBase> &) {}
    const MonitorEvalBase<T, VBase> &operator=
        (const MonitorEvalBase<T, VBase> &) {return *this;}
    virtual ~MonitorEvalBase () {}
}; // class MonitorEvalBase<T, VBase>

/// \brief Base Path evaluation class
/// \ingroup Helper
///
/// \tparam T Particle::value_type
/// \tparam Derived PathBase<T, Derived> subclass or a subtype with all
/// and only static member functions
template <typename T, typename Derived>
class PathEvalBase
{
    protected :

    PathEvalBase () {}
    PathEvalBase (const PathEvalBase<T, Derived> &) {}
    const PathEvalBase<T, Derived> &operator=
        (const PathEvalBase<T, Derived> &) {return *this;}
    VSMC_VIRTUAL_BASE_DESTRUCTOR ~PathEvalBase () {}

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

    void pre_processor_dispatch (unsigned iter, const Particle<T> &,
            void (PathEvalBase::*) (unsigned, const Particle<T> &)) {}

    void post_processor_dispatch (unsigned iter, const Particle<T> &,
            void (PathEvalBase::*) (unsigned, const Particle<T> &)) {}
}; // class PathEvalBase

/// \brief Base Path evaluation class with virtual interface
/// \ingroup Helper
///
/// \tparam T Particle::value_type
template <typename T>
class PathEvalBase<T, VBase>
{
    public :

    virtual double path_state (unsigned, ConstSingleParticle<T>) {return 0;}
    virtual double path_width (unsigned, const Particle<T> &) {return 0;}
    virtual void post_processor (unsigned, const Particle<T> &) {}
    virtual void pre_processor (unsigned, const Particle<T> &) {}

    protected :

    PathEvalBase () {}
    PathEvalBase (const PathEvalBase<T, VBase> &) {}
    const PathEvalBase<T, VBase> &operator=
        (const PathEvalBase<T, VBase> &) {return *this;}
    virtual ~PathEvalBase () {}
}; // class PathEval<T, VBase>

} // namespace vsmc

#endif // VSMC_HELPER_BASE_HPP
