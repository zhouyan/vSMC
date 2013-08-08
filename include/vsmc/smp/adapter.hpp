#ifndef VSMC_SMP_ADAPTER_HPP
#define VSMC_SMP_ADAPTER_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/adapter.hpp>

namespace vsmc {

/// \brief Initialize class adapter
/// \ingroup Adapter
template <typename T, template <typename, typename> class Impl, typename F>
class InitializeAdapter :
    public InitializeAdapterBase<T, F,
    Impl<T, InitializeAdapter<T, Impl, F> > >
{
    typedef InitializeAdapterBase<T, F,
    Impl<T, InitializeAdapter<T, Impl, F> > > base;

    public :

    InitializeAdapter () {}

    InitializeAdapter (const F &f) : base(f) {}

    std::size_t initialize_state (SingleParticle<T> part)
    {return this->implementation().initialize_state(part);}
}; // InitializeAdapter

/// \brief Move class adapter
/// \ingroup Adapter
template <typename T, template <typename, typename> class Impl, typename F>
class MoveAdapter :
    public MoveAdapterBase<T, F,
    Impl<T, MoveAdapter<T, Impl, F> > >
{
    typedef MoveAdapterBase<T, F,
    Impl<T, MoveAdapter<T, Impl, F> > > base;

    public :

    MoveAdapter () {}

    MoveAdapter (const F &f) : base(f) {}

    std::size_t move_state (std::size_t iter, SingleParticle<T> part)
    {return this->implementation().move_state(iter, part);}
}; // MoveAdapter

/// \brief Monitor evaluation class adapter
/// \ingroup Adapter
template <typename T, template <typename, typename> class Impl, typename F>
class MonitorEvalAdapter :
    public MonitorEvalAdapterBase<T, F,
    Impl<T, MonitorEvalAdapter<T, Impl, F> > >
{
    typedef MonitorEvalAdapterBase<T, F,
    Impl<T, MonitorEvalAdapter<T, Impl, F> > > base;

    public :

    MonitorEvalAdapter () {}

    MonitorEvalAdapter (const F &f) : base(f) {}

    void monitor_state (std::size_t iter, std::size_t dim,
            ConstSingleParticle<T> part, double *res)
    {this->implementation().monitor_state(iter, dim, part, res);}
}; // MonitorEvalAdapter

/// \brief Path evaluation class adapter
/// \ingroup Adapter
template <typename T, template <typename, typename> class Impl, typename F>
class PathEvalAdapter :
    public PathEvalAdapterBase<T, F,
    Impl<T, PathEvalAdapter<T, Impl, F> > >
{
    typedef PathEvalAdapterBase<T, F,
    Impl<T, PathEvalAdapter<T, Impl, F> > > base;

    public :

    PathEvalAdapter () {}

    PathEvalAdapter (const F &f) : base(f) {}

    double path_state (std::size_t iter, ConstSingleParticle<T> part)
    {return this->implementation().path_state(iter, part);}
}; // PathEvalAdapter

/// \brief Initialize class adapter
/// \ingroup Adapter
template <typename T, template <typename, typename> class Impl>
class InitializeAdapter<T, Impl, NullType> :
    public InitializeAdapterBase<T, NullType,
    Impl<T, InitializeAdapter<T, Impl, NullType> > >
{
    typedef InitializeAdapterBase<T, NullType,
    Impl<T, InitializeAdapter<T, Impl, NullType> > > base;

    public :

    typedef cxx11::function<std::size_t (SingleParticle<T>)>
        initialize_state_type;
    typedef typename base::initialize_param_type initialize_param_type;
    typedef typename base::pre_processor_type pre_processor_type;
    typedef typename base::post_processor_type post_processor_type;

    InitializeAdapter (const initialize_state_type &init_state,
            const initialize_param_type &init_param = initialize_param_type(),
            const pre_processor_type &pre = pre_processor_type(),
            const post_processor_type &post = post_processor_type()) :
        base(init_param, pre, post), initialize_state_(init_state) {}

    std::size_t initialize_state (SingleParticle<T> part)
    {return initialize_state_(part);}

    private :

    const initialize_state_type initialize_state_;
}; // class InitializeAdapter

/// \brief Move class adapter
/// \ingroup Adapter
template <typename T, template <typename, typename> class Impl>
class MoveAdapter<T, Impl, NullType> :
    public MoveAdapterBase<T, NullType,
    Impl<T, MoveAdapter<T, Impl, NullType> > >
{
    typedef MoveAdapterBase<T, NullType,
    Impl<T, MoveAdapter<T, Impl, NullType> > > base;

    public :

    typedef cxx11::function<std::size_t (std::size_t, SingleParticle<T>)>
        move_state_type;
    typedef typename base::pre_processor_type pre_processor_type;
    typedef typename base::post_processor_type post_processor_type;

    MoveAdapter (const move_state_type &move_state,
            const pre_processor_type &pre = pre_processor_type(),
            const post_processor_type &post = post_processor_type()) :
        base(pre, post), move_state_(move_state) {}

    std::size_t move_state (std::size_t iter, SingleParticle<T> part)
    {return move_state_(iter, part);}

    private :

    const move_state_type move_state_;
}; // class MoveAdapter

/// \brief Monitor evaluation class adapter
/// \ingroup Adapter
template <typename T, template <typename, typename> class Impl>
class MonitorEvalAdapter<T, Impl, NullType> :
    public MonitorEvalAdapterBase<T, NullType,
    Impl<T, MonitorEvalAdapter<T, Impl, NullType> > >
{
    typedef MonitorEvalAdapterBase<T, NullType,
    Impl<T, MonitorEvalAdapter<T, Impl, NullType> > > base;

    public :

    typedef cxx11::function<
        void (std::size_t, std::size_t, ConstSingleParticle<T>, double *)>
        monitor_state_type;
    typedef typename base::pre_processor_type pre_processor_type;
    typedef typename base::post_processor_type post_processor_type;

    MonitorEvalAdapter (const monitor_state_type &monitor_state,
            const pre_processor_type &pre = pre_processor_type(),
            const post_processor_type &post = post_processor_type()) :
        base(pre, post), monitor_state_(monitor_state) {}

    void monitor_state (std::size_t iter, std::size_t dim,
            ConstSingleParticle<T> part, double *res)
    {monitor_state_(iter, dim, part, res);}

    private :

    const monitor_state_type monitor_state_;
}; // class MonitorEvalAdapter

/// \brief Path evaluation class adapter
/// \ingroup Adapter
template <typename T, template <typename, typename> class Impl>
class PathEvalAdapter<T, Impl, NullType> :
    public PathEvalAdapterBase<T, NullType,
    Impl<T, PathEvalAdapter<T, Impl, NullType> > >
{
    typedef PathEvalAdapterBase<T, NullType,
    Impl<T, PathEvalAdapter<T, Impl, NullType> > > base;

    public :

    typedef cxx11::function<double (std::size_t, ConstSingleParticle<T>)>
        path_state_type;
    typedef typename base::path_grid_type path_grid_type;
    typedef typename base::pre_processor_type pre_processor_type;
    typedef typename base::post_processor_type post_processor_type;

    PathEvalAdapter (const path_state_type &path_state,
            const path_grid_type &path_grid,
            const pre_processor_type &pre = pre_processor_type(),
            const post_processor_type &post = post_processor_type()) :
        base(path_grid, pre, post), path_state_(path_state) {}

    double path_state (std::size_t iter, ConstSingleParticle<T> part)
    {return path_state_(iter, part);}

    private :

    const path_state_type path_state_;
}; // class PathEvalAdapter

} // namespace vsmc

#endif // VSMC_SMP_ADAPTER_HPP
