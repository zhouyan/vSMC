#ifndef VSMC_SMP_ADAPTER_HPP
#define VSMC_SMP_ADAPTER_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

/// \brief Initialize class adapter
/// \ingroup Adapter
template <typename T, template <typename, typename> class Impl,
         typename BaseType>
class InitializeAdapter :
    public internal::AdapImplTrait<T, Impl, InitializeAdapter, BaseType>::type
{
    public :

    typedef typename internal::AdapImplTrait<
        T, Impl, vsmc::InitializeAdapter, BaseType>::type
        initialize_impl_type;
    typedef cxx11::function<std::size_t (SingleParticle<T>)>
        initialize_state_type;
    typedef cxx11::function<void (Particle<T> &, void *)>
        initialize_param_type;
    typedef cxx11::function<void (Particle<T> &)>
        pre_processor_type;
    typedef cxx11::function<void (Particle<T> &)>
        post_processor_type;

    InitializeAdapter (const initialize_state_type &init_state,
            const initialize_param_type &init_param = initialize_param_type(),
            const pre_processor_type &pre = pre_processor_type(),
            const post_processor_type &post = post_processor_type()) :
        initialize_state_(init_state), initialize_param_(init_param),
        pre_processor_(pre), post_processor_(post)
    {
        VSMC_STATIC_ASSERT_ADAPTER_IMPL(Initialize, INITIALIZE);
    }

    std::size_t initialize_state (SingleParticle<T> part)
    {
        return initialize_state_(part);
    }

    void initialize_param (Particle<T> &particle, void *param)
    {
        if (bool(initialize_param_))
            initialize_param_(particle, param);
    }

    void pre_processor (Particle<T> &particle)
    {
        if (bool(pre_processor_))
            pre_processor_(particle);
    }

    void post_processor (Particle<T> &particle)
    {
        if (bool(post_processor_))
            post_processor_(particle);
    }

    private :

    const initialize_state_type initialize_state_;
    const initialize_param_type initialize_param_;
    const pre_processor_type pre_processor_;
    const post_processor_type post_processor_;
}; // class InitializeAdapter

/// \brief Move class adapter
/// \ingroup Adapter
template <typename T, template <typename, typename> class Impl,
         typename BaseType>
class MoveAdapter :
    public internal::AdapImplTrait<T, Impl, MoveAdapter, BaseType>::type
{
    public :

    typedef typename internal::AdapImplTrait<
        T, Impl, vsmc::MoveAdapter, BaseType>::type
        move_impl_type;
    typedef cxx11::function<std::size_t (std::size_t, SingleParticle<T>)>
        move_state_type;
    typedef cxx11::function<void (std::size_t, Particle<T> &)>
        pre_processor_type;
    typedef cxx11::function<void (std::size_t, Particle<T> &)>
        post_processor_type;

    MoveAdapter (const move_state_type &move_state,
            const pre_processor_type &pre = pre_processor_type(),
            const post_processor_type &post = post_processor_type()) :
        move_state_(move_state), pre_processor_(pre), post_processor_(post)
    {
        VSMC_STATIC_ASSERT_ADAPTER_IMPL(Move, MOVE);
    }

    std::size_t move_state (std::size_t iter, SingleParticle<T> part)
    {
        return move_state_(iter, part);
    }

    void pre_processor (std::size_t iter, Particle<T> &particle)
    {
        if (bool(pre_processor_))
            pre_processor_(iter, particle);
    }

    void post_processor (std::size_t iter, Particle<T> &particle)
    {
        if (bool(post_processor_))
            post_processor_(iter, particle);
    }

    private :

    const move_state_type move_state_;
    const pre_processor_type pre_processor_;
    const post_processor_type post_processor_;
}; // class MoveAdapter

/// \brief Monitor evaluation class adapter
/// \ingroup Adapter
template <typename T, template <typename, typename> class Impl,
         typename BaseType>
class MonitorEvalAdapter :
    public internal::AdapImplTrait<T, Impl, MonitorEvalAdapter, BaseType>::type
{
    public :

    typedef typename internal::AdapImplTrait<
        T, Impl, vsmc::MonitorEvalAdapter, BaseType>::type
        monitor_eval_impl_type;
    typedef cxx11::function<
        void (std::size_t, std::size_t, ConstSingleParticle<T>, double *)>
        monitor_state_type;
    typedef cxx11::function<void (std::size_t, const Particle<T> &)>
        pre_processor_type;
    typedef cxx11::function<void (std::size_t, const Particle<T> &)>
        post_processor_type;

    MonitorEvalAdapter (const monitor_state_type &monitor_state,
            const pre_processor_type &pre = pre_processor_type(),
            const post_processor_type &post = post_processor_type()) :
        monitor_state_(monitor_state),
        pre_processor_(pre), post_processor_(post)
    {
        VSMC_STATIC_ASSERT_ADAPTER_IMPL(MonitorEval, MONITOR_EVAL);
    }

    void monitor_state (std::size_t iter, std::size_t dim,
            ConstSingleParticle<T> part, double *res)
    {
        monitor_state_(iter, dim, part, res);
    }

    void pre_processor (std::size_t iter, const Particle<T> &particle)
    {
        if (bool(pre_processor_))
            pre_processor_(iter, particle);
    }

    void post_processor (std::size_t iter, const Particle<T> &particle)
    {
        if (bool(post_processor_))
            post_processor_(iter, particle);
    }

    private :

    const monitor_state_type monitor_state_;
    const pre_processor_type pre_processor_;
    const post_processor_type post_processor_;
}; // class MonitorEvalAdapter

/// \brief Path evaluation class adapter
/// \ingroup Adapter
template <typename T, template <typename, typename> class Impl,
         typename BaseType>
class PathEvalAdapter :
    public internal::AdapImplTrait<T, Impl, PathEvalAdapter, BaseType>::type
{
    public :

    typedef typename internal::AdapImplTrait<
        T, Impl, vsmc::PathEvalAdapter, BaseType>::type
        path_eval_impl_type;
    typedef cxx11::function<double (std::size_t, ConstSingleParticle<T>)>
        path_state_type;
    typedef cxx11::function<double (std::size_t, const Particle<T> &)>
        path_grid_type;
    typedef cxx11::function<void (std::size_t, const Particle<T> &)>
        pre_processor_type;
    typedef cxx11::function<void (std::size_t, const Particle<T> &)>
        post_processor_type;

    PathEvalAdapter (const path_state_type &path_state,
            const path_grid_type &path_grid,
            const pre_processor_type &pre = pre_processor_type(),
            const post_processor_type &post = post_processor_type()) :
        path_state_(path_state), path_grid_(path_grid),
        pre_processor_(pre), post_processor_(post)
    {
        VSMC_STATIC_ASSERT_ADAPTER_IMPL(PathEval, PATH_EVAL);
    }

    double path_state (std::size_t iter, ConstSingleParticle<T> part)
    {
        return path_state_(iter, part);
    }

    double path_grid (std::size_t iter, const Particle<T> &particle)
    {
        return path_grid_(iter, particle);
    }

    void pre_processor (std::size_t iter, const Particle<T> &particle)
    {
        if (bool(pre_processor_))
            pre_processor_(iter, particle);
    }

    void post_processor (std::size_t iter, const Particle<T> &particle)
    {
        if (bool(post_processor_))
            post_processor_(iter, particle);
    }

    private :

    const path_state_type path_state_;
    const path_grid_type path_grid_;
    const pre_processor_type pre_processor_;
    const post_processor_type post_processor_;
}; // class PathEvalAdapter

} // namespace vsmc

#endif // VSMC_SMP_ADAPTER_HPP
