#ifndef VSMC_HELPER_ADAPTER_HPP
#define VSMC_HELPER_ADAPTER_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

namespace traits {

template <typename, template <typename, typename> class,
         template <typename, template <typename, typename> class, typename>
             class, typename>
struct AdapImplTrait;

template <typename T,
         template <typename, typename> class Impl,
         template <typename, template <typename, typename> class, typename>
             class Adapter>
struct AdapImplTrait<T, Impl, Adapter, VBase>
{
    typedef Impl<T, VBase> type;
};

template <typename T,
         template <typename, typename> class Impl,
         template <typename, template <typename, typename> class, typename>
         class Adapter>
struct AdapImplTrait<T, Impl, Adapter, CBase>
{
    typedef Impl<T, Adapter<T, Impl, CBase> > type;
};

} // namesapce vsmc::traits

/// \brief Initialize class adapter
/// \ingroup Adapter
template <typename T, template <typename, typename> class Impl,
         typename BaseType>
class InitializeAdapter :
    public traits::AdapImplTrait<T, Impl, InitializeAdapter, BaseType>::type
{
    public :

    typedef typename traits::AdapImplTrait<
        T, Impl, vsmc::InitializeAdapter, BaseType>::type
        initialize_impl_type;
    typedef cxx11::function<unsigned (SingleParticle<T>)>
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
        VSMC_STATIC_ASSERT(traits::IsInitializeImpl<Impl>::value,
                USE_InitializeAdapter_WITHOUT_AN_INITIAILIZE_IMPLEMENTATION);
    }

    unsigned initialize_state (SingleParticle<T> part)
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
    public traits::AdapImplTrait<T, Impl, MoveAdapter, BaseType>::type
{
    public :

    typedef typename traits::AdapImplTrait<
        T, Impl, vsmc::MoveAdapter, BaseType>::type
        move_impl_type;
    typedef cxx11::function<unsigned (unsigned, SingleParticle<T>)>
        move_state_type;
    typedef cxx11::function<void (unsigned, Particle<T> &)>
        pre_processor_type;
    typedef cxx11::function<void (unsigned, Particle<T> &)>
        post_processor_type;

    MoveAdapter (const move_state_type &move_state,
            const pre_processor_type &pre = pre_processor_type(),
            const post_processor_type &post = post_processor_type()) :
        move_state_(move_state), pre_processor_(pre), post_processor_(post)
    {
        VSMC_STATIC_ASSERT(traits::IsMoveImpl<Impl>::value,
                USE_MoveAdapter_WITHOUT_A_MOVE_IMPLEMENTATION);
    }

    unsigned move_state (unsigned iter, SingleParticle<T> part)
    {
        return move_state_(iter, part);
    }

    void pre_processor (unsigned iter, Particle<T> &particle)
    {
        if (bool(pre_processor_))
            pre_processor_(iter, particle);
    }

    void post_processor (unsigned iter, Particle<T> &particle)
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
    public traits::AdapImplTrait<T, Impl, MonitorEvalAdapter, BaseType>::type
{
    public :

    typedef typename traits::AdapImplTrait<
        T, Impl, vsmc::MonitorEvalAdapter, BaseType>::type
        monitor_eval_impl_type;
    typedef cxx11::function<
        void (unsigned, unsigned, ConstSingleParticle<T>, double *)>
        monitor_state_type;
    typedef cxx11::function<void (unsigned, const Particle<T> &)>
        pre_processor_type;
    typedef cxx11::function<void (unsigned, const Particle<T> &)>
        post_processor_type;

    MonitorEvalAdapter (const monitor_state_type &monitor_state,
            const pre_processor_type &pre = pre_processor_type(),
            const post_processor_type &post = post_processor_type()) :
        monitor_state_(monitor_state),
        pre_processor_(pre), post_processor_(post)
    {
        VSMC_STATIC_ASSERT(traits::IsMonitorEvalImpl<Impl>::value,
                USE_MonitorEvalAdapter_WITHOUT_A_MONITOR_EVAL_IMPLEMENTATION);
    }

    void monitor_state (unsigned iter, unsigned dim,
            ConstSingleParticle<T> part, double *res)
    {
        monitor_state_(iter, dim, part, res);
    }

    void pre_processor (unsigned iter, const Particle<T> &particle)
    {
        if (bool(pre_processor_))
            pre_processor_(iter, particle);
    }

    void post_processor (unsigned iter, const Particle<T> &particle)
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
    public traits::AdapImplTrait<T, Impl, PathEvalAdapter, BaseType>::type
{
    public :

    typedef typename traits::AdapImplTrait<
        T, Impl, vsmc::PathEvalAdapter, BaseType>::type
        path_eval_impl_type;
    typedef cxx11::function<double (unsigned, ConstSingleParticle<T>)>
        path_state_type;
    typedef cxx11::function<double (unsigned, const Particle<T> &)>
        path_width_type;
    typedef cxx11::function<void (unsigned, const Particle<T> &)>
        pre_processor_type;
    typedef cxx11::function<void (unsigned, const Particle<T> &)>
        post_processor_type;

    PathEvalAdapter (const path_state_type &path_state,
            const path_width_type &path_width,
            const pre_processor_type &pre = pre_processor_type(),
            const post_processor_type &post = post_processor_type()) :
        path_state_(path_state), path_width_(path_width),
        pre_processor_(pre), post_processor_(post)
    {
        VSMC_STATIC_ASSERT(traits::IsPathEvalImpl<Impl>::value,
                USE_PathEvalAdapter_WITHOUT_A_PATH_EVAL_IMPLEMENTATION);
    }

    double path_state (unsigned iter, ConstSingleParticle<T> part)
    {
        return path_state_(iter, part);
    }

    double path_width (unsigned iter, const Particle<T> &particle)
    {
        return path_width_(iter, particle);
    }

    void pre_processor (unsigned iter, const Particle<T> &particle)
    {
        if (bool(pre_processor_))
            pre_processor_(iter, particle);
    }

    void post_processor (unsigned iter, const Particle<T> &particle)
    {
        if (bool(post_processor_))
            post_processor_(iter, particle);
    }

    private :

    const path_state_type path_state_;
    const path_width_type path_width_;
    const pre_processor_type pre_processor_;
    const post_processor_type post_processor_;
}; // class PathEvalAdapter

} // namespace vsmc

#endif // VSMC_HELPER_ADAPTER_HPP
