#ifndef VSMC_HELPER_ADAPTER_HPP
#define VSMC_HELPER_ADAPTER_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

/// \brief Initialize class adapter
/// \ingroup Helper
template <typename T, template <typename, typename> class InitializeImpl>
class InitializeAdapter :
    public InitializeImpl<T, InitializeAdapter<T, InitializeImpl> >
{
    public :

    typedef InitializeImpl<T, InitializeAdapter<T, InitializeImpl> >
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
            const initialize_param_type &init_param = VSMC_NULLPTR,
            const pre_processor_type &pre = VSMC_NULLPTR,
            const post_processor_type &post = VSMC_NULLPTR) :
        initialize_state_(init_state), initialize_param_(init_param),
        pre_processor_(pre), post_processor_(post) {}

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

    initialize_state_type initialize_state_;
    initialize_param_type initialize_param_;
    pre_processor_type pre_processor_;
    post_processor_type post_processor_;
}; // class InitializeAdapter

/// \brief Move class adapter
/// \ingroup Helper
template <typename T, template <typename, typename> class MoveImpl>
class MoveAdapter :
    public MoveImpl<T, MoveAdapter<T, MoveImpl> >
{
    public :

    typedef MoveImpl<T, MoveAdapter<T, MoveImpl> >
        move_impl_type;
    typedef cxx11::function<unsigned (unsigned, SingleParticle<T>)>
        move_state_type;
    typedef cxx11::function<void (unsigned, Particle<T> &)>
        pre_processor_type;
    typedef cxx11::function<void (unsigned, Particle<T> &)>
        post_processor_type;

    MoveAdapter (const move_state_type &move_state,
            const pre_processor_type &pre = VSMC_NULLPTR,
            const post_processor_type &post = VSMC_NULLPTR) :
        move_state_(move_state), pre_processor_(pre), post_processor_(post) {}

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

    move_state_type move_state_;
    pre_processor_type pre_processor_;
    post_processor_type post_processor_;
}; // class MoveAdapter

/// \brief Monitor evaluation class adapter
/// \ingroup Helper
template <typename T, template <typename, typename> class MonitorEvalImpl>
class MonitorEvalAdapter :
    public MonitorEvalImpl<T, MonitorEvalAdapter<T, MonitorEvalImpl> >
{
    public :

    typedef MonitorEvalImpl<T, MonitorEvalAdapter<T, MonitorEvalImpl> >
        monitor_eval_impl_type;
    typedef cxx11::function<
        void (unsigned, unsigned, ConstSingleParticle<T>, double *)>
        monitor_state_type;
    typedef cxx11::function<void (unsigned, const Particle<T> &)>
        pre_processor_type;
    typedef cxx11::function<void (unsigned, const Particle<T> &)>
        post_processor_type;

    MonitorEvalAdapter (const monitor_state_type &monitor_state,
            const pre_processor_type &pre = VSMC_NULLPTR,
            const post_processor_type &post = VSMC_NULLPTR) :
        monitor_state_(monitor_state),
        pre_processor_(pre), post_processor_(post) {}

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

    monitor_state_type monitor_state_;
    pre_processor_type pre_processor_;
    post_processor_type post_processor_;
}; // class MonitorEvalAdapter

/// \brief Path evaluation class adapter
/// \ingroup Helper
template <typename T, template <typename, typename> class PathEvalImpl>
class PathEvalAdapter :
    public PathEvalImpl<T, PathEvalAdapter<T, PathEvalImpl> >
{
    public :

    typedef PathEvalImpl<T, PathEvalAdapter<T, PathEvalImpl> >
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
            const pre_processor_type &pre = VSMC_NULLPTR,
            const post_processor_type &post = VSMC_NULLPTR) :
        path_state_(path_state), path_width_(path_width),
        pre_processor_(pre), post_processor_(post) {}

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

    path_state_type path_state_;
    path_width_type path_width_;
    pre_processor_type pre_processor_;
    post_processor_type post_processor_;
}; // class PathEvalAdapter

} // namespace vsmc

#endif // VSMC_HELPER_ADAPTER_HPP
