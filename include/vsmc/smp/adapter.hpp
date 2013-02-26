#ifndef VSMC_SMP_ADAPTER_HPP
#define VSMC_SMP_ADAPTER_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

/// \brief Initialize class adapter
/// \ingroup Adapter
template <typename T, template <typename, typename> class Impl, typename F>
class InitializeAdapter : public Impl<T, InitializeAdapter<T, Impl, F> >
{
    public :

    InitializeAdapter () {}

    InitializeAdapter (const F &f) : f_(f) {}

    std::size_t initialize_state (SingleParticle<T> part)
    {return f_.initialize_state(part);}

    void initialize_param (Particle<T> &particle, void *param)
    {
        initialize_param_dispatch(particle, param,
                typename has_initialize_param_<F>::type());
    }

    void pre_processor (Particle<T> &particle)
    {
        pre_processor_dispatch(particle,
            typename has_pre_processor_<F>::type());
    }

    void post_processor (Particle<T> &particle)
    {
        post_processor_dispatch(particle,
                typename has_post_processor_<F>::type());
    }

    private :

    F f_;

    VSMC_DEFINE_SMP_MF_CHECKER(initialize_param,
            void, (Particle<T> &, void *))
    VSMC_DEFINE_SMP_MF_CHECKER(pre_processor,
            void, (Particle<T> &))
    VSMC_DEFINE_SMP_MF_CHECKER(post_processor,
            void, (Particle<T> &))

    void initialize_param_dispatch (Particle<T> &particle, void *param,
            cxx11::true_type)
    {f_.initialize_param(particle, param);}

    void pre_processor_dispatch (Particle<T> &particle, cxx11::true_type)
    {f_.pre_processor(particle);}

    void post_processor_dispatch (Particle<T> &particle, cxx11::true_type)
    {f_.post_processor(particle);}

    void initialize_param_dispatch
        (Particle<T> &, void *param, cxx11::false_type) {}
    void pre_processor_dispatch
        (Particle<T> &, cxx11::false_type) {}
    void post_processor_dispatch
        (Particle<T> &, cxx11::false_type) {}
}; // InitializeAdapter

/// \brief Move class adapter
/// \ingroup Adapter
template <typename T, template <typename, typename> class Impl, typename F>
class MoveAdapter : public Impl<T, MoveAdapter<T, Impl, F> >
{
    public :

    MoveAdapter () {}

    MoveAdapter (const F &f) : f_(f) {}

    std::size_t move_state (std::size_t iter, SingleParticle<T> part)
    {return f_.move_state(iter, part);}

    void pre_processor (std::size_t iter, Particle<T> &particle)
    {
        pre_processor_dispatch(iter, particle,
                typename has_pre_processor_<F>::type());
    }

    void post_processor (std::size_t iter, Particle<T> &particle)
    {
        post_processor_dispatch(iter, particle,
                typename has_post_processor_<F>::type());
    }

    private :

    F f_;

    VSMC_DEFINE_SMP_MF_CHECKER(pre_processor,
            void, (std::size_t, Particle<T> &))
    VSMC_DEFINE_SMP_MF_CHECKER(post_processor,
            void, (std::size_t, Particle<T> &))

    void pre_processor_dispatch (std::size_t iter, Particle<T> &particle,
            cxx11::true_type)
    {f_.pre_processor(iter, particle);}

    void post_processor_dispatch (std::size_t iter, Particle<T> &particle,
            cxx11::true_type)
    {f_.post_processor(iter, particle);}

    void pre_processor_dispatch
        (std::size_t, Particle<T> &, cxx11::false_type) {}
    void post_processor_dispatch
        (std::size_t, Particle<T> &, cxx11::false_type) {}
}; // MoveAdapter

/// \brief Monitor evaluation class adapter
/// \ingroup Adapter
template <typename T, template <typename, typename> class Impl, typename F>
class MonitorEvalAdapter : public Impl<T, MonitorEvalAdapter<T, Impl, F> >
{
    public :

    MonitorEvalAdapter () {}

    MonitorEvalAdapter (const F &f) : f_(f) {}

    void monitor_state (std::size_t iter, std::size_t dim,
            ConstSingleParticle<T> part, double *res)
    {f_.monitor_state(iter, dim, part, res);}

    void pre_processor (std::size_t iter, const Particle<T> &particle)
    {
        pre_processor_dispatch(iter, particle,
                typename has_pre_processor_<F>::type());
    }

    void post_processor (std::size_t iter, const Particle<T> &particle)
    {
        post_processor_dispatch(iter, particle,
                typename has_post_processor_<F>::type());
    }

    private :

    F f_;

    VSMC_DEFINE_SMP_MF_CHECKER(pre_processor,
            void, (std::size_t, const Particle<T> &))
    VSMC_DEFINE_SMP_MF_CHECKER(post_processor,
            void, (std::size_t, const Particle<T> &))

    void pre_processor_dispatch (std::size_t iter,
            const Particle<T> &particle, cxx11::true_type)
    {f_.pre_processor(iter, particle);}

    void post_processor_dispatch (std::size_t iter,
            const Particle<T> &particle, cxx11::true_type)
    {f_.post_processor(iter, particle);}

    void pre_processor_dispatch
        (std::size_t, const Particle<T> &, cxx11::false_type) {}
    void post_processor_dispatch
        (std::size_t, const Particle<T> &, cxx11::false_type) {}
}; // MonitorEvalAdapter

/// \brief Path evaluation class adapter
/// \ingroup Adapter
template <typename T, template <typename, typename> class Impl, typename F>
class PathEvalAdapter : public Impl<T, PathEvalAdapter<T, Impl, F> >
{
    public :

    PathEvalAdapter () {}

    PathEvalAdapter (const F &f) : f_(f) {}

    double path_state (std::size_t iter, ConstSingleParticle<T> part)
    {return f_.path_state(iter, part);}

    double path_grid (std::size_t iter, const Particle<T> &particle)
    {return f_.path_grid(iter, particle);}

    void pre_processor (std::size_t iter, const Particle<T> &particle)
    {
        pre_processor_dispatch(iter, particle,
                typename has_pre_processor_<F>::type());
    }

    void post_processor (std::size_t iter, const Particle<T> &particle)
    {
        post_processor_dispatch(iter, particle,
                typename has_post_processor_<F>::type());
    }

    private :

    F f_;

    VSMC_DEFINE_SMP_MF_CHECKER(pre_processor,
            void, (std::size_t, const Particle<T> &))
    VSMC_DEFINE_SMP_MF_CHECKER(post_processor,
            void, (std::size_t, const Particle<T> &))

    void pre_processor_dispatch (std::size_t iter,
            const Particle<T> &particle, cxx11::true_type)
    {f_.pre_processor(iter, particle);}

    void post_processor_dispatch (std::size_t iter,
            const Particle<T> &particle, cxx11::true_type)
    {f_.post_processor(iter, particle);}

    void pre_processor_dispatch
        (std::size_t, const Particle<T> &, cxx11::false_type) {}
    void post_processor_dispatch
        (std::size_t, const Particle<T> &, cxx11::false_type) {}
}; // PathEvalAdapter

/// \brief Initialize class adapter of functors
/// \ingroup Adapter
template <typename T, template <typename, typename> class Impl>
class InitializeAdapter<T, Impl, Functor> :
    public Impl<T, InitializeAdapter<T, Impl, Functor> >
{
    public :

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
    {VSMC_STATIC_ASSERT_ADAPTER_IMPL(Initialize, INITIALIZE);}

    std::size_t initialize_state (SingleParticle<T> part)
    {return initialize_state_(part);}

    void initialize_param (Particle<T> &particle, void *param)
    {if (bool(initialize_param_)) initialize_param_(particle, param);}

    void pre_processor (Particle<T> &particle)
    {if (bool(pre_processor_)) pre_processor_(particle);}

    void post_processor (Particle<T> &particle)
    {if (bool(post_processor_)) post_processor_(particle);}

    private :

    const initialize_state_type initialize_state_;
    const initialize_param_type initialize_param_;
    const pre_processor_type pre_processor_;
    const post_processor_type post_processor_;
}; // class InitializeAdapter

/// \brief Move class adapter of functors
/// \ingroup Adapter
template <typename T, template <typename, typename> class Impl>
class MoveAdapter<T, Impl, Functor> :
    public Impl<T, MoveAdapter<T, Impl, Functor> >
{
    public :

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
    {VSMC_STATIC_ASSERT_ADAPTER_IMPL(Move, MOVE);}

    std::size_t move_state (std::size_t iter, SingleParticle<T> part)
    {return move_state_(iter, part);}

    void pre_processor (std::size_t iter, Particle<T> &particle)
    {if (bool(pre_processor_)) pre_processor_(iter, particle);}

    void post_processor (std::size_t iter, Particle<T> &particle)
    {if (bool(post_processor_)) post_processor_(iter, particle);}

    private :

    const move_state_type move_state_;
    const pre_processor_type pre_processor_;
    const post_processor_type post_processor_;
}; // class MoveAdapter

/// \brief Monitor evaluation class adapter of functors
/// \ingroup Adapter
template <typename T, template <typename, typename> class Impl>
class MonitorEvalAdapter<T, Impl, Functor> :
    public Impl<T, MonitorEvalAdapter<T, Impl, Functor> >
{
    public :

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
    {VSMC_STATIC_ASSERT_ADAPTER_IMPL(MonitorEval, MONITOR_EVAL);}

    void monitor_state (std::size_t iter, std::size_t dim,
            ConstSingleParticle<T> part, double *res)
    {monitor_state_(iter, dim, part, res);}

    void pre_processor (std::size_t iter, const Particle<T> &particle)
    {if (bool(pre_processor_)) pre_processor_(iter, particle);}

    void post_processor (std::size_t iter, const Particle<T> &particle)
    {if (bool(post_processor_)) post_processor_(iter, particle);}

    private :

    const monitor_state_type monitor_state_;
    const pre_processor_type pre_processor_;
    const post_processor_type post_processor_;
}; // class MonitorEvalAdapter

/// \brief Path evaluation class adapter of functors
/// \ingroup Adapter
template <typename T, template <typename, typename> class Impl>
class PathEvalAdapter<T, Impl, Functor> :
    public Impl<T, PathEvalAdapter<T, Impl, Functor> >
{
    public :

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
    {VSMC_STATIC_ASSERT_ADAPTER_IMPL(PathEval, PATH_EVAL);}

    double path_state (std::size_t iter, ConstSingleParticle<T> part)
    {return path_state_(iter, part);}

    double path_grid (std::size_t iter, const Particle<T> &particle)
    {return path_grid_(iter, particle);}

    void pre_processor (std::size_t iter, const Particle<T> &particle)
    {if (bool(pre_processor_)) pre_processor_(iter, particle);}

    void post_processor (std::size_t iter, const Particle<T> &particle)
    {if (bool(post_processor_)) post_processor_(iter, particle);}

    private :

    const path_state_type path_state_;
    const path_grid_type path_grid_;
    const pre_processor_type pre_processor_;
    const post_processor_type post_processor_;
}; // class PathEvalAdapter

} // namespace vsmc

#endif // VSMC_SMP_ADAPTER_HPP
