#ifndef VSMC_CORE_ADAPTER_HPP
#define VSMC_CORE_ADAPTER_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

template <typename T, template <typename, typename> class,
         typename = NullType> class InitializeAdapter;
template <typename T, template <typename, typename> class,
         typename = NullType> class MoveAdapter;
template <typename T, template <typename, typename> class,
         typename = NullType> class MonitorEvalAdapter;
template <typename T, template <typename, typename> class,
         typename = NullType> class PathEvalAdapter;

/// \brief Initialize class adapter base
/// \ingroup Adapter
template <typename T, typename F, typename BaseType>
class InitializeAdapterBase : public BaseType
{
    public :

    InitializeAdapterBase () {}

    InitializeAdapterBase (const F &f) : f_(f) {}

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

    F &implementation () {return f_;}

    const F &implementation () const {return f_;}

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
        (Particle<T> &, void *, cxx11::false_type) {}
    void pre_processor_dispatch
        (Particle<T> &, cxx11::false_type) {}
    void post_processor_dispatch
        (Particle<T> &, cxx11::false_type) {}
}; // InitializeAdapterBase

/// \brief Move class adapter base
/// \ingroup Adapter
template <typename T, typename F, typename BaseType>
class MoveAdapterBase : public BaseType
{
    public :

    MoveAdapterBase () {}

    MoveAdapterBase (const F &f) : f_(f) {}

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

    F &implementation () {return f_;}

    const F &implementation () const {return f_;}

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
}; // MoveAdapterBase

/// \brief Monitor evaluation base
/// \ingroup Adapter
template <typename T, typename F, typename BaseType>
class MonitorEvalAdapterBase : public BaseType
{
    public :

    MonitorEvalAdapterBase () {}

    MonitorEvalAdapterBase (const F &f) : f_(f) {}

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

    F &implementation () {return f_;}

    const F &implementation () const {return f_;}

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
}; // MonitorEvalAdapterBase

/// \brief Path evaluation class base
/// \ingroup Adapter
template <typename T, typename F, typename BaseType>
class PathEvalAdapterBase : public BaseType
{
    public :

    PathEvalAdapterBase () {}

    PathEvalAdapterBase (const F &f) : f_(f) {}

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

    F &implementation () {return f_;}

    const F &implementation () const {return f_;}

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
}; // PathEvalAdapterBase

/// \brief Initialize class adapter base
/// \ingroup Adapter
template <typename T, typename BaseType>
class InitializeAdapterBase<T, NullType, BaseType> : public BaseType
{
    public :

    typedef cxx11::function<void (Particle<T> &, void *)>
        initialize_param_type;
    typedef cxx11::function<void (Particle<T> &)>
        pre_processor_type;
    typedef cxx11::function<void (Particle<T> &)>
        post_processor_type;

    InitializeAdapterBase (
            const initialize_param_type &init_param = initialize_param_type(),
            const pre_processor_type &pre = pre_processor_type(),
            const post_processor_type &post = post_processor_type()) :
        initialize_param_(init_param),
        pre_processor_(pre), post_processor_(post) {}

    void initialize_param (Particle<T> &particle, void *param)
    {if (bool(initialize_param_)) initialize_param_(particle, param);}

    void pre_processor (Particle<T> &particle)
    {if (bool(pre_processor_)) pre_processor_(particle);}

    void post_processor (Particle<T> &particle)
    {if (bool(post_processor_)) post_processor_(particle);}

    private :

    const initialize_param_type initialize_param_;
    const pre_processor_type pre_processor_;
    const post_processor_type post_processor_;
}; // class InitializeAdapterBase

/// \brief Move class adapter base
/// \ingroup Adapter
template <typename T, typename BaseType>
class MoveAdapterBase<T, NullType, BaseType> : public BaseType
{
    public :

    typedef cxx11::function<void (std::size_t, Particle<T> &)>
        pre_processor_type;
    typedef cxx11::function<void (std::size_t, Particle<T> &)>
        post_processor_type;

    MoveAdapterBase (
            const pre_processor_type &pre = pre_processor_type(),
            const post_processor_type &post = post_processor_type()) :
        pre_processor_(pre), post_processor_(post) {}

    void pre_processor (std::size_t iter, Particle<T> &particle)
    {if (bool(pre_processor_)) pre_processor_(iter, particle);}

    void post_processor (std::size_t iter, Particle<T> &particle)
    {if (bool(post_processor_)) post_processor_(iter, particle);}

    private :

    const pre_processor_type pre_processor_;
    const post_processor_type post_processor_;
}; // class MoveAdapterBase

/// \brief Monitor evaluation base
/// \ingroup Adapter
template <typename T, typename BaseType>
class MonitorEvalAdapterBase<T, NullType, BaseType> : public BaseType
{
    public :

    typedef cxx11::function<void (std::size_t, const Particle<T> &)>
        pre_processor_type;
    typedef cxx11::function<void (std::size_t, const Particle<T> &)>
        post_processor_type;

    MonitorEvalAdapterBase (
            const pre_processor_type &pre = pre_processor_type(),
            const post_processor_type &post = post_processor_type()) :
        pre_processor_(pre), post_processor_(post) {}

    void pre_processor (std::size_t iter, const Particle<T> &particle)
    {if (bool(pre_processor_)) pre_processor_(iter, particle);}

    void post_processor (std::size_t iter, const Particle<T> &particle)
    {if (bool(post_processor_)) post_processor_(iter, particle);}

    private :

    const pre_processor_type pre_processor_;
    const post_processor_type post_processor_;
}; // class MonitorEvalAdapterBase

/// \brief Path evaluation class base
/// \ingroup Adapter
template <typename T, typename BaseType>
class PathEvalAdapterBase<T, NullType, BaseType> : public BaseType
{
    public :

    typedef cxx11::function<double (std::size_t, const Particle<T> &)>
        path_grid_type;
    typedef cxx11::function<void (std::size_t, const Particle<T> &)>
        pre_processor_type;
    typedef cxx11::function<void (std::size_t, const Particle<T> &)>
        post_processor_type;

    PathEvalAdapterBase (
            const path_grid_type &path_grid,
            const pre_processor_type &pre = pre_processor_type(),
            const post_processor_type &post = post_processor_type()) :
        path_grid_(path_grid), pre_processor_(pre), post_processor_(post) {}

    double path_grid (std::size_t iter, const Particle<T> &particle)
    {return path_grid_(iter, particle);}

    void pre_processor (std::size_t iter, const Particle<T> &particle)
    {if (bool(pre_processor_)) pre_processor_(iter, particle);}

    void post_processor (std::size_t iter, const Particle<T> &particle)
    {if (bool(post_processor_)) post_processor_(iter, particle);}

    private :

    const path_grid_type path_grid_;
    const pre_processor_type pre_processor_;
    const post_processor_type post_processor_;
}; // class PathEvalAdapterBase

} // namespace vsmc

#endif // VSMC_CORE_ADAPTER_HPP
