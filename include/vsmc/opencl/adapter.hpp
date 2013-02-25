#ifndef VSMC_OPENCL_ADAPTER_HPP
#define VSMC_OPENCL_ADAPTER_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

/// \brief Initialize class adapter specialization for OpenCL
/// \ingroup Adapter
template <typename T, typename BaseType>
class InitializeAdapter<T, InitializeCL, BaseType> :
    public InitializeCL<T, BaseType>
{
    public :

    typedef InitializeCL<T, BaseType> initialize_impl_type;
    typedef cxx11::function<void (Particle<T> &, void *)>
        initialize_param_type;
    typedef cxx11::function<void (Particle<T> &)>
        pre_processor_type;
    typedef cxx11::function<void (Particle<T> &)>
        post_processor_type;

    InitializeAdapter (const std::string &init_state,
            const initialize_param_type &init_param = initialize_param_type(),
            const pre_processor_type &pre = pre_processor_type(),
            const post_processor_type &post = post_processor_type()) :
        initialize_state_(init_state), initialize_param_(init_param),
        pre_processor_(pre), post_processor_(post) {}

    void initialize_state (std::string &kernel_name)
    {kernel_name = initialize_state_;}

    void initialize_param (Particle<T> &particle, void *param)
    {if (bool(initialize_param_)) initialize_param_(particle, param);}

    void pre_processor (Particle<T> &particle)
    {if (bool(pre_processor_)) pre_processor_(particle);}

    void post_processor (Particle<T> &particle)
    {if (bool(post_processor_)) post_processor_(particle);}

    private :

    const std::string initialize_state_;
    const initialize_param_type initialize_param_;
    const pre_processor_type pre_processor_;
    const post_processor_type post_processor_;
}; // class InitializeAdapter

/// \brief Move class adapter specialization for OpenCL
/// \ingroup Adapter
template <typename T, typename BaseType>
class MoveAdapter<T, MoveCL, BaseType> : public MoveCL<T, BaseType>
{
    public :

    typedef MoveCL<T, BaseType> move_impl_type;
    typedef cxx11::function<void (std::size_t, Particle<T> &)>
        pre_processor_type;
    typedef cxx11::function<void (std::size_t, Particle<T> &)>
        post_processor_type;

    MoveAdapter (const std::string &move_state,
            const pre_processor_type &pre = pre_processor_type(),
            const post_processor_type &post = post_processor_type()) :
        move_state_(move_state), pre_processor_(pre), post_processor_(post) {}

    void move_state (std::size_t iter, std::string &kernel_name)
    {kernel_name = move_state_;}

    void pre_processor (std::size_t iter, Particle<T> &particle)
    {if (bool(pre_processor_)) pre_processor_(iter, particle);}

    void post_processor (std::size_t iter, Particle<T> &particle)
    {if (bool(post_processor_)) post_processor_(iter, particle);}

    private :

    const std::string move_state_;
    const pre_processor_type pre_processor_;
    const post_processor_type post_processor_;
}; // class MoveAdapter

/// \brief Monitor evaluation class adapter specialization for OpenCL
/// \ingroup Adapter
template <typename T, typename BaseType>
class MonitorEvalAdapter<T, MonitorEvalCL, BaseType> :
    public MonitorEvalCL<T, BaseType>
{
    public :

    typedef MonitorEvalCL<T, BaseType> monitor_eval_impl_type;
    typedef cxx11::function<void (std::size_t, const Particle<T> &)>
        pre_processor_type;
    typedef cxx11::function<void (std::size_t, const Particle<T> &)>
        post_processor_type;

    MonitorEvalAdapter (const std::string &monitor_state,
            const pre_processor_type &pre = pre_processor_type(),
            const post_processor_type &post = post_processor_type()) :
        monitor_state_(monitor_state),
        pre_processor_(pre), post_processor_(post) {}

    void monitor_state (std::size_t, std::string &kernel_name)
    {kernel_name = monitor_state_;}

    void pre_processor (std::size_t iter, const Particle<T> &particle)
    {if (bool(pre_processor_)) pre_processor_(iter, particle);}

    void post_processor (std::size_t iter, const Particle<T> &particle)
    {if (bool(post_processor_)) post_processor_(iter, particle);}

    private :

    const std::string monitor_state_;
    const pre_processor_type pre_processor_;
    const post_processor_type post_processor_;
}; // class MonitorEvalAdapter

/// \brief Path evaluation class adapter specialization for OpenCL
/// \ingroup Adapter
template <typename T, typename BaseType>
class PathEvalAdapter<T, PathEvalCL, BaseType> :
    public PathEvalCL<T, BaseType>
{
    public :

    typedef PathEvalCL<T, BaseType> path_eval_impl_type;
    typedef cxx11::function<double (std::size_t, const Particle<T> &)>
        path_grid_type;
    typedef cxx11::function<void (std::size_t, const Particle<T> &)>
        pre_processor_type;
    typedef cxx11::function<void (std::size_t, const Particle<T> &)>
        post_processor_type;

    PathEvalAdapter (const std::string &path_state,
            const path_grid_type &path_grid,
            const pre_processor_type &pre = pre_processor_type(),
            const post_processor_type &post = post_processor_type()) :
        path_state_(path_state), path_grid_(path_grid),
        pre_processor_(pre), post_processor_(post) {}

    void path_state (std::size_t, std::string &kernel_name)
    {kernel_name = path_state_;}

    double path_grid (std::size_t iter, const Particle<T> &particle)
    {return path_grid_(iter, particle);}

    void pre_processor (std::size_t iter, const Particle<T> &particle)
    {if (bool(pre_processor_)) pre_processor_(iter, particle);}

    void post_processor (std::size_t iter, const Particle<T> &particle)
    {if (bool(post_processor_)) post_processor_(iter, particle);}

    private :

    const std::string path_state_;
    const path_grid_type path_grid_;
    const pre_processor_type pre_processor_;
    const post_processor_type post_processor_;
}; // class PathEvalAdapter

} // namespace vsmc

#endif // VSMC_OPENCL_ADAPTER_HPP
