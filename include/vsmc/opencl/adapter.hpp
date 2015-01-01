//============================================================================
// vSMC/include/vsmc/opencl/adapter.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#ifndef VSMC_OPENCL_ADAPTER_HPP
#define VSMC_OPENCL_ADAPTER_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/adapter.hpp>
#include <vsmc/opencl/backend_cl.hpp>

namespace vsmc {

/// \brief Initialize class adapter specialization for OpenCL
/// \ingroup Adapter
template <typename T, typename F>
class InitializeAdapter<T, InitializeCL, F> :
    public InitializeAdapterBase<T, F, InitializeCL<T> >
{
    typedef InitializeAdapterBase<T, F, InitializeCL<T> > base;

    public :

    InitializeAdapter () {}

    InitializeAdapter (const F &f) : base(f) {}

    void initialize_state (std::string &kernel_name)
    {return this->implementation().initialize_state(kernel_name);}
}; // InitializeAdapter

/// \brief Move class adapter specialization for OpenCL
/// \ingroup Adapter
template <typename T, typename F>
class MoveAdapter<T, MoveCL, F> : public MoveAdapterBase<T, F, MoveCL<T> >
{
    typedef MoveAdapterBase<T, F, MoveCL<T> > base;

    public :

    MoveAdapter () {}

    MoveAdapter (const F &f) : base(f) {}

    void move_state (std::size_t iter, std::string &kernel_name)
    {return this->implementation().move_state(iter, kernel_name);}
}; // MoveAdapter

/// \brief Monitor evaluation class adapter specialization for OpenCL
/// \ingroup Adapter
template <typename T, typename F>
class MonitorEvalAdapter<T, MonitorEvalCL, F> :
    public MonitorEvalAdapterBase<T, F, MonitorEvalCL<T> >
{
    typedef MonitorEvalAdapterBase<T, F, MonitorEvalCL<T> > base;

    public :

    MonitorEvalAdapter () {}

    MonitorEvalAdapter (const F &f) : base(f) {}

    void move_state (std::size_t iter, std::string &kernel_name)
    {this->implementation().monitor_state(iter, kernel_name);}
}; // MonitorEvalAdapter

/// \brief Path evaluation class adapter specialization for OpenCL
/// \ingroup Adapter
template <typename T, typename F>
class PathEvalAdapter<T, PathEvalCL, F> :
    public PathEvalAdapterBase<T, F, PathEvalCL<T> >
{
    typedef PathEvalAdapterBase<T, F, PathEvalCL<T> > base;

    public :

    PathEvalAdapter () {}

    PathEvalAdapter (const F &f) : base(f) {}

    void path_state (std::size_t iter, std::string &kernel_name)
    {return this->implementation().path_state(iter, kernel_name);}
}; // PathEvalAdapter

/// \brief Initialize class adapter specialization for OpenCL
/// \ingroup Adapter
template <typename T>
class InitializeAdapter<T, InitializeCL, NullType> :
    public InitializeAdapterBase<T, NullType, InitializeCL<T> >
{
    typedef InitializeAdapterBase<T, NullType, InitializeCL<T> > base;

    public :

    typedef typename base::initialize_param_type initialize_param_type;
    typedef typename base::pre_processor_type pre_processor_type;
    typedef typename base::post_processor_type post_processor_type;

    InitializeAdapter (const std::string &init_state,
            const initialize_param_type &init_param = initialize_param_type(),
            const pre_processor_type &pre = pre_processor_type(),
            const post_processor_type &post = post_processor_type()) :
        base(init_param, pre, post), initialize_state_(init_state) {}

    void initialize_state (std::string &kernel_name)
    {kernel_name = initialize_state_;}

    private :

    const std::string initialize_state_;
}; // class InitializeAdapter

/// \brief Move class adapter specialization for OpenCL
/// \ingroup Adapter
template <typename T>
class MoveAdapter<T, MoveCL, NullType> :
    public MoveAdapterBase<T, NullType, MoveCL<T> >
{
    typedef MoveAdapterBase<T, NullType, MoveCL<T> > base;

    public :

    typedef typename base::pre_processor_type pre_processor_type;
    typedef typename base::post_processor_type post_processor_type;

    MoveAdapter (const std::string &move_state,
            const pre_processor_type &pre = pre_processor_type(),
            const post_processor_type &post = post_processor_type()) :
        base(pre, post), move_state_(move_state) {}

    void move_state (std::size_t, std::string &kernel_name)
    {kernel_name = move_state_;}

    private :

    const std::string move_state_;
}; // class MoveAdapter

/// \brief Monitor evaluation class adapter specialization for OpenCL
/// \ingroup Adapter
template <typename T>
class MonitorEvalAdapter<T, MonitorEvalCL, NullType> :
    public MonitorEvalAdapterBase<T, NullType, MonitorEvalCL<T> >
{
    typedef MonitorEvalAdapterBase<T, NullType, MonitorEvalCL<T> > base;

    public :

    typedef typename base::pre_processor_type pre_processor_type;
    typedef typename base::post_processor_type post_processor_type;

    MonitorEvalAdapter (const std::string &monitor_state,
            const pre_processor_type &pre = pre_processor_type(),
            const post_processor_type &post = post_processor_type()) :
        base(pre, post), monitor_state_(monitor_state) {}

    void monitor_state (std::size_t, std::string &kernel_name)
    {kernel_name = monitor_state_;}

    private :

    const std::string monitor_state_;
}; // class MonitorEvalAdapter

/// \brief Path evaluation class adapter specialization for OpenCL
/// \ingroup Adapter
template <typename T>
class PathEvalAdapter<T, PathEvalCL, NullType> :
    public PathEvalAdapterBase<T, NullType, PathEvalCL<T> >
{
    typedef PathEvalAdapterBase<T, NullType, PathEvalCL<T> > base;

    public :

    typedef typename base::path_grid_type path_grid_type;
    typedef typename base::pre_processor_type pre_processor_type;
    typedef typename base::post_processor_type post_processor_type;

    PathEvalAdapter (const std::string &path_state,
            const path_grid_type &path_grid,
            const pre_processor_type &pre = pre_processor_type(),
            const post_processor_type &post = post_processor_type()) :
        base(path_grid, pre, post), path_state_(path_state) {}

    void path_state (std::size_t, std::string &kernel_name)
    {kernel_name = path_state_;}

    private :

    const std::string path_state_;
}; // class PathEvalAdapter

} // namespace vsmc

#endif // VSMC_OPENCL_ADAPTER_HPP
