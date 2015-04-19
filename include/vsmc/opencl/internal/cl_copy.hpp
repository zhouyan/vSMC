//============================================================================
// vSMC/include/vsmc/opencl/internal/cl_copy.hpp
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

#ifndef VSMC_OPENCL_INTERNAL_COPY_HPP
#define VSMC_OPENCL_INTERNAL_COPY_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/opencl/cl_configure.hpp>
#include <vsmc/opencl/cl_manager.hpp>

namespace vsmc
{
namespace internal
{

template <typename ID> class CLCopy
{
    public:
    typedef CLManager<ID> manager_type;

    CLCopy() : size_(0) {}

    std::size_t size() { return size_; }

    static manager_type &manager() { return manager_type::instance(); }

    void operator()(const ::cl::Buffer &copy_from, const ::cl::Buffer &state)
    {
        cl_set_kernel_args(kernel_, 0, copy_from, state);
        manager().run_kernel(kernel_, size_, configure_.local_size());
    }

    void operator()(const ::cl::Buffer &idx,
                    const ::cl::Buffer &tmp,
                    const ::cl::Buffer &state)
    {
        cl_set_kernel_args(kernel_post_, 0, idx, tmp, state);
        manager().run_kernel(kernel_, size_, configure_post_.local_size());
    }

    void build(std::size_t size, std::size_t state_size)
    {
        size_ = size;

        std::stringstream ss;

        ss << "#define Size " << size << "UL\n";
        ss << "typedef struct {char c[" << state_size << "];} state_type;";

        ss << "__kernel void copy (__global const ulong *copy_from,\n";
        ss << "                    __global state_type *state)\n";
        ss << "{\n";
        ss << "    ulong to = get_global_id(0);\n";
        ss << "    if (to >= Size) return;\n";
        ss << "    ulong from = copy_from[to];\n";
        ss << "    if (to == from) return;\n";
        ss << "    state[to] = state[from];\n";
        ss << "}\n";

        ss << "__kernel void copy_post (__global const char *idx,\n";
        ss << "                         __global const state_type *tmp,\n";
        ss << "                         __global state_type *state)\n";
        ss << "{\n";
        ss << "    ulong id = get_global_id(0);\n";
        ss << "    if (id >= Size) return;\n";
        ss << "    if (idx[id] != 0) state[id] = tmp[id];\n";
        ss << "}\n";

        program_ = manager().create_program(ss.str());
        program_.build(manager().device_vec());
        kernel_ = ::cl::Kernel(program_, "copy");
        kernel_post_ = ::cl::Kernel(program_, "copy_post");
        configure_.local_size(size, kernel_, manager().device());
        configure_post_.local_size(size, kernel_post_, manager().device());
    }

    ::cl::Program &program() { return program_; }
    const ::cl::Program &program() const { return program_; }

    ::cl::Kernel &kernel() { return kernel_; }
    const ::cl::Kernel &kernel() const { return kernel_; }

    CLConfigure &configure() { return configure_; }
    const CLConfigure &configure() const { return configure_; }

    private:
    std::size_t size_;

    ::cl::Program program_;
    ::cl::Kernel kernel_;
    ::cl::Kernel kernel_post_;
    CLConfigure configure_;
    CLConfigure configure_post_;
};  // class CLCopy
}
}  // namespace vsmc::internal

#endif  // VSMC_OPENCL_INTERNAL_COPY_HPP
