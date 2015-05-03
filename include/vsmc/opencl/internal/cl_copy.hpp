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

#include <vsmc/opencl/internal/common.hpp>
#include <vsmc/opencl/cl_configure.hpp>
#include <vsmc/opencl/cl_manager.hpp>
#include <vsmc/opencl/cl_type.hpp>

namespace vsmc
{
namespace internal
{

template <typename ID>
class CLCopy
{
    public:
    typedef CLManager<ID> manager_type;

    CLCopy() : size_(0) {}

    std::size_t size() { return size_; }

    static manager_type &manager() { return manager_type::instance(); }

    void operator()(::cl_mem copy_from, ::cl_mem state)
    {
        cl_set_kernel_args(kernel_.get(), 0, copy_from, state);
        manager().run_kernel(kernel_.get(), size_, configure_.local_size());
    }

    void operator()(::cl_mem idx, ::cl_mem tmp, ::cl_mem state)
    {
        cl_set_kernel_args(kernel_post_.get(), 0, idx, tmp, state);
        manager().run_kernel(
            kernel_.get(), size_, configure_post_.local_size());
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

        ::cl_int status = CL_SUCCESS;

        program_ = manager().create_program(ss.str());

        std::vector<::cl_device_id> dev_vec_ptr;
        for (const auto &dev : manager().device_vec())
            dev_vec_ptr.push_back(dev.get());
        status = ::clBuildProgram(program_.get(),
            static_cast<::cl_uint>(dev_vec_ptr.size()), dev_vec_ptr.data(),
            nullptr, nullptr, nullptr);
        internal::cl_error_check(status, "CLCopy::build", "::clBuildProgram");

        ::cl_kernel kern = nullptr;

        kern = ::clCreateKernel(program_.get(), "copy", &status);
        internal::cl_error_check(status, "CLCopy::build", "::clCreateKernel");
        kernel_ = cl_kernel_make_shared(kern);

        kern = ::clCreateKernel(program_.get(), "copy_post", &status);
        internal::cl_error_check(status, "CLCopy::build", "::clCreateKernel");
        kernel_post_ = cl_kernel_make_shared(kern);

        configure_.local_size(size, kernel_.get(), manager().device().get());
        configure_post_.local_size(
            size, kernel_post_.get(), manager().device().get());
    }

    const std::shared_ptr<CLProgram> &program() { return program_; }

    const std::shared_ptr<CLKernel> &kernel() { return kernel_; }

    CLConfigure &configure() { return configure_; }
    const CLConfigure &configure() const { return configure_; }

    private:
    std::size_t size_;

    std::shared_ptr<CLProgram> program_;
    std::shared_ptr<CLKernel> kernel_;
    std::shared_ptr<CLKernel> kernel_post_;
    CLConfigure configure_;
    CLConfigure configure_post_;
}; // class CLCopy
}
} // namespace vsmc::internal

#endif // VSMC_OPENCL_INTERNAL_COPY_HPP
