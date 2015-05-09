//============================================================================
// vSMC/example/rng/include/rng_cl.hpp
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

#ifndef VSMC_EXAMPLE_RNG_OPENCL_HPP
#define VSMC_EXAMPLE_RNG_OPENCL_HPP

#include <vsmc/opencl/cl_buffer.hpp>
#include <vsmc/opencl/cl_manager.hpp>
#include <vsmc/opencl/cl_manip.hpp>
#include <vsmc/rng/philox.hpp>
#include <vsmc/rng/threefry.hpp>
#include <vsmc/rngc/rngc.h>
#include <vsmc/utility/stop_watch.hpp>

#if VSMC_HAS_HDF5
#include "rng_output_data_hdf5.hpp"
#else
#include "rng_output_data.hpp"
#endif

inline void rng_cl_philox2x32(std::size_t N, const vsmc::CLProgram &program)
{
    vsmc::StopWatch watch;

    std::vector<std::uint32_t> cpp(N * 2);
    vsmc::Philox2x32 eng(1);
    watch.reset();
    watch.start();
    for (std::size_t i = 0; i != N * 2; ++i)
        cpp[i] = eng();
    watch.stop();
    double tcpp = watch.seconds();

    std::vector<std::uint32_t> c(N * 2);
    vsmc_philox2x32 rng;
    vsmc_philox2x32_init(&rng, 1);
    watch.reset();
    watch.start();
    for (std::size_t i = 0; i != N * 2; ++i)
        c[i] = vsmc_philox2x32_rand(&rng);
    watch.stop();
    double tc = watch.seconds();

    std::vector<cl_uint> cl(N * 2);
    vsmc::CLBuffer<cl_uint> buffer(N * 2);
    vsmc::CLKernel kernel(program, "kernel_philox2x32");
    vsmc::cl_set_kernel_args(
        kernel, 0, static_cast<cl_ulong>(N), buffer.data());
    vsmc::CLManager<>::instance().run_kernel(kernel, N, 0);
    watch.reset();
    watch.start();
    vsmc::CLManager<>::instance().run_kernel(kernel, N, 0);
    watch.stop();
    vsmc::CLManager<>::instance().read_buffer<cl_uint>(
        buffer.data(), N * 2, cl.data());
    double tcl = watch.seconds();

    std::cout << std::setw(20) << std::left << "Philox2x32";
    std::cout << std::setw(20) << std::right << tcpp;
    std::cout << std::setw(20) << std::right << tc;
    std::cout << std::setw(20) << std::right << tcl;
    std::cout << std::setw(20) << std::right << tcpp / tc;
    std::cout << std::setw(20) << std::right << tcpp / tcl;
    std::cout << std::endl;

    for (std::size_t i = 0; i != N * 2; ++i) {
        if (cpp[i] != c[i]) {
            std::cout << "Failure: C Philox2x32" << std::endl;
            break;
        }
    }
    for (std::size_t i = 0; i != N * 2; ++i) {
        if (cpp[i] != cl[i]) {
            std::cout << "Failure: OpenCL Philox2x32" << std::endl;
            break;
        }
    }
}

inline void rng_cl_philox4x32(std::size_t N, const vsmc::CLProgram &program)
{
    vsmc::StopWatch watch;

    std::vector<std::uint32_t> cpp(N * 4);
    vsmc::Philox4x32 eng(1);
    watch.reset();
    watch.start();
    for (std::size_t i = 0; i != N * 4; ++i)
        cpp[i] = eng();
    watch.stop();
    double tcpp = watch.seconds();

    std::vector<std::uint32_t> c(N * 4);
    vsmc_philox4x32 rng;
    vsmc_philox4x32_init(&rng, 1);
    watch.reset();
    watch.start();
    for (std::size_t i = 0; i != N * 4; ++i)
        c[i] = vsmc_philox4x32_rand(&rng);
    watch.stop();
    double tc = watch.seconds();

    std::vector<cl_uint> cl(N * 4);
    vsmc::CLBuffer<cl_uint> buffer(N * 4);
    vsmc::CLKernel kernel(program, "kernel_philox4x32");
    vsmc::cl_set_kernel_args(
        kernel, 0, static_cast<cl_ulong>(N), buffer.data());
    vsmc::CLManager<>::instance().run_kernel(kernel, N, 0);
    watch.reset();
    watch.start();
    vsmc::CLManager<>::instance().run_kernel(kernel, N, 0);
    watch.stop();
    vsmc::CLManager<>::instance().read_buffer<cl_uint>(
        buffer.data(), N * 4, cl.data());
    double tcl = watch.seconds();

    std::cout << std::setw(20) << std::left << "Philox4x32";
    std::cout << std::setw(20) << std::right << tcpp;
    std::cout << std::setw(20) << std::right << tc;
    std::cout << std::setw(20) << std::right << tcl;
    std::cout << std::setw(20) << std::right << tcpp / tc;
    std::cout << std::setw(20) << std::right << tcpp / tcl;
    std::cout << std::endl;

    for (std::size_t i = 0; i != N * 4; ++i) {
        if (cpp[i] != c[i]) {
            std::cout << "Failure: C Philox4x32" << std::endl;
            break;
        }
    }
    for (std::size_t i = 0; i != N * 4; ++i) {
        if (cpp[i] != cl[i]) {
            std::cout << "Failure: OpenCL Philox4x32" << std::endl;
            break;
        }
    }
}

template <typename FPType>
inline void rng_cl(std::size_t N, const vsmc::CLProgram &program)
{
    std::cout << std::string(120, '-') << std::endl;
    rng_cl_philox2x32(N, program);
    rng_cl_philox4x32(N, program);
    std::cout << std::string(120, '-') << std::endl;
}

#endif // VSMC_EXAMPLE_RNG_OPENCL_HPP
