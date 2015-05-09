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

template <typename Engine, typename CEngine, typename CEngineInit,
    typename CEngineRand>
inline void rng_cl_engine(std::size_t N, std::size_t M,
    const vsmc::CLProgram &program, CEngineInit &cinit, CEngineRand &crand,
    const std::string &name)
{
    vsmc::StopWatch watch;

    typedef typename Engine::result_type rt;

    std::vector<rt> cpp(N * M);
    Engine eng(1);
    watch.reset();
    watch.start();
    for (std::size_t i = 0; i != N * M; ++i)
        cpp[i] = eng();
    watch.stop();
    double tcpp = watch.seconds();

    std::vector<rt> c(N * M);
    CEngine rng;
    cinit(&rng, 1);
    watch.reset();
    watch.start();
    for (std::size_t i = 0; i != N * M; ++i)
        c[i] = crand(&rng);
    watch.stop();
    double tc = watch.seconds();

    std::vector<rt> cl(N * M);
    vsmc::CLBuffer<rt> buffer(N * M);
    vsmc::CLKernel kernel(program, "kernel_" + name);
    vsmc::cl_set_kernel_args(
        kernel, 0, static_cast<cl_ulong>(N), buffer.data());
    vsmc::CLManager<>::instance().run_kernel(kernel, N, 0);
    watch.reset();
    watch.start();
    vsmc::CLManager<>::instance().run_kernel(kernel, N, 0);
    watch.stop();
    vsmc::CLManager<>::instance().read_buffer<rt>(
        buffer.data(), N * M, cl.data());
    double tcl = watch.seconds();

    std::cout << std::setw(20) << std::left << name;
    std::cout << std::setw(20) << std::fixed << std::right << tcpp;
    std::cout << std::setw(20) << std::fixed << std::right << tc;
    std::cout << std::setw(20) << std::fixed << std::right << tcl;
    std::cout << std::setw(20) << std::fixed << std::right << tcpp / tc;
    std::cout << std::setw(20) << std::fixed << std::right << tcpp / tcl;
    std::cout << std::endl;

    for (std::size_t i = 0; i != N * 2; ++i) {
        if (cpp[i] != c[i]) {
            std::cout << "Failure: C      " << name << std::endl;
            break;
        }
    }
    for (std::size_t i = 0; i != N * 2; ++i) {
        if (cpp[i] != cl[i]) {
            std::cout << "Failure: OpenCL " << name << std::endl;
            break;
        }
    }
}

template <typename FPType>
inline void rng_cl(std::size_t N, const vsmc::CLProgram &program)
{
    std::cout << std::string(120, '=') << std::endl;
    std::cout << "Number of samples: " << N << std::endl;
    std::cout << std::string(120, '=') << std::endl;
    std::cout << std::setw(20) << std::left << "Test name";
    std::cout << std::setw(20) << std::right << "Time (C++)";
    std::cout << std::setw(20) << std::right << "Time (C)";
    std::cout << std::setw(20) << std::right << "Time (OpenCL)";
    std::cout << std::setw(20) << std::right << "Speedup (C)";
    std::cout << std::setw(20) << std::right << "Speedup (OpenCL)";
    std::cout << std::endl;
    std::cout << std::string(120, '-') << std::endl;
    rng_cl_engine<vsmc::Philox2x32, vsmc_philox2x32>(N, 2, program,
        vsmc_philox2x32_init, vsmc_philox2x32_rand, "Philox2x32");
    rng_cl_engine<vsmc::Philox4x32, vsmc_philox4x32>(N, 4, program,
        vsmc_philox4x32_init, vsmc_philox4x32_rand, "Philox4x32");
    rng_cl_engine<vsmc::Threefry2x32, vsmc_threefry2x32>(N, 2, program,
        vsmc_threefry2x32_init, vsmc_threefry2x32_rand, "Threefry2x32");
    rng_cl_engine<vsmc::Threefry4x32, vsmc_threefry4x32>(N, 4, program,
        vsmc_threefry4x32_init, vsmc_threefry4x32_rand, "Threefry4x32");
    rng_cl_engine<vsmc::Threefry2x64, vsmc_threefry2x64>(N, 2, program,
        vsmc_threefry2x64_init, vsmc_threefry2x64_rand, "Threefry2x64");
    rng_cl_engine<vsmc::Threefry4x64, vsmc_threefry4x64>(N, 4, program,
        vsmc_threefry4x64_init, vsmc_threefry4x64_rand, "Threefry4x64");
    std::cout << std::string(120, '-') << std::endl;
}

#endif // VSMC_EXAMPLE_RNG_OPENCL_HPP
