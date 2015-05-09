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
#include <vsmc/utility/hdf5io.hpp>
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

inline double rng_cl_normal01(std::size_t N, float *c)
{
    vsmc::StopWatch watch;

    vsmc_rng rng;
    vsmc_rng_init(&rng, 1);
    vsmc_normal01_24 norm01;
    vsmc_normal01_24_init(&norm01, &rng);
    watch.reset();
    watch.start();
    for (std::size_t i = 0; i != N; ++i)
        c[i] = vsmc_normal01_24_rand(&norm01, &rng);
    watch.stop();

    return watch.seconds();
}

inline double rng_cl_normal01(std::size_t N, double *c)
{
    vsmc::StopWatch watch;

    vsmc_rng rng;
    vsmc_rng_init(&rng, 1);
    vsmc_normal01_53 norm01;
    vsmc_normal01_53_init(&norm01, &rng);
    watch.reset();
    watch.start();
    for (std::size_t i = 0; i != N; ++i)
        c[i] = vsmc_normal01_53_rand(&norm01, &rng);
    watch.stop();

    return watch.seconds();
}

template <typename FPType>
inline void rng_cl_normal01(std::size_t N, const vsmc::CLProgram &program,
    std::vector<std::vector<FPType>> &result)
{
    vsmc::StopWatch watch;

    std::vector<FPType> cpp(N);
    vsmc::Philox2x32 eng(1);
    std::normal_distribution<FPType> rnorm(0, 1);
    watch.reset();
    watch.start();
    for (std::size_t i = 0; i != N; ++i)
        cpp[i] = rnorm(eng);
    watch.stop();
    double tcpp = watch.seconds();

    std::vector<FPType> c(N);
    double tc = rng_cl_normal01(N, c.data());

    std::vector<FPType> cl(N);
    vsmc::CLBuffer<FPType> buffer(N);
    vsmc::CLKernel kernel(program, "kernel_Normal01");
    vsmc::cl_set_kernel_args(
        kernel, 0, static_cast<cl_ulong>(N), buffer.data());
    vsmc::CLManager<>::instance().run_kernel(kernel, N, 0);
    watch.reset();
    watch.start();
    vsmc::CLManager<>::instance().run_kernel(kernel, N, 0);
    watch.stop();
    vsmc::CLManager<>::instance().read_buffer<FPType>(
        buffer.data(), N, cl.data());
    double tcl = watch.seconds();

    std::cout << std::setw(20) << std::left << "Normal(0, 1)";
    std::cout << std::setw(20) << std::fixed << std::right << tcpp;
    std::cout << std::setw(20) << std::fixed << std::right << tc;
    std::cout << std::setw(20) << std::fixed << std::right << tcl;
    std::cout << std::setw(20) << std::fixed << std::right << tcpp / tc;
    std::cout << std::setw(20) << std::fixed << std::right << tcpp / tcl;
    std::cout << std::endl;

    result.push_back(std::move(cpp));
    result.push_back(std::move(c));
    result.push_back(std::move(cl));
}

inline double rng_cl_gammak1(std::size_t N, float *c, float shape)
{
    vsmc::StopWatch watch;

    vsmc_rng rng;
    vsmc_rng_init(&rng, 1);
    vsmc_gammak1_24 gammak1;
    vsmc_gammak1_24_init(&gammak1, &rng, shape);
    watch.reset();
    watch.start();
    for (std::size_t i = 0; i != N; ++i)
        c[i] = vsmc_gammak1_24_rand(&gammak1, &rng);
    watch.stop();

    return watch.seconds();
}

inline double rng_cl_gammak1(std::size_t N, double *c, double shape)
{
    vsmc::StopWatch watch;

    vsmc_rng rng;
    vsmc_rng_init(&rng, 1);
    vsmc_gammak1_53 gammak1;
    vsmc_gammak1_53_init(&gammak1, &rng, shape);
    watch.reset();
    watch.start();
    for (std::size_t i = 0; i != N; ++i)
        c[i] = vsmc_gammak1_53_rand(&gammak1, &rng);
    watch.stop();

    return watch.seconds();
}

template <typename FPType>
inline void rng_cl_gammak1(std::size_t N, const vsmc::CLProgram &program,
    FPType shape, std::vector<std::vector<FPType>> &result)
{
    vsmc::StopWatch watch;

    std::vector<FPType> cpp(N);
    vsmc::Philox2x32 eng(1);
    std::gamma_distribution<FPType> rgamma(shape, 1);
    watch.reset();
    watch.start();
    for (std::size_t i = 0; i != N; ++i)
        cpp[i] = rgamma(eng);
    watch.stop();
    double tcpp = watch.seconds();

    std::vector<FPType> c(N);
    double tc = rng_cl_gammak1(N, c.data(), shape);

    std::vector<FPType> cl(N);
    vsmc::CLBuffer<FPType> buffer(N);
    vsmc::CLKernel kernel(program, "kernel_GammaK1");
    vsmc::cl_set_kernel_args(
        kernel, 0, static_cast<cl_ulong>(N), buffer.data(), shape);
    vsmc::CLManager<>::instance().run_kernel(kernel, N, 0);
    watch.reset();
    watch.start();
    vsmc::CLManager<>::instance().run_kernel(kernel, N, 0);
    watch.stop();
    vsmc::CLManager<>::instance().read_buffer<FPType>(
        buffer.data(), N, cl.data());
    double tcl = watch.seconds();

    std::stringstream ss;
    ss << "Gamma(" << shape << ", 1)";
    std::cout << std::setw(20) << std::left << ss.str();
    std::cout << std::setw(20) << std::fixed << std::right << tcpp;
    std::cout << std::setw(20) << std::fixed << std::right << tc;
    std::cout << std::setw(20) << std::fixed << std::right << tcl;
    std::cout << std::setw(20) << std::fixed << std::right << tcpp / tc;
    std::cout << std::setw(20) << std::fixed << std::right << tcpp / tcl;
    std::cout << std::endl;

    result.push_back(std::move(cpp));
    result.push_back(std::move(c));
    result.push_back(std::move(cl));
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

    std::vector<std::vector<FPType>> result;
    std::cout << std::string(120, '-') << std::endl;
    rng_cl_normal01<FPType>(N, program, result);
    std::cout << std::string(120, '-') << std::endl;
    rng_cl_gammak1<FPType>(N, program, static_cast<FPType>(0.01), result);
    rng_cl_gammak1<FPType>(N, program, static_cast<FPType>(0.1), result);
    rng_cl_gammak1<FPType>(N, program, static_cast<FPType>(1), result);
    rng_cl_gammak1<FPType>(N, program, static_cast<FPType>(10), result);
    rng_cl_gammak1<FPType>(N, program, static_cast<FPType>(100), result);
    std::cout << std::string(120, '-') << std::endl;

#if VSMC_HAS_HDF5
    std::size_t nrow = N;
    std::size_t ncol = result.size();
    std::vector<FPType> mat(nrow * ncol);
    FPType *first = mat.data();
    for (std::size_t i = 0; i != ncol; ++i)
        first = std::copy(result[i].begin(), result[i].end(), first);
    vsmc::hdf5store_matrix<vsmc::ColMajor, FPType>(
        nrow, ncol, "rng_cl.h5", "result", mat.data());
#else
    std::ofstream rng_cl_txt("rng_cl.txt");
    for (std::size_t i = 0; i != N; ++i) {
        for (std::size_t j = 0; j != result.size(); ++j)
            rng_cl_txt << result[j][i] << '\t';
        rng_cl_txt << '\n';
    }
    rng_cl_txt.close();
#endif
}

#endif // VSMC_EXAMPLE_RNG_OPENCL_HPP
