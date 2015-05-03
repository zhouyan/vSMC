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

#include <vsmc/opencl/cl_manager.hpp>
#include <vsmc/opencl/cl_manip.hpp>
#include <vsmc/utility/stop_watch.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#if VSMC_HAS_HDF5
#include "rng_output_data_hdf5.hpp"
#else
#include "rng_output_data.hpp"
#endif

struct rng_device;

inline void set_kernel_vec(const std::string &kbase,
    const vsmc::CLProgram &program, std::vector<vsmc::CLKernel> &kernel_vec,
    bool clear_first = true)
{
    if (clear_first)
        kernel_vec.clear();
    std::string kernel_name_2x32(kbase + "_2x32_ker");
    std::string kernel_name_4x32(kbase + "_4x32_ker");
    kernel_vec.push_back(vsmc::CLKernel(program, kernel_name_2x32.c_str()));
    kernel_vec.push_back(vsmc::CLKernel(program, kernel_name_4x32.c_str()));
}

template <typename FP, typename DistType, typename Eng>
inline vsmc::StopWatch rng_cl_test_cpp(
    std::size_t N, DistType &dist, Eng &eng, std::vector<FP> &host)
{
    vsmc::StopWatch watch;
    watch.start();
    for (std::size_t i = 0; i != N; ++i)
        host[i] = dist(eng);
    watch.stop();
    std::cout << std::right << std::fixed << std::setw(20)
              << watch.milliseconds();

    return watch;
}

template <typename FP>
inline void rng_cl_test_ocl(std::size_t N,
    std::vector<vsmc::CLKernel> &kernel_vec, const vsmc::CLMemory &buffer,
    std::vector<FP> &host, double tcpp)
{
    vsmc::CLManager<rng_device> &manager =
        vsmc::CLManager<rng_device>::instance();

    std::size_t offset = N;
    for (std::size_t i = 0; i != kernel_vec.size(); ++i, offset += N) {
        std::size_t global_size, local_size;
        vsmc::cl_preferred_work_size(
            N, kernel_vec[i], manager.device(), global_size, local_size);
        vsmc::cl_set_kernel_args(
            kernel_vec[i], 0, static_cast<cl_ulong>(N), buffer);

        // Run kernel once to cache
        manager.run_kernel(kernel_vec[i], N, local_size);
        vsmc::StopWatch watch;
        watch.start();
        manager.run_kernel(kernel_vec[i], N, local_size);
        watch.stop();
        manager.read_buffer<FP>(buffer, N, &host[offset]);

        double tocl = watch.milliseconds();
        std::cout << std::right << std::fixed << std::setw(20) << tocl;
        std::cout << std::right << std::fixed << std::setw(20) << tcpp / tocl;
    }
}

template <typename FP, typename DistType, typename Eng>
inline void rng_test_dist(std::size_t N,
    std::vector<vsmc::CLKernel> &kernel_vec, const std::string &dname,
    DistType &dist, Eng &eng, const vsmc::CLMemory &buffer,
    std::vector<FP> &host)
{
    std::cout << std::left << std::setw(20) << dname;
    vsmc::StopWatch tcpp(rng_cl_test_cpp(N, dist, eng, host));
    rng_cl_test_ocl(N, kernel_vec, buffer, host, tcpp.milliseconds());
    std::cout << std::endl;
}

template <typename FP>
inline void rng_cl_test(std::size_t N, const vsmc::CLProgram &program,
    std::vector<std::string> &dnames, std::vector<std::string> &unames,
    std::vector<std::string> &rcodes, std::vector<std::vector<FP>> &values,
    std::vector<std::vector<cl_uint>> &values_ui)
{
    vsmc::CLManager<rng_device> &manager =
        vsmc::CLManager<rng_device>::instance();

    std::mt19937_64 eng;
    std::vector<vsmc::CLKernel> kernel_vec;

    vsmc::CLMemory buffer(manager.create_buffer<FP>(N));
    vsmc::CLMemory buffer_ui_0(manager.create_buffer<cl_uint>(N));
    vsmc::CLMemory buffer_ui_1(manager.create_buffer<cl_uint>(N));
    vsmc::CLMemory buffer_ui_2(manager.create_buffer<cl_uint>(N));
    vsmc::CLMemory buffer_ui_3(manager.create_buffer<cl_uint>(N));
    std::vector<FP> host(3 * N);
    std::vector<cl_uint> host_ui_0(N);
    std::vector<cl_uint> host_ui_1(N);
    std::vector<cl_uint> host_ui_2(N);
    std::vector<cl_uint> host_ui_3(N);

    dnames.clear();
    unames.clear();
    rcodes.clear();
    values.clear();
    values_ui.clear();

    // Test R123
    std::cout << std::string(120, '=') << std::endl;
    std::cout << std::left << std::setw(20) << "Engines";
    std::cout << std::right << std::setw(20) << "Time (ms)";
    std::cout << std::endl;
    std::cout << std::string(120, '-') << std::endl;

    std::vector<std::string> engines;
    std::vector<std::string> engnames;
    engines.push_back("threefry");
    engines.push_back("philox");
    engines.push_back("cburng_threefry");
    engines.push_back("cburng_philox");
    kernel_vec.clear();
    for (std::size_t i = 0; i != engines.size(); ++i) {
        set_kernel_vec(engines[i], program, kernel_vec, false);
        unames.push_back(engines[i] + ".2x32.v0");
        unames.push_back(engines[i] + ".2x32.v1");
        unames.push_back(engines[i] + ".2x32.v2");
        unames.push_back(engines[i] + ".2x32.v3");
        unames.push_back(engines[i] + ".4x32.v0");
        unames.push_back(engines[i] + ".4x32.v1");
        unames.push_back(engines[i] + ".4x32.v2");
        unames.push_back(engines[i] + ".4x32.v3");
        engnames.push_back(engines[i] + "2x32");
        engnames.push_back(engines[i] + "4x32");
    }
    for (std::size_t i = 0; i != kernel_vec.size(); ++i) {
        std::size_t global_size = 0;
        std::size_t local_size = 0;
        vsmc::cl_preferred_work_size(
            N, kernel_vec[i], manager.device(), global_size, local_size);
        vsmc::cl_set_kernel_args(kernel_vec[i], 0, static_cast<cl_ulong>(N),
            buffer_ui_0, buffer_ui_1, buffer_ui_2, buffer_ui_3);

        // Run kernel once first to cache
        manager.run_kernel(kernel_vec[i], N, local_size);
        vsmc::StopWatch watch;
        watch.start();
        manager.run_kernel(kernel_vec[i], N, local_size);
        watch.stop();

        std::cout << std::left << std::setw(20) << engnames[i];
        std::cout << std::right << std::fixed << std::setw(20);
        std::cout << watch.milliseconds() << std::endl;

        manager.read_buffer<cl_uint>(buffer_ui_0, N, host_ui_0.data());
        manager.read_buffer<cl_uint>(buffer_ui_1, N, host_ui_1.data());
        manager.read_buffer<cl_uint>(buffer_ui_2, N, host_ui_2.data());
        manager.read_buffer<cl_uint>(buffer_ui_3, N, host_ui_3.data());
        values_ui.push_back(host_ui_0);
        values_ui.push_back(host_ui_1);
        values_ui.push_back(host_ui_2);
        values_ui.push_back(host_ui_3);
    }

    // Test distributions
    std::cout << std::string(120, '=') << std::endl;
    std::cout << std::left << std::setw(20) << "Distribuiton";
    std::cout << std::right << std::setw(20) << "MT19937 Time (ms)";
    std::cout << std::right << std::setw(20) << "OCL 2x32 Time (ms)";
    std::cout << std::right << std::setw(20) << "OCL 2x32 Speedup";
    std::cout << std::right << std::setw(20) << "OCL 4x32 Time (ms)";
    std::cout << std::right << std::setw(20) << "OCL 4x32 Speedup";
    std::cout << std::endl;
    std::cout << std::string(120, '-') << std::endl;

    // Test u01
    std::uniform_real_distribution<FP> runif(0, 1);
    dnames.push_back("u01");
    rcodes.push_back("runif(N)");
    set_kernel_vec("u01", program, kernel_vec);
    rng_test_dist(N, kernel_vec, "u01", runif, eng, buffer, host);
    values.push_back(host);

    // Test normal01
    std::normal_distribution<FP> rnorm(0, 1);
    dnames.push_back("normal01");
    rcodes.push_back("rnorm(N)");
    set_kernel_vec("normal01", program, kernel_vec);
    rng_test_dist(N, kernel_vec, "normal01", rnorm, eng, buffer, host);
    values.push_back(host);

    // Test gammak1
    std::vector<FP> gammak1_shape;
    gammak1_shape.push_back(static_cast<FP>(0.1));
    gammak1_shape.push_back(static_cast<FP>(0.5));
    gammak1_shape.push_back(static_cast<FP>(1.0));
    gammak1_shape.push_back(static_cast<FP>(1.5));
    gammak1_shape.push_back(static_cast<FP>(2.5));
    gammak1_shape.push_back(static_cast<FP>(3.5));
    gammak1_shape.push_back(static_cast<FP>(4.5));
    gammak1_shape.push_back(static_cast<FP>(10.));
    gammak1_shape.push_back(static_cast<FP>(15.));
    gammak1_shape.push_back(static_cast<FP>(30.));
    for (std::size_t i = 0; i != gammak1_shape.size(); ++i) {
        std::stringstream dname;
        dname << "gammak1_" << gammak1_shape[i];

        std::stringstream rcode;
        rcode << "rgamma(N, " << gammak1_shape[i] << ")";

        std::gamma_distribution<FP> rgamma(gammak1_shape[i], 1);
        dnames.push_back(dname.str());
        rcodes.push_back(rcode.str());
        set_kernel_vec("gammak1", program, kernel_vec);
        for (std::size_t k = 0; k != kernel_vec.size(); ++k)
            vsmc::cl_set_kernel_args(kernel_vec[k], 2, gammak1_shape[i]);
        rng_test_dist(N, kernel_vec, dname.str(), rgamma, eng, buffer, host);
        values.push_back(host);
    }
    std::cout << std::string(120, '=') << std::endl;
}

template <typename FP>
inline void rng_cl_output(std::size_t N, std::vector<std::string> &dnames,
    std::vector<std::string> &unames, std::vector<std::string> &rcodes,
    std::vector<std::vector<FP>> &values,
    std::vector<std::vector<cl_uint>> &values_ui)
{

    rng_output_data("rng_cl", dnames, values);
    rng_output_data("rng_cl_uint", unames, values_ui);

    std::size_t M = rcodes.size();
    if (M == 0)
        return;

    std::ofstream output_file("rng_cl_dt.R");
    output_file << "N <- " << N << '\n';
    output_file << "refoutput <- data.frame(\n";
    for (std::size_t j = 0; j != M; ++j) {
        output_file << dnames[j] << " = " << rcodes[j];
        output_file << (j == M - 1 ? ')' : ',') << '\n';
    }
    output_file.close();
    output_file.clear();
}

template <typename FT>
inline void rng_cl(std::size_t N, const vsmc::CLProgram &program)
{
    std::vector<std::string> dnames;
    std::vector<std::string> unames;
    std::vector<std::string> rcodes;
    std::vector<std::vector<FT>> values;
    std::vector<std::vector<cl_uint>> values_ui;
    vsmc::StopWatch watch;

    rng_cl_test<FT>(N, program, dnames, unames, rcodes, values, values_ui);
    rng_cl_output<FT>(N, dnames, unames, rcodes, values, values_ui);
}

#endif // VSMC_EXAMPLE_RNG_OPENCL_HPP
