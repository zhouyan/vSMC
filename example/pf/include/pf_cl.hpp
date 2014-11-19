//============================================================================
// vSMC/vSMCExample/pf/include/pf_cl.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013,2014, Yan Zhou
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

#ifndef VSMC_EXAMPLE_PF_CL_HPP
#define VSMC_EXAMPLE_PF_CL_HPP

#include <vsmc/core/sampler.hpp>
#include <vsmc/opencl/adapter.hpp>
#include <vsmc/opencl/backend_cl.hpp>
#include <Random123/array.h>

#if VSMC_HAS_HDF5
#include <vsmc/utility/hdf5io.hpp>
#endif

#include <fstream>
#include <iostream>

#define PF_CV_DO(Res) cv_do(sampler, vsmc::Res, argv, "."#Res);

static const std::size_t DataNum = 100;
static const std::size_t ParticleNum = 10000;
static const std::size_t StateSize = 4 * sizeof(cl_float);

class cv : public vsmc::StateCL<StateSize, cl_float, vsmc::CLDefault>
{
    public :

    cv (size_type N) : vsmc::StateCL<StateSize, cl_float, vsmc::CLDefault>(N)
    {
        counter_ = manager().create_buffer<struct r123array4x32>(N);
    }

    const cl::Buffer &obs_x () const {return obs_x_;}
    const cl::Buffer &obs_y () const {return obs_y_;}
    const cl::Buffer &counter () const {return counter_;}

    void read_data (const char *file)
    {
        if (!file)
            return;

        std::vector<cv::fp_type> x(DataNum);
        std::vector<cv::fp_type> y(DataNum);
        std::ifstream data(file);
        for (std::size_t i = 0; i != DataNum; ++i)
            data >> x[i] >> y[i];
        data.close();

        obs_x_ = manager().create_buffer<cv::fp_type>(x.begin(), x.end());
        obs_y_ = manager().create_buffer<cv::fp_type>(y.begin(), y.end());
    }

    private :

    cl::Buffer obs_x_;
    cl::Buffer obs_y_;
    cl::Buffer counter_;
};

class cv_init : public vsmc::InitializeCL<cv>
{
    public :

    void initialize_state (std::string &kernel_name)
    {kernel_name = std::string("cv_init");}

    void initialize_param (vsmc::Particle<cv> &particle, void *file)
    {particle.value().read_data(static_cast<const char *>(file));}

    void pre_processor (vsmc::Particle<cv> &particle)
    {
        if (log_weight_.size() != particle.size()) {
            log_weight_device_ = particle.value().manager()
                .create_buffer<cv::fp_type>(particle.size());
            log_weight_.resize(particle.size());
        }

        vsmc::cl_set_kernel_args(
                kernel(), kernel_args_offset(),
                log_weight_device_,
                particle.value().obs_x(),
                particle.value().obs_y(),
                particle.value().counter());
    }

    void post_processor (vsmc::Particle<cv> &particle)
    {
        particle.value().manager().read_buffer<cv::fp_type>(
                log_weight_device_, particle.size(), &log_weight_[0]);
        particle.weight_set().set_log_weight(&log_weight_[0]);
    }

    private :

    cl::Buffer log_weight_device_;
    std::vector<double> log_weight_;
};

class cv_move : public vsmc::MoveCL<cv>
{
    public :

    void move_state (std::size_t, std::string &kernel_name)
    {kernel_name = std::string("cv_move");}

    void pre_processor (std::size_t, vsmc::Particle<cv> &particle)
    {
        if (inc_weight_.size() != particle.size()) {
            inc_weight_device_ = particle.value().manager()
                .create_buffer<cv::fp_type>(particle.size());
            inc_weight_.resize(particle.size());
        }

        vsmc::cl_set_kernel_args(
                kernel(), kernel_args_offset(),
                inc_weight_device_,
                particle.value().obs_x(),
                particle.value().obs_y(),
                particle.value().counter());
    }

    void post_processor (std::size_t, vsmc::Particle<cv> &particle)
    {
        particle.value().manager().read_buffer<cv::fp_type>(
                inc_weight_device_, particle.size(), &inc_weight_[0]);
        particle.weight_set().add_log_weight(&inc_weight_[0]);
    }

    private :

    cl::Buffer inc_weight_device_;
    std::vector<double> inc_weight_;
};

inline void cv_do (vsmc::Sampler<cv> &sampler, vsmc::ResampleScheme res,
        char **argv, const std::string &name)
{
    sampler.resample_scheme(res);
    sampler.resample_threshold(0.5);

#if VSMC_HAS_HDF5
    sampler.initialize(argv[1]);
    vsmc::hdf5store<vsmc::RowMajor, cv::fp_type>(
            sampler.particle().value(),
            argv[2] + name + ".trace.hdf5", "Trace.0");
    for (std::size_t i = 0; i != DataNum - 1; ++i) {
        std::stringstream ss;
        ss << "Trace." << (i + 1);
        sampler.iterate();
        vsmc::hdf5store<vsmc::RowMajor, cv::fp_type>(
                sampler.particle().value(),
                argv[2] + name + ".trace.hdf5", ss.str(), true);
    }
#else
    sampler.initialize(argv[1]);
    sampler.iterate(DataNum - 1);
#endif

    std::string est_file_name(argv[2] + name + ".txt");
    std::ofstream est_file;
    est_file.open(est_file_name.c_str());
    est_file << sampler << std::endl;
    est_file.close();

#if VSMC_HAS_HDF5
    vsmc::hdf5store(sampler, argv[2] + name + ".hdf5", "Sampler");
#endif
}


#endif // VSMC_EXAMPLE_PF_CL_HPP
