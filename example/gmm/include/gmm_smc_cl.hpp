//============================================================================
// vSMC/example/gmm/include/gmm_smc_cl.hpp
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

#ifndef VSMC_EXAMPLE_GMM_SMC_CL_HPP
#define VSMC_EXAMPLE_GMM_SMC_CL_HPP

#include <cstddef>

const std::size_t InitCompNum = 4;
const std::size_t MinCompNum = 1;
const std::size_t MaxCompNum = 10;

#include "common.hpp"
#include <vsmc/opencl/adapter.hpp>
#include <vsmc/opencl/backend_cl.hpp>
#include <vsmc/opencl/cl_query.hpp>
#include <Random123/array.h>

#ifdef VSMC_GMM_SMC_CL_MPI
#include <vsmc/mpi/backend_mpi.hpp>
#endif

struct data_info
{
    const std::size_t data_num;
    const char *file_name;

    data_info (std::size_t num, const char *file) :
        data_num(num), file_name(file) {}
};


template <typename FPType>
#ifdef VSMC_GMM_SMC_CL_MPI
class gmm_state : public vsmc::StateMPI<vsmc::StateCL<vsmc::Dynamic, FPType> >
#else
class gmm_state : public vsmc::StateCL<vsmc::Dynamic, FPType>
#endif
{
    public :

#ifdef VSMC_GMM_SMC_CL_MPI
    typedef vsmc::StateMPI<vsmc::StateCL<vsmc::Dynamic, FPType> > base;
#else
    typedef vsmc::StateCL<vsmc::Dynamic, FPType> base;
#endif
    typedef typename base::size_type size_type;
    typedef typename base::fp_type fp_type;

    gmm_state (size_type N) : base(N)
    {
#if VSMC_OPENCL_VERSION >= 120
        if (this->manager().opencl_version() >= 120) {
#ifdef VSMC_GMM_SMC_CL_MPI
            this->update_state(CL_MEM_READ_WRITE|CL_MEM_HOST_READ_ONLY);
#else
            this->update_state(CL_MEM_READ_WRITE|CL_MEM_HOST_NO_ACCESS);
#endif
            counter_ = this->manager().template
                create_buffer<struct r123array2x32>(N,
                        CL_MEM_READ_WRITE|CL_MEM_HOST_NO_ACCESS);
        } else {
            counter_ = this->manager().template
                create_buffer<struct r123array2x32>(N);
        }
#else
        counter_ = this->manager().template
            create_buffer<struct r123array2x32>(N);
#endif
    }

    const cl::Buffer &obs () const {return obs_;}
    const cl::Buffer &counter () const {return counter_;}

    std::size_t obs_num () const {return obs_num_;}
    std::size_t comp_num () const {return comp_num_;}
    fp_type alpha () const {return alpha_;}
    fp_type alpha_inc () const {return alpha_inc_;}
    double zconst () const {return zconst_;}
    double &zconst () {return zconst_;}

    void comp_num (std::size_t num)
    {
        this->resize_state((num * 3 + 2) * sizeof(fp_type));
        comp_num_ = num;
    }

    void alpha (fp_type a)
    {
        a = a < 1 ? a : 1;
        a = a > 0 ? a : 0;
        fp_type a_inc = a > 0 ? a - alpha_ : 0;
        alpha_ = a;
        alpha_inc_ = a_inc;
    }

    void read_data (const data_info *info)
    {
        std::ifstream data;
        data.open(info->file_name);
        std::vector<fp_type> obs(info->data_num);
        obs_num_ = obs.size();
        for (std::size_t i = 0; i != obs.size(); ++i)
            data >> obs[i];
#if VSMC_OPENCL_VERSION >= 120
        if (this->manager().opencl_version() >= 120) {
            obs_ = this->manager().template
                create_buffer<fp_type>(obs.size(),
                        CL_MEM_READ_ONLY|CL_MEM_COPY_HOST_PTR|
                        CL_MEM_HOST_WRITE_ONLY, &obs[0]);
        } else {
            obs_ = this->manager().template
                create_buffer<fp_type>(obs.size(),
                        CL_MEM_READ_ONLY|CL_MEM_COPY_HOST_PTR, &obs[0]);
        }
#else
        obs_ = this->manager().template
            create_buffer<fp_type>(obs.size(),
                    CL_MEM_READ_ONLY|CL_MEM_COPY_HOST_PTR, &obs[0]);
#endif
        data.close();
        data.clear();

        // find min and max of data
        fp_type xmax = obs[0];
        fp_type xmin = obs[0];
        for (std::size_t i = 0; i != obs.size(); ++i) {
            if (obs[i] > xmax)
                xmax = obs[i];
            if (obs[i] < xmin)
                xmin = obs[i];
        }
        fp_type kappa = 1 / ((xmax - xmin) * (xmax - xmin));

        // store them for future use
        mu0_    = 0.5f * (xmin + xmax);
        sd0_    = static_cast<fp_type>(1 / sqrt(kappa));
        shape0_ = 2;
        scale0_ = 50 * kappa;
    }

    fp_type mu0    () const {return mu0_;}
    fp_type sd0    () const {return sd0_;}
    fp_type shape0 () const {return shape0_;}
    fp_type scale0 () const {return scale0_;}

    fp_type mu_sd     () const {return mu_sd_;}
    fp_type lambda_sd () const {return lambda_sd_;}
    fp_type weight_sd () const {return weight_sd_;}

    fp_type &mu_sd     () {return mu_sd_;}
    fp_type &lambda_sd () {return lambda_sd_;}
    fp_type &weight_sd () {return weight_sd_;}

    private :

    cl::Buffer obs_;
    cl::Buffer counter_;

    std::size_t obs_num_;
    std::size_t comp_num_;
    fp_type alpha_;
    fp_type alpha_inc_;
    double zconst_;

    fp_type mu0_;    // prior mean for mu
    fp_type sd0_;    // prior sd for mu
    fp_type shape0_; // prior shape for lambda
    fp_type scale0_; // prior scale for lambda

    fp_type mu_sd_;
    fp_type lambda_sd_;
    fp_type weight_sd_;
};

template <typename FPType>
class gmm_init : public vsmc::InitializeCL<gmm_state<FPType> >
{
    public :

    void initialize_state (std::string &kernel_name)
    {kernel_name = std::string("gmm_init");}

    void initialize_param (vsmc::Particle<gmm_state<FPType> > &particle,
            void *info)
    {
        if (info)
            particle.value().read_data(static_cast<const data_info *>(info));
    }

    void pre_processor (vsmc::Particle<gmm_state<FPType> > &particle)
    {
        gmm_state<FPType> &state = particle.value();
        particle.weight_set().set_equal_weight();
        state.alpha(0);
        state.zconst() = 0;

        FPType shape0 = state.shape0();
        FPType scale0 = state.scale0();

        vsmc::cxx11::gamma_distribution<> rlambda(shape0, scale0);
        vsmc::cxx11::gamma_distribution<> rweight(1.0, 1.0);

        const std::size_t cn = state.comp_num();
        std::vector<FPType> lambda_init(state.size() * cn);
        std::vector<FPType> weight_init(state.size() * cn);
        std::vector<FPType> sum_w(state.size());
        for (std::size_t i = 0; i != sum_w.size(); ++i)
            sum_w[i] = 0;
        for (std::size_t d = 0; d != state.comp_num(); ++d) {
            typename vsmc::Particle<gmm_state<FPType> >::size_type i0 =
                d * state.size();
            for (typename vsmc::Particle<gmm_state<FPType> >::size_type i = 0;
                    i != state.size(); ++i) {
                lambda_init[i0 + i] = static_cast<FPType>(
                        rlambda(particle.rng(i)));
                weight_init[i0 + i] = static_cast<FPType>(
                        rweight(particle.rng(i)));
                sum_w[i] += weight_init[i0 + i];
            }
        }
        for (std::size_t d = 0; d != state.comp_num(); ++d) {
            typename vsmc::Particle<gmm_state<FPType> >::size_type i0 =
                d * state.size();
            for (typename vsmc::Particle<gmm_state<FPType> >::size_type i = 0;
                    i != state.size(); ++i) {
                weight_init[i0 + i] /= sum_w[i];
            }
        }

        lambda_init_ = state.manager().template create_buffer<FPType>(
                lambda_init.size());
        weight_init_ = state.manager().template create_buffer<FPType>(
                weight_init.size());

        state.manager().template write_buffer<FPType>(
                lambda_init_, lambda_init.size(), &lambda_init[0]);
        state.manager().template write_buffer<FPType>(
                weight_init_, weight_init.size(), &weight_init[0]);

        vsmc::cl_set_kernel_args(this->kernel(),
                this->kernel_args_offset(),
                lambda_init_, weight_init_, state.obs(),
                static_cast<FPType>(state.mu0()),
                static_cast<FPType>(state.sd0()),
                static_cast<FPType>(state.shape0()),
                static_cast<FPType>(state.scale0()),
                state.counter());
    }


    private :

    cl::Buffer lambda_init_;
    cl::Buffer weight_init_;
};

template <typename FPType>
class gmm_move_smc : public vsmc::MoveCL<gmm_state<FPType> >
{
    public :

    gmm_move_smc (std::size_t iter_num) : iter_num_(iter_num) {}

    void move_state (std::size_t, std::string &kernel_name)
    {kernel_name = std::string("gmm_move_smc");}

    void pre_processor (std::size_t iter,
            vsmc::Particle<gmm_state<FPType> > &particle)
    {
        using std::sqrt;

        gmm_state<FPType> &state = particle.value();
        if (exp_weight_.size() != state.size()) {
            exp_weight_.resize(state.size());
#if VSMC_OPENCL_VERSION >= 120
            if (state.manager().opencl_version() >= 120) {
                exp_weight_device_ = state.manager().template
                    create_buffer<FPType>(state.size(),
                            CL_MEM_WRITE_ONLY|CL_MEM_HOST_READ_ONLY|
                            CL_MEM_USE_HOST_PTR, &exp_weight_[0]);
            } else {
                exp_weight_device_ = state.manager().template
                    create_buffer<FPType>(state.size(),
                            CL_MEM_WRITE_ONLY|CL_MEM_USE_HOST_PTR,
                            &exp_weight_[0]);
            }
#else
            exp_weight_device_ = state.manager().template
                create_buffer<FPType>(state.size(),
                        CL_MEM_WRITE_ONLY|CL_MEM_USE_HOST_PTR,
                        &exp_weight_[0]);
#endif
        }

        FPType a = static_cast<FPType>(iter) / iter_num_;
        a = a * a;
        state.alpha(a);

        a = a < 0.05f ? 0.05f : a;
        state.mu_sd()     = 0.15f / a;
        state.lambda_sd() = (1 + sqrt(1 / a)) * 0.15f;
        state.weight_sd() = (1 + sqrt(1 / a)) * 0.20f;

        vsmc::cl_set_kernel_args(this->kernel(),
                this->kernel_args_offset(), exp_weight_device_,
                static_cast<FPType>(state.alpha_inc()));
    }

    void post_processor (std::size_t,
            vsmc::Particle<gmm_state<FPType> > &particle)
    {
        using std::exp;

        gmm_state<FPType> &state = particle.value();
        state.manager().template read_buffer<FPType>(
                exp_weight_device_, state.size(), &exp_weight_[0]);

        double sum = 0;
        const double *wptr = particle.weight_set().weight_data();
        for (typename vsmc::Particle<gmm_state<FPType> >::size_type i = 0;
                i != state.size(); ++i) {
            sum += wptr[i] * exp_weight_[i];
        }

#ifdef VSMC_GMM_SMC_CL_MPI
        double ssum;
        ::boost::mpi::all_reduce(particle.value().world(),
                sum, ssum, std::plus<double>());
        sum = ssum;
#endif

        state.zconst() += log(sum);
        particle.weight_set().mul_weight(&exp_weight_[0]);
    }

    private :

    std::size_t iter_num_;
    cl::Buffer exp_weight_device_;
    std::vector<FPType> exp_weight_;
};

template <typename Move, typename FPType>
inline void gmm_move_set_args (Move *move, const gmm_state<FPType> &state,
        FPType sd)
{
    vsmc::cl_set_kernel_args(move->kernel(), move->kernel_args_offset(),
            state.obs(),
            static_cast<FPType>(state.alpha()), sd,
            static_cast<FPType>(state.mu0()),
            static_cast<FPType>(state.sd0()),
            static_cast<FPType>(state.shape0()),
            static_cast<FPType>(state.scale0()),
            state.counter());
}

template <typename FPType>
class gmm_move_mu : public vsmc::MoveCL<gmm_state<FPType> >
{
    public :

    gmm_move_mu (std::size_t local_size) :
        profiled_(false), local_size_(local_size) {}

    void move_state (std::size_t, std::string &kernel_name)
    {kernel_name = std::string("gmm_move_mu");}

    void pre_processor (std::size_t,
            vsmc::Particle<gmm_state<FPType> > &particle)
    {
        gmm_move_set_args(this, particle.value(),
                particle.value().mu_sd());

        if (!profiled_ && local_size_ == 0) {
            profiled_ = true;
            local_size_ = particle.value().manager().profile_kernel(
                    this->kernel(), particle.size());
        }

        if (local_size_ != 0)
            this->configure().local_size(local_size_);
    }

    private :

    bool profiled_;
    std::size_t local_size_;
};

template <typename FPType>
class gmm_move_lambda : public vsmc::MoveCL<gmm_state<FPType> >
{
    public :

    gmm_move_lambda (std::size_t local_size) :
        profiled_(false), local_size_(local_size) {}

    void move_state (std::size_t, std::string &kernel_name)
    {kernel_name = std::string("gmm_move_lambda");}

    void pre_processor (std::size_t,
            vsmc::Particle<gmm_state<FPType> > &particle)
    {
        gmm_move_set_args(this, particle.value(),
                particle.value().lambda_sd());

        if (!profiled_ && local_size_ == 0) {
            profiled_ = true;
            local_size_ = particle.value().manager().profile_kernel(
                    this->kernel(), particle.size());
        }

        if (local_size_ != 0)
            this->configure().local_size(local_size_);
    }

    private :

    bool profiled_;
    std::size_t local_size_;
};

template <typename FPType>
class gmm_move_weight : public vsmc::MoveCL<gmm_state<FPType> >
{
    public :

    gmm_move_weight (std::size_t local_size) :
        profiled_(false), local_size_(local_size) {}

    void move_state (std::size_t, std::string &kernel_name)
    {kernel_name = std::string("gmm_move_weight");}

    void pre_processor (std::size_t,
            vsmc::Particle<gmm_state<FPType> > &particle)
    {
        gmm_move_set_args(this, particle.value(),
                particle.value().weight_sd());

        if (!profiled_ && local_size_ == 0) {
            profiled_ = true;
            local_size_ = particle.value().manager().profile_kernel(
                    this->kernel(), particle.size());
        }

        if (local_size_ != 0)
            this->configure().local_size(local_size_);
    }

    private :

    bool profiled_;
    std::size_t local_size_;
};

template <typename FPType>
class gmm_path : public vsmc::PathEvalCL<gmm_state<FPType> >
{
    public :

    gmm_path () {}

    void path_state (std::size_t, std::string &kernel_name)
    {kernel_name = std::string("gmm_path_state");}

    double path_grid (std::size_t,
            const vsmc::Particle<gmm_state<FPType> > &particle)
    {return particle.value().alpha();}
};

template <typename FPType>
inline void gmm_do_smc_model (vsmc::Sampler<gmm_state<FPType> > &sampler,
        std::size_t iter_num, std::size_t local_size,
        std::size_t model_num, std::size_t repeat,
        const std::string &src, const std::string &opt, const data_info &info)
{
    if (model_num < 1)
        return;

    sampler.mcmc_queue_clear();
    sampler
        .init(gmm_init<FPType>())
        .move(gmm_move_smc<FPType>(iter_num), false)
        .mcmc(gmm_move_mu<FPType>(local_size), true)
        .mcmc(gmm_move_lambda<FPType>(local_size), true)
        .mcmc(gmm_move_weight<FPType>(local_size), true)
        .path_sampling(gmm_path<FPType>());
    sampler.particle().value().comp_num(model_num);
    std::stringstream ss;
    ss << "#define CompNum " << model_num << "U\n";
    ss << "#define DataNum " << info.data_num << "U\n";
    sampler.particle().value().build(ss.str() + src, opt);
    sampler.initialize(const_cast<data_info *>(&info));
    sampler.iterate();
    for (std::size_t i = 0; i != repeat; ++i) {
#ifdef VSMC_GMM_SMC_CL_MPI
        sampler.particle().value().barrier();
#endif
        sampler.initialize(const_cast<data_info *>(&info));
        vsmc::StopWatch watch;
        watch.start();
        while (sampler.particle().value().alpha() < 1)
            sampler.iterate();
        watch.stop();

        // log(2 pi) / 2 * data_num;
#ifdef VSMC_GMM_SMC_CL_MPI
        if (sampler.particle().value().world().rank() == 0) {
            double ps_sum = 0;
            double ts_max = 0;
            boost::mpi::reduce(sampler.particle().value().world(),
                    sampler.path_sampling(), ps_sum, std::plus<double>(), 0);
            boost::mpi::reduce(sampler.particle().value().world(),
                    watch.seconds(), ts_max, boost::mpi::maximum<double>(), 0);

            const double ll_const = -0.9189385332046727 * info.data_num;
            std::cout << std::string(78, '-') << std::endl;
            std::cout << "Model order\t\t\t\t\t"
                << model_num << std::endl;
            std::cout << "Log normalizing constant estimate standard\t"
                << std::fixed << sampler.particle().value().zconst() + ll_const
                << std::endl;
            std::cout << "Log normalizing constant estimate path sampling\t"
                << std::fixed << ps_sum + ll_const << std::endl;
            std::fprintf(stderr, "time.model.order.%u\t\t\t\t%f\n",
                    static_cast<unsigned>(model_num), ts_max);
            std::fflush(stderr);
            std::cout << std::string(78, '=') << std::endl;
        } else {
            boost::mpi::reduce(sampler.particle().value().world(),
                    sampler.path_sampling(), std::plus<double>(), 0);
            boost::mpi::reduce(sampler.particle().value().world(),
                    watch.seconds(), boost::mpi::maximum<double>(), 0);
        }
#else
        std::cout << std::string(78, '-') << std::endl;
        std::cout << "Model order\t\t\t\t\t"
            << model_num << std::endl;
        std::cout << "Log normalizing constant estimate standard\t"
            << std::fixed << sampler.particle().value().zconst() + ll_const
            << std::endl;
        std::cout << "Log normalizing constant estimate path sampling\t"
            << std::fixed << sampler.path_sampling() + ll_const
            << std::endl;
        std::fprintf(stderr, "time.model.order.%u\t\t\t\t%f\n",
                static_cast<unsigned>(model_num), watch.seconds());
        std::fflush(stderr);
        std::cout << std::string(78, '=') << std::endl;
#endif
    }
}

template <typename FPType>
inline void gmm_do_smc (std::size_t particle_num, std::size_t iter_num,
        std::size_t data_num, const std::string &data_file, double threshold,
        const std::string &vsmc_inc_path, const std::string &r123_inc_path,
        const std::string &build_option, std::size_t local_size,
        std::size_t sm, std::size_t cm, std::size_t repeat)
{
    if (iter_num < 1)
        return;

    vsmc::Sampler<gmm_state<FPType> > sampler(
            particle_num, vsmc::Stratified, threshold);

    std::string name;
    std::cout << std::string(78, '=') << std::endl;
    gmm_state<FPType>::manager().platform().getInfo(
            static_cast<cl_device_info>(CL_PLATFORM_NAME), &name);
    std::cout << "Using platform\t" << name << std::endl;
    gmm_state<FPType>::manager().device().getInfo(
            static_cast<cl_device_info>(CL_DEVICE_NAME), &name);
    std::cout << "Using device\t" << name << std::endl;
    std::cout << "Using OpenCL\t"
        << gmm_state<FPType>::manager().opencl_version() / 100.0 << std::endl;
    std::cout << std::string(78, '=') << std::endl;

    std::ifstream src_file("gmm_smc_cl.cl");
    std::string src((std::istreambuf_iterator<char>(src_file)),
            std::istreambuf_iterator<char>());
    src_file.close();
    src_file.clear();

    std::string opt;
    opt += " -I " + vsmc_inc_path;
    opt += " -I " + r123_inc_path;
    opt += " " + build_option;

    data_info info(data_num, data_file.c_str());
    std::ofstream save_file;

    if (sm) {
        save_file.open("smc.sampler.save.Prior2.Simple");
        gmm_do_smc_model(sampler, iter_num, local_size,
                sm, repeat, src, opt, info);
        save_file << sampler << std::endl;
        save_file.close();
        save_file.clear();
    }

    if (cm) {
        save_file.open("smc.sampler.save.Prior2.Complex");
        gmm_do_smc_model(sampler, iter_num, local_size,
                cm, repeat, src, opt, info);
        save_file << sampler << std::endl;
        save_file.close();
        save_file.clear();
    }
}

#endif // VSMC_EXAMPLE_GMM_SMC_CL_HPP
