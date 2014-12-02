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

struct data_info
{
    const std::size_t data_num;
    const char *file_name;

    data_info (std::size_t num, const char *file) :
        data_num(num), file_name(file) {}
};

struct gmm_device;

template <typename FPType>
class gmm_state : public vsmc::StateCL<vsmc::Dynamic, FPType, gmm_device>
{
    public :

    typedef vsmc::StateCL<vsmc::Dynamic, FPType, gmm_device> base;
    typedef typename base::size_type size_type;
    typedef typename base::fp_type fp_type;

    gmm_state (size_type N) : base(N)
    {
        counter_ = this->manager().template
            create_buffer<struct r123array4x32>(N);
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
        this->resize_state((num * 3 + 3) * sizeof(fp_type));
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
        obs_ = this->manager().template
            create_buffer<fp_type>(obs.begin(), obs.end());
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

        lambda_init_ = state.manager().template
            create_buffer<FPType>(lambda_init.begin(), lambda_init.end());
        weight_init_ = state.manager().template
            create_buffer<FPType>(weight_init.begin(), weight_init.end());

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
        if (weight_.size() != state.size()) {
            inc_weight_device_ = state.manager().template
                create_buffer<FPType>(state.size());
            weight_.resize(state.size());
            inc_weight_.resize(state.size());
            exp_weight_.resize(state.size());
        }

        FPType a = static_cast<FPType>(iter) / iter_num_;
        a = a * a;
        state.alpha(a);

        a = a < 0.05f ? 0.05f : a;
        state.mu_sd()     = 0.15f / a;
        state.lambda_sd() = (1 + sqrt(1 / a)) * 0.15f;
        state.weight_sd() = (1 + sqrt(1 / a)) * 0.20f;

        vsmc::cl_set_kernel_args(this->kernel(),
                this->kernel_args_offset(), inc_weight_device_,
                static_cast<FPType>(state.alpha_inc()));
    }

    void post_processor (std::size_t,
            vsmc::Particle<gmm_state<FPType> > &particle)
    {
        using std::exp;

        gmm_state<FPType> &state = particle.value();
        state.manager().template read_buffer<FPType>(
                inc_weight_device_, state.size(), &inc_weight_[0]);

        for (typename vsmc::Particle<gmm_state<FPType> >::size_type i = 0;
                i != state.size(); ++i) {
            exp_weight_[i] = exp(inc_weight_[i]);
        }

        particle.weight_set().read_weight(&weight_[0]);
        double sum = 0;
        for (typename vsmc::Particle<gmm_state<FPType> >::size_type i = 0;
                i != state.size(); ++i) {
            sum += weight_[i] * exp_weight_[i];
        }

        state.zconst() += log(sum);
        particle.weight_set().add_log_weight(&inc_weight_[0]);
    }

    private :

    std::size_t iter_num_;
    cl::Buffer inc_weight_device_;
    std::vector<double> weight_;
    std::vector<double> inc_weight_;
    std::vector<double> exp_weight_;
};

template <typename Move, typename FPType>
class gmm_move_set_args
{
    public :

    gmm_move_set_args (std::size_t iter,
            const vsmc::Particle<gmm_state<FPType> > &particle,
            Move &move, FPType sd) :
    iter_(iter), particle_(particle), move_(move), sd_(sd) {}

    void operator() (::cl::Kernel &) const
    {
        move_.set_kernel_args(iter_, particle_);
        vsmc::cl_set_kernel_args(move_.kernel(), move_.kernel_args_offset(),
                particle_.value().obs(),
                static_cast<FPType>(particle_.value().alpha()), sd_,
                static_cast<FPType>(particle_.value().mu0()),
                static_cast<FPType>(particle_.value().sd0()),
                static_cast<FPType>(particle_.value().shape0()),
                static_cast<FPType>(particle_.value().scale0()),
                particle_.value().counter());
    }

    private :

    std::size_t iter_;
    const vsmc::Particle<gmm_state<FPType> > &particle_;
    Move &move_;
    FPType sd_;
};

template <typename FPType>
class gmm_move_mu : public vsmc::MoveCL<gmm_state<FPType> >
{
    public :

    gmm_move_mu (std::size_t local_size) :
        profiled_(false), local_size_(local_size) {}

    void move_state (std::size_t, std::string &kernel_name)
    {kernel_name = std::string("gmm_move_mu");}

    void pre_processor (std::size_t iter,
            vsmc::Particle<gmm_state<FPType> > &particle)
    {
        gmm_move_set_args<gmm_move_mu<FPType>, FPType> set_args(
                iter, particle, *this, particle.value().mu_sd());

        if (!profiled_ && local_size_ == 0) {
            profiled_ = true;
            local_size_ = particle.value().manager().profile_kernel(
                    this->kernel(), particle.size(), set_args, 10);
            std::cout << "Profiled local size of kernel gmm_move_mu\t"
                << local_size_ << std::endl;
        }

        if (local_size_ != 0)
            this->configure().local_size(local_size_);

        set_args(this->kernel());
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

    void pre_processor (std::size_t iter,
            vsmc::Particle<gmm_state<FPType> > &particle)
    {
        gmm_move_set_args<gmm_move_lambda<FPType>, FPType> set_args(
                iter, particle, *this, particle.value().lambda_sd());

        if (!profiled_ && local_size_ == 0) {
            profiled_ = true;
            local_size_ = particle.value().manager().profile_kernel(
                    this->kernel(), particle.size(), set_args, 10);
            std::cout << "Profiled local size of kernel gmm_move_lambda\t"
                << local_size_ << std::endl;
        }

        if (local_size_ != 0)
            this->configure().local_size(local_size_);

        set_args(this->kernel());
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

    void pre_processor (std::size_t iter,
            vsmc::Particle<gmm_state<FPType> > &particle)
    {
        gmm_move_set_args<gmm_move_weight<FPType>, FPType> set_args(
                iter, particle, *this, particle.value().weight_sd());

        if (!profiled_ && local_size_ == 0) {
            profiled_ = true;
            local_size_ = particle.value().manager().profile_kernel(
                    this->kernel(), particle.size(), set_args, 10);
            std::cout << "Profiled local size of kernel gmm_move_weight\t"
                << local_size_ << std::endl;
        }

        if (local_size_ != 0)
            this->configure().local_size(local_size_);

        set_args(this->kernel());
    }

    private :

    bool profiled_;
    std::size_t local_size_;
};

template <typename FPType>
class gmm_path : public vsmc::PathEvalCL<gmm_state<FPType> >
{
    public :

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
        sampler.initialize(const_cast<data_info *>(&info));
        vsmc::StopWatch watch;
        watch.start();
        while (sampler.particle().value().alpha() < 1)
            sampler.iterate();
        watch.stop();

        cl::Kernel kern_size =
            sampler.particle().value().create_kernel("query_size");
        cl::Buffer buff_size =
            sampler.particle().value().manager().template
            create_buffer<cl_uint>(3);
        kern_size.setArg(0, buff_size);
        sampler.particle().value().manager().run_kernel(kern_size, 1, 0);
        std::vector<cl_uint> ans(3);
        sampler.particle().value().manager().template read_buffer<cl_uint>(
                buff_size, 3, ans.begin());

        // log(2 pi) / 2 * data_num;
        const double ll_const = -0.9189385332046727 * info.data_num;
        std::cout << std::string(78, '=') << std::endl;
        std::cout << "Model order\t\t\t\t\t\t"
            << model_num << std::endl;
        std::cout << "Log normalizing constant estimate standard\t\t"
            << std::fixed << sampler.particle().value().zconst() + ll_const
            << std::endl;
        std::cout << "Log normalizing constant estimate path sampling\t\t"
            << std::fixed << sampler.path_sampling() + ll_const << std::endl;
        std::cout << std::string(78, '-') << std::endl;
        std::cout << "Device size of gmm_param\t\t\t\t"
            << ans[0] << std::endl;
        std::cout << "Device size of an array of gmm_param of size "
            << ans[1] << "\t\t" << ans[2] << std::endl;
        std::cout << "Host size allocated for each gmm_param\t\t\t"
            << sampler.particle().value().state_size() << std::endl;
        std::cout << std::string(78, '-') << std::endl;
        vsmc::CLQuery::info(std::cout, sampler.particle().value().program(),
                "gmm_move_mu");
        std::cout << std::string(78, '-') << std::endl;
        vsmc::CLQuery::info(std::cout, sampler.particle().value().program(),
                "gmm_move_lambda");
        std::cout << std::string(78, '-') << std::endl;
        vsmc::CLQuery::info(std::cout, sampler.particle().value().program(),
                "gmm_move_weight");
        std::cout << std::string(78, '-') << std::endl;
        std::fprintf(stderr, "time.model.order.%u\t\t\t\t\t%f\n",
                static_cast<unsigned>(model_num), watch.seconds());
        std::cout << std::string(78, '=') << std::endl;
    }
}

template <typename FPType>
inline void gmm_do_smc (std::size_t particle_num, std::size_t iter_num,
        std::size_t data_num, const std::string &data_file, double threshold,
        const std::string &vsmc_inc_path, const std::string &r123_inc_path,
        std::size_t local_size,
        std::size_t sm, std::size_t cm, std::size_t repeat)
{
    if (iter_num < 1)
        return;

    vsmc::Sampler<gmm_state<FPType> > sampler(
            particle_num, vsmc::Stratified, threshold);

    std::string name;
    gmm_state<FPType>::manager_type::instance().platform().getInfo(
            static_cast<cl_device_info>(CL_PLATFORM_NAME), &name);
    std::cout << "Using platform\t" << name << std::endl;
    gmm_state<FPType>::manager_type::instance().device().getInfo(
            static_cast<cl_device_info>(CL_DEVICE_NAME), &name);
    std::cout << "Using device\t" << name << std::endl;

    std::ifstream src_file("gmm_smc_cl.cl");
    std::string src((std::istreambuf_iterator<char>(src_file)),
            std::istreambuf_iterator<char>());
    src_file.close();
    src_file.clear();

    std::string opt;
    opt += " -I " + vsmc_inc_path;
    opt += " -I " + r123_inc_path;

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
