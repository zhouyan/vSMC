//============================================================================
// vSMC/example/gmm/include/gmm.hpp
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

// clang-format off
#ifndef VSMC_EXAMPLE_GMM_@SMP@_HPP
#define VSMC_EXAMPLE_GMM_@SMP@_HPP
// clang-format on

#ifndef VSMC_GMM_MPI
#define VSMC_GMM_MPI 0
#endif

#include <vsmc/core/sampler.hpp>
#include <vsmc/core/state_matrix.hpp>
#include <vsmc/smp/backend_@smp@.hpp>
#include <vsmc/utility/program_option.hpp>
#include <vsmc/utility/stop_watch.hpp>
#ifdef VSMC_GMM_MPI
#include <vsmc/mpi/backend_mpi.hpp>
#endif

// clang-format off
template <typename T>
using StateSMP = vsmc::State@SMP@<T>;

template <typename T, typename Derived>
using InitializeSMP = vsmc::Initialize@SMP@<T, Derived>;

template <typename T, typename Derived>
using MoveSMP = vsmc::Move@SMP@<T, Derived>;

template <typename T, typename Derived>
using PathEvalSMP = vsmc::PathEval@SMP@<T, Derived>;
// clang-format on

class gmm_param
{
    public:
    gmm_param()
        : comp_num_(0)
        , log_prior_(0)
        , log_likelihood_(0)
        , log_prior_old_(0)
        , log_likelihood_old_(0)
    {
    }

    std::size_t comp_num() const { return comp_num_; }

    double mu(std::size_t c) const { return mu_[c]; }
    double lambda(std::size_t c) const { return lambda_[c]; }
    double weight(std::size_t c) const { return weight_[c]; }
    double log_lambda(std::size_t c) const { return log_lambda_[c]; }
    double log_prior() const { return log_prior_; }
    double log_likelihood() const { return log_likelihood_; }

    double &mu(std::size_t c) { return mu_[c]; }
    double &lambda(std::size_t c) { return lambda_[c]; }
    double &weight(std::size_t c) { return weight_[c]; }
    double &log_prior() { return log_prior_; }
    double &log_likelihood() { return log_likelihood_; }

    void comp_num(std::size_t num)
    {
        if (comp_num_ == num)
            return;

        comp_num_ = num;

        mu_.resize(num);
        lambda_.resize(num);
        weight_.resize(num);

        mu_old_.resize(num);
        lambda_old_.resize(num);
        weight_old_.resize(num);

        log_lambda_.resize(num);
    }

    void save_old()
    {
        log_prior_old_ = log_prior_;
        log_likelihood_old_ = log_likelihood_;

        for (std::size_t i = 0; i != mu_.size(); ++i)
            mu_old_[i] = mu_[i];
        for (std::size_t i = 0; i != lambda_.size(); ++i)
            lambda_old_[i] = lambda_[i];
        for (std::size_t i = 0; i != weight_.size(); ++i)
            weight_old_[i] = weight_[i];
    }

    void update_log_lambda()
    {
        for (std::size_t i = 0; i != comp_num_; ++i)
            log_lambda_[i] = std::log(lambda_[i]);
    }

    double log_lambda_diff() const
    {
        double sum = 0;
        for (std::size_t i = 0; i != comp_num_; ++i)
            sum += std::log(lambda_[i] / lambda_old_[i]);

        return sum;
    }

    double logit_weight_diff() const
    {
        double sum_lw = 1;
        double sum_llw = 0;
        for (std::size_t i = 0; i != comp_num_ - 1; ++i) {
            double w = weight_[i] / weight_[comp_num_ - 1];
            sum_lw += w;
            sum_llw += std::log(w);
        }
        sum_llw -= comp_num_ * std::log(sum_lw);

        double sum_lw_old = 1;
        double sum_llw_old = 0;
        for (std::size_t i = 0; i != comp_num_ - 1; ++i) {
            double w = weight_old_[i] / weight_old_[comp_num_ - 1];
            sum_lw_old += w;
            sum_llw_old += std::log(w);
        }
        sum_llw_old -= comp_num_ * std::log(sum_lw_old);

        return sum_llw - sum_llw_old;
    }

    unsigned mh_reject_mu(double p, double u)
    {
        if (p < u)
            for (std::size_t i = 0; i != mu_old_.size(); ++i)
                mu_[i] = mu_old_[i];
        return mh_reject_common(p, u);
    }

    unsigned mh_reject_lambda(double p, double u)
    {
        if (p < u)
            for (std::size_t i = 0; i != lambda_old_.size(); ++i)
                lambda_[i] = lambda_old_[i];
        return mh_reject_common(p, u);
    }

    unsigned mh_reject_weight(double p, double u)
    {
        if (p < u)
            for (std::size_t i = 0; i != weight_old_.size(); ++i)
                weight_[i] = weight_old_[i];
        return mh_reject_common(p, u);
    }

    unsigned mh_reject_common(double p, double u)
    {
        if (p < u) {
            log_prior_ = log_prior_old_;
            log_likelihood_ = log_likelihood_old_;
            return 0;
        } else {
            return 1;
        }
    }

    template <typename Archive>
    void serialize(Archive &ar, const unsigned)
    {
        int num = static_cast<int>(comp_num_);
        ar &num;
        comp_num(static_cast<std::size_t>(num));

        ar &log_prior_;
        ar &log_likelihood_;
        for (std::size_t i = 0; i != comp_num_; ++i) {
            ar &mu_[i];
            ar &lambda_[i];
            ar &weight_[i];
            ar &log_lambda_[i];
        }
    }

    private:
    std::size_t comp_num_;

    double log_prior_;
    double log_likelihood_;

    double log_prior_old_;
    double log_likelihood_old_;

    vsmc::Vector<double> mu_;
    vsmc::Vector<double> lambda_;
    vsmc::Vector<double> weight_;

    vsmc::Vector<double> mu_old_;
    vsmc::Vector<double> lambda_old_;
    vsmc::Vector<double> weight_old_;

    vsmc::Vector<double> log_lambda_;
};

#if VSMC_GMM_MPI
using StateBase =
    vsmc::StateMPI<StateSMP<vsmc::StateMatrix<vsmc::RowMajor, 1, gmm_param>>>;
#else
using StateBase = StateSMP<vsmc::StateMatrix<vsmc::RowMajor, 1, gmm_param>>;
#endif

class gmm_state : public StateBase
{
    public:
    using size_type = typename StateBase::size_type;

    gmm_state(size_type N)
        : StateBase(N)
        , comp_num_(0)
        , alpha_(0)
        , alpha_inc_(0)
        , mu0_(1)
        , sd0_(1)
        , shape0_(1)
        , scale0_(1)
        , mu_sd_(1)
        , lambda_sd_(1)
        , weight_sd_(1)
    {
    }

    std::size_t comp_num() const { return comp_num_; }

    double alpha() const { return alpha_; }
    double alpha_inc() const { return alpha_inc_; }

    double mu0() const { return mu0_; }
    double sd0() const { return sd0_; }
    double shape0() const { return shape0_; }
    double scale0() const { return scale0_; }

    double mu_sd() const { return mu_sd_; }
    double lambda_sd() const { return lambda_sd_; }
    double weight_sd() const { return weight_sd_; }

    void mu_sd(double sd)
    {
        assert(sd > 0);
        mu_sd_ = sd;
    }
    void lambda_sd(double sd)
    {
        assert(sd > 0);
        lambda_sd_ = sd;
    }
    void weight_sd(double sd)
    {
        assert(sd > 0);
        weight_sd_ = sd;
    }

    void alpha(double a)
    {
        a = a < 1 ? a : 1;
        a = a > 0 ? a : 0;
        if (a > 0) {
            alpha_inc_ = a - alpha_;
            alpha_ = a;
        } else {
            alpha_inc_ = 0;
            alpha_ = 0;
        }
    }

    void comp_num(std::size_t num)
    {
        comp_num_ = num;
        for (size_type i = 0; i != this->size(); ++i)
            this->state(i, 0).comp_num(num);
    }

    double update_log_prior(gmm_param &param) const
    {
        double lp = 0;
        for (std::size_t i = 0; i != comp_num_; ++i) {
            double resid = param.mu(i) - mu0_;
            lp += -0.5 * (resid * resid) / (sd0_ * sd0_);
            lp += (shape0_ - 1) * std::log(param.lambda(i)) -
                param.lambda(i) / scale0_;
        }

        return param.log_prior() = lp;
    }

    double update_log_likelihood(gmm_param &param) const
    {
        double ll = -0.5 * obs_.size() * vsmc::math::ln_pi_2<double>();
        param.update_log_lambda();
        for (std::size_t k = 0; k != obs_.size(); ++k) {
            double lli = 0;
            for (std::size_t i = 0; i != param.comp_num(); ++i) {
                double resid = obs_[k] - param.mu(i);
                lli += param.weight(i) *
                    std::exp(0.5 * param.log_lambda(i) -
                           0.5 * param.lambda(i) * resid * resid);
            }
            ll += std::log(lli + 1e-13); // lli can be numerically zero!
        }

        return param.log_likelihood() = ll;
    }

    void read_data(const char *filename)
    {
        obs_.clear();
        std::ifstream data(filename);
        double tmp;
        while (data >> tmp)
            obs_.push_back(tmp);
        data.close();
        data.clear();

        double xmax = obs_[0];
        double xmin = obs_[0];
        for (std::size_t i = 0; i != obs_.size(); ++i) {
            if (obs_[i] > xmax)
                xmax = obs_[i];
            if (obs_[i] < xmin)
                xmin = obs_[i];
        }
        double kappa = 1 / ((xmax - xmin) * (xmax - xmin));
        mu0_ = 0.5 * (xmin + xmax);
        sd0_ = 1 / std::sqrt(kappa);
        shape0_ = 2;
        scale0_ = 50 * kappa;
    }

    private:
    std::size_t comp_num_;

    double alpha_;
    double alpha_inc_;

    double mu0_;
    double sd0_;
    double shape0_;
    double scale0_;

    double mu_sd_;
    double lambda_sd_;
    double weight_sd_;

    vsmc::Vector<double> obs_;
};

class gmm_init : public InitializeSMP<gmm_state, gmm_init>
{
    public:
    std::size_t eval_sp(vsmc::SingleParticle<gmm_state> sp)
    {
        const gmm_state &state = sp.particle().value();
        gmm_param &param = sp.state(0);

        std::normal_distribution<> rmu(state.mu0(), state.sd0());
        std::gamma_distribution<> rlambda(state.shape0(), state.scale0());
        std::gamma_distribution<> rweight(1, 1);

        double sum = 0;
        for (std::size_t i = 0; i != param.comp_num(); ++i) {
            param.mu(i) = rmu(sp.rng());
            param.lambda(i) = rlambda(sp.rng());
            param.weight(i) = rweight(sp.rng());
            sum += param.weight(i);
        }
        for (std::size_t i = 0; i != param.comp_num(); ++i)
            param.weight(i) /= sum;

        state.update_log_prior(param);
        state.update_log_likelihood(param);

        return 1;
    }

    void eval_param(vsmc::Particle<gmm_state> &particle, void *filename)
    {
        if (filename)
            particle.value().read_data(static_cast<const char *>(filename));
        particle.value().alpha(0);
        particle.weight().set_equal();
    }
};

class gmm_move_smc
{
    public:
    typedef std::function<void(std::size_t, vsmc::Particle<gmm_state> &)>
        alpha_setter_type;

    gmm_move_smc(const alpha_setter_type &alpha_setter)
        : alpha_setter_(alpha_setter)
    {
    }

    std::size_t operator()(
        std::size_t iter, vsmc::Particle<gmm_state> &particle)
    {
        alpha_setter_(iter, particle);

        double alpha = particle.value().alpha();
        alpha = alpha < 0.02 ? 0.02 : alpha;
        particle.value().mu_sd(0.15 / alpha);
        particle.value().lambda_sd((1 + std::sqrt(1 / alpha)) * 0.15);
        particle.value().weight_sd((1 + std::sqrt(1 / alpha)) * 0.2);

        w_.resize(particle.size());
        double coeff = particle.value().alpha_inc();
        for (std::size_t i = 0; i != particle.size(); ++i)
            w_[i] = coeff * particle.value().state(i, 0).log_likelihood();
        particle.weight().add_log(w_.data());

        return 0;
    }

    private:
    alpha_setter_type alpha_setter_;
    vsmc::Vector<double> w_;
};

class gmm_move_mu : public MoveSMP<gmm_state, gmm_move_mu>
{
    public:
    std::size_t eval_sp(std::size_t, vsmc::SingleParticle<gmm_state> sp)
    {
        const gmm_state &state = sp.particle().value();
        gmm_param &param = sp.state(0);

        std::normal_distribution<> rmu(0, state.mu_sd());
        std::uniform_real_distribution<> runif(0, 1);

        double p = param.log_prior() + state.alpha() * param.log_likelihood();
        param.save_old();
        for (std::size_t i = 0; i != param.comp_num(); ++i)
            param.mu(i) += rmu(sp.rng());
        p = state.update_log_prior(param) +
            state.alpha() * state.update_log_likelihood(param) - p;
        double u = std::log(runif(sp.rng()));

        return param.mh_reject_mu(p, u);
    }
};

class gmm_move_lambda : public MoveSMP<gmm_state, gmm_move_lambda>
{
    public:
    std::size_t eval_sp(std::size_t, vsmc::SingleParticle<gmm_state> sp)
    {
        const gmm_state &state = sp.particle().value();
        gmm_param &param = sp.state(0);

        std::lognormal_distribution<> rlambda(0, state.lambda_sd());
        std::uniform_real_distribution<> runif(0, 1);

        double p = param.log_prior() + state.alpha() * param.log_likelihood();
        param.save_old();
        for (std::size_t i = 0; i != param.comp_num(); ++i)
            param.lambda(i) *= rlambda(sp.rng());
        p = param.log_lambda_diff() + state.update_log_prior(param) +
            state.alpha() * state.update_log_likelihood(param) - p;
        double u = std::log(runif(sp.rng()));

        return param.mh_reject_lambda(p, u);
    }
};

class gmm_move_weight : public MoveSMP<gmm_state, gmm_move_weight>
{
    public:
    std::size_t eval_sp(std::size_t, vsmc::SingleParticle<gmm_state> sp)
    {
        const gmm_state &state = sp.particle().value();
        gmm_param &param = sp.state(0);

        std::normal_distribution<> rweight(0, state.weight_sd());
        std::uniform_real_distribution<> runif(0, 1);

        double p = param.log_prior() + state.alpha() * param.log_likelihood();
        param.save_old();
        double sum = 1;
        for (std::size_t i = 0; i != param.comp_num() - 1; ++i) {
            param.weight(i) =
                std::log(param.weight(i) / param.weight(param.comp_num() - 1));
            param.weight(i) += rweight(sp.rng());
            param.weight(i) = std::exp(param.weight(i));
            sum += param.weight(i);
        }
        param.weight(param.comp_num() - 1) = 1;
        for (std::size_t i = 0; i != param.comp_num(); ++i)
            param.weight(i) /= sum;
        p = param.logit_weight_diff() + state.update_log_prior(param) +
            state.alpha() * state.update_log_likelihood(param) - p;
        double u = std::log(runif(sp.rng()));

        return param.mh_reject_weight(p, u);
    }
};

class gmm_path : public PathEvalSMP<gmm_state, gmm_path>
{
    public:
    double eval_sp(std::size_t, vsmc::SingleParticle<gmm_state> sp)
    {
        return sp.state(0).log_likelihood();
    }

    double eval_grid(std::size_t, vsmc::Particle<gmm_state> &particle)
    {
        return particle.value().alpha();
    }
};

class gmm_alpha_linear
{
    public:
    gmm_alpha_linear(const std::size_t iter_num) : iter_num_(iter_num) {}

    void operator()(
        std::size_t iter, vsmc::Particle<gmm_state> &particle) const
    {
        particle.value().alpha(static_cast<double>(iter) / iter_num_);
    }

    private:
    std::size_t iter_num_;
};

class gmm_alpha_prior
{
    public:
    gmm_alpha_prior(std::size_t iter_num, std::size_t power)
        : iter_num_(iter_num), power_(power)
    {
    }

    void operator()(
        std::size_t iter, vsmc::Particle<gmm_state> &particle) const
    {
        double base = static_cast<double>(iter) / iter_num_;
        double alpha = 1;
        for (std::size_t p = 0; p != power_; ++p)
            alpha *= base;
        particle.value().alpha(alpha);
    }

    private:
    std::size_t iter_num_;
    std::size_t power_;
};

inline int gmm_main(int argc, char **argv)
{
#if VSMC_GMM_MPI
    vsmc::MPIEnvironment env(argc, argv);
    boost::mpi::communicator world;
    const bool master = (world.rank() == 0);
    vsmc::Seed::instance().modulo(
        static_cast<vsmc::Seed::skip_type>(world.size()),
        static_cast<vsmc::Seed::skip_type>(world.rank()));
#else
    const bool master = true;
#endif

    std::size_t N = 1000;
    std::size_t n = 100;
    std::size_t c = 4;
    std::size_t power = 2;
    std::string datafile("gmm.data");

    if (master) {
        vsmc::ProgramOptionMap option;
        option.add("N", "Number of particles", &N, 1000)
            .add("n", "Number of iterations", &n, 100)
            .add("c", "Number of components", &c, 4)
            .add("power", "Power of the prior annealing (0 for linear)",
                &power, 2)
            .add("datafile", "File name of the data", &datafile, "gmm.data");
        option.process(argc, argv);
#if VSMC_GMM_MPI
        boost::mpi::broadcast(world, N, 0);
        boost::mpi::broadcast(world, n, 0);
        boost::mpi::broadcast(world, c, 0);
        boost::mpi::broadcast(world, power, 0);
        boost::mpi::broadcast(world, datafile, 0);
        N = N / static_cast<std::size_t>(world.size());
#endif
    }

    gmm_move_smc::alpha_setter_type alpha_setter;
    if (power == 0)
        alpha_setter = gmm_alpha_linear(n);
    else
        alpha_setter = gmm_alpha_prior(n, power);

    vsmc::Seed::instance().set(101);
    vsmc::Sampler<gmm_state> sampler(N, vsmc::Stratified, 0.5);
    sampler.particle().value().comp_num(c);
    sampler.init(gmm_init())
        .move(gmm_move_smc(alpha_setter), false)
        .mcmc(gmm_move_mu(), false)
        .mcmc(gmm_move_lambda(), true)
        .mcmc(gmm_move_weight(), true)
        .path_sampling(gmm_path());

    vsmc::StopWatch watch;
    watch.start();
    sampler.initialize(const_cast<char *>(datafile.c_str())).iterate(n);
    double ps = sampler.path().log_zconst();
#if VSMC_GMM_MPI
    double ps_sum = 0;
    boost::mpi::reduce(world, ps, ps_sum, std::plus<double>(), 0);
    ps = ps_sum;
#endif
    watch.stop();

    if (master) {
        std::cout << "Path sampling estimate: " << ps << std::endl;
        std::cout << "Wallclock Time: " << watch.seconds() << "s" << std::endl;
    }

    return 0;
}

#endif // VSMC_EXAMPLE_GMM_@SMP@_HPP
