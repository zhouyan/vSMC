//============================================================================
// vSMC/example/gmm/include/gmm.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2016, Yan Zhou
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

#ifndef VSMC_EXAMPLE_GMM_HPP
#define VSMC_EXAMPLE_GMM_HPP

#include <vsmc/vsmc.hpp>

template <typename>
std::string gmm_backend_name();

template <>
std::string gmm_backend_name<vsmc::BackendSEQ>()
{
    return "BackendSEQ";
}

template <>
std::string gmm_backend_name<vsmc::BackendSTD>()
{
    return "BackendSTD";
}

#if VSMC_HAS_OMP
template <>
std::string gmm_backend_name<vsmc::BackendOMP>()
{
    return "BackendOMP";
}
#endif

#if VSMC_HAS_TBB
template <>
std::string gmm_backend_name<vsmc::BackendTBB>()
{
    return "BackendTBB";
}
#endif

class GMMState
{
    public:
    GMMState()
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
}; // class GMM

using GMMBase = vsmc::StateMatrix<vsmc::RowMajor, 1, GMMState>;

class GMM : public GMMBase
{
    public:
    GMM(std::size_t N)
        : GMMBase(N)
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
        for (std::size_t i = 0; i != this->size(); ++i)
            this->operator()(i, 0).comp_num(num);
    }

    double update_log_prior(GMMState &state) const
    {
        double lp = 0;
        for (std::size_t i = 0; i != comp_num_; ++i) {
            double resid = state.mu(i) - mu0_;
            lp += -0.5 * (resid * resid) / (sd0_ * sd0_);
            lp += (shape0_ - 1) * std::log(state.lambda(i)) -
                state.lambda(i) / scale0_;
        }

        return state.log_prior() = lp;
    }

    double update_log_likelihood(GMMState &state) const
    {
        double ll = -0.5 * obs_.size() * vsmc::const_ln_pi_2<double>();
        state.update_log_lambda();
        for (std::size_t k = 0; k != obs_.size(); ++k) {
            double lli = 0;
            for (std::size_t i = 0; i != state.comp_num(); ++i) {
                double resid = obs_[k] - state.mu(i);
                lli += state.weight(i) *
                    std::exp(0.5 * state.log_lambda(i) -
                           0.5 * state.lambda(i) * resid * resid);
            }
            ll += std::log(lli + 1e-13); // lli can be numerically zero!
        }

        return state.log_likelihood() = ll;
    }

    void initialize()
    {
        obs_.clear();
        std::ifstream data("gmm.data");
        double tmp;
        while (data >> tmp)
            obs_.push_back(tmp);
        data.close();

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
}; // class GMM

template <typename Backend>
class GMMInit : public vsmc::SamplerEvalSMP<GMM, GMMInit<Backend>, Backend>
{
    public:
    void eval_pre(std::size_t, vsmc::Particle<GMM> &particle)
    {
        particle.state().initialize();
        particle.state().alpha(0);
        particle.weight().set_equal();
    }

    std::size_t eval_sp(std::size_t, vsmc::SingleParticle<GMM> sp)
    {
        const GMM &gmm = sp.particle().state();
        GMMState &state = sp(0);

        std::normal_distribution<double> rmu(gmm.mu0(), gmm.sd0());
        std::gamma_distribution<double> rlambda(gmm.shape0(), gmm.scale0());
        std::gamma_distribution<double> rweight(1, 1);

        double sum = 0;
        for (std::size_t i = 0; i != state.comp_num(); ++i) {
            state.mu(i) = rmu(sp.rng());
            state.lambda(i) = rlambda(sp.rng());
            state.weight(i) = rweight(sp.rng());
            sum += state.weight(i);
        }
        for (std::size_t i = 0; i != state.comp_num(); ++i)
            state.weight(i) /= sum;

        gmm.update_log_prior(state);
        gmm.update_log_likelihood(state);

        return 0;
    }
}; // class GMMInit

class GMMMoveSMC
{
    public:
    typedef std::function<void(std::size_t, vsmc::Particle<GMM> &)>
        alpha_setter_type;

    GMMMoveSMC(const alpha_setter_type &alpha_setter)
        : alpha_setter_(alpha_setter)
    {
    }

    std::size_t operator()(std::size_t iter, vsmc::Particle<GMM> &particle)
    {
        alpha_setter_(iter, particle);

        double alpha = particle.state().alpha();
        alpha = alpha < 0.02 ? 0.02 : alpha;
        particle.state().mu_sd(0.15 / alpha);
        particle.state().lambda_sd((1 + std::sqrt(1 / alpha)) * 0.15);
        particle.state().weight_sd((1 + std::sqrt(1 / alpha)) * 0.2);

        w_.resize(particle.size());
        double coeff = particle.state().alpha_inc();
        for (std::size_t i = 0; i != particle.size(); ++i)
            w_[i] = coeff * particle.state()(i, 0).log_likelihood();
        particle.weight().add_log(w_.data());

        return 0;
    }

    private:
    alpha_setter_type alpha_setter_;
    vsmc::Vector<double> w_;
}; // class GMMMoveSMC

template <typename Backend>
class GMMMoveMu : public vsmc::SamplerEvalSMP<GMM, GMMMoveMu<Backend>, Backend>
{
    public:
    std::size_t eval_sp(std::size_t, vsmc::SingleParticle<GMM> sp)
    {
        const GMM &gmm = sp.particle().state();
        GMMState &state = sp(0);

        std::normal_distribution<double> rmu(0, gmm.mu_sd());
        std::uniform_real_distribution<double> runif(0, 1);

        double p = state.log_prior() + gmm.alpha() * state.log_likelihood();
        state.save_old();
        for (std::size_t i = 0; i != state.comp_num(); ++i)
            state.mu(i) += rmu(sp.rng());
        p = gmm.update_log_prior(state) +
            gmm.alpha() * gmm.update_log_likelihood(state) - p;
        double u = std::log(runif(sp.rng()));

        return state.mh_reject_mu(p, u);
    }
}; // class GMMMoveMu

template <typename Backend>
class GMMMoveLambda
    : public vsmc::SamplerEvalSMP<GMM, GMMMoveLambda<Backend>, Backend>
{
    public:
    std::size_t eval_sp(std::size_t, vsmc::SingleParticle<GMM> sp)
    {
        const GMM &gmm = sp.particle().state();
        GMMState &state = sp(0);

        std::lognormal_distribution<double> rlambda(0, gmm.lambda_sd());
        std::uniform_real_distribution<double> runif(0, 1);

        double p = state.log_prior() + gmm.alpha() * state.log_likelihood();
        state.save_old();
        for (std::size_t i = 0; i != state.comp_num(); ++i)
            state.lambda(i) *= rlambda(sp.rng());
        p = state.log_lambda_diff() + gmm.update_log_prior(state) +
            gmm.alpha() * gmm.update_log_likelihood(state) - p;
        double u = std::log(runif(sp.rng()));

        return state.mh_reject_lambda(p, u);
    }
}; // class GMMMoveLambda

template <typename Backend>
class GMMMoveWeight
    : public vsmc::SamplerEvalSMP<GMM, GMMMoveWeight<Backend>, Backend>
{
    public:
    std::size_t eval_sp(std::size_t, vsmc::SingleParticle<GMM> sp)
    {
        const GMM &gmm = sp.particle().state();
        GMMState &state = sp(0);

        std::normal_distribution<double> rweight(0, gmm.weight_sd());
        std::uniform_real_distribution<double> runif(0, 1);

        double p = state.log_prior() + gmm.alpha() * state.log_likelihood();
        state.save_old();
        double sum = 1;
        for (std::size_t i = 0; i != state.comp_num() - 1; ++i) {
            state.weight(i) =
                std::log(state.weight(i) / state.weight(state.comp_num() - 1));
            state.weight(i) += rweight(sp.rng());
            state.weight(i) = std::exp(state.weight(i));
            sum += state.weight(i);
        }
        state.weight(state.comp_num() - 1) = 1;
        for (std::size_t i = 0; i != state.comp_num(); ++i)
            state.weight(i) /= sum;
        p = state.logit_weight_diff() + gmm.update_log_prior(state) +
            gmm.alpha() * gmm.update_log_likelihood(state) - p;
        double u = std::log(runif(sp.rng()));

        return state.mh_reject_weight(p, u);
    }
}; // class GMMMoveWeight

template <typename Backend>
class GMMPathIntegrand
    : public vsmc::MonitorEvalSMP<GMM, GMMPathIntegrand<Backend>, Backend>
{
    public:
    void eval_sp(
        std::size_t, std::size_t, vsmc::SingleParticle<GMM> sp, double *res)
    {
        *res = sp(0).log_likelihood();
    }
}; // class GMMPathIntegrand

class GMMPathGrid
{
    public:
    void operator()(
        std::size_t, std::size_t, vsmc::Particle<GMM> &particle, double *res)
    {
        *res = particle.state().alpha();
    }
}; // class GMMPathGrid

class GMMAlphaLinear
{
    public:
    GMMAlphaLinear(const std::size_t iter_num) : iter_num_(iter_num) {}

    void operator()(std::size_t iter, vsmc::Particle<GMM> &particle) const
    {
        particle.state().alpha(static_cast<double>(iter) / iter_num_);
    }

    private:
    std::size_t iter_num_;
}; // GMMAlphaLinear

class GMMAlphaPrior
{
    public:
    GMMAlphaPrior(std::size_t iter_num, std::size_t power)
        : iter_num_(iter_num), power_(power)
    {
    }

    void operator()(std::size_t iter, vsmc::Particle<GMM> &particle) const
    {
        double base = static_cast<double>(iter) / iter_num_;
        double alpha = 1;
        for (std::size_t p = 0; p != power_; ++p)
            alpha *= base;
        particle.state().alpha(alpha);
    }

    private:
    std::size_t iter_num_;
    std::size_t power_;
}; // class GMMAlphaPrior

#endif // VSMC_EXAMPLE_GMM_HPP
