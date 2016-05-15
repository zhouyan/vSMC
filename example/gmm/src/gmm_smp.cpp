//============================================================================
// vSMC/example/gmm/src/gmm_smp.cpp
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

#include <vsmc/vsmc.hpp>

using namespace vsmc;

using GMMBase = StateMatrix<RowMajor, Dynamic, double>;

template <typename T>
using GMMIndexBase = GMMBase::particle_index_type<T>;

class GMM : public GMMBase
{
    public:
    template <typename T>
    class particle_index_type : public GMMIndexBase<T>
    {
        public:
        particle_index_type(std::size_t id, Particle<T> *pptr)
            : GMMIndexBase<T>(id, pptr)
        {
        }

        std::size_t c() const { return this->particle().state().c(); }
        double *mu() const { return &this->at(0); }
        double *lambda() const { return &this->at(c()); }
        double *omega() const { return &this->at(c() * 2); }
        double &log_prior() const { return this->at(c() * 3); }
        double &log_likelihood() const { return this->at(c() * 3 + 1); }
    }; // class particle_index_type

    GMM(std::size_t N, std::size_t c) : GMMBase(N, c * 3 + 2), c_(c)
    {
        std::ifstream data("gmm.data");
        double y;
        while (data >> y)
            obs_.push_back(y);
        data.close();

        double ymin = *std::min_element(obs_.begin(), obs_.end());
        double ymax = *std::max_element(obs_.begin(), obs_.end());
        double kappa = 1 / ((ymax - ymin) * (ymax - ymin));
        mu0_ = 0.5 * (ymin + ymax);
        sd0_ = 1 / std::sqrt(kappa);
        shape0_ = 2;
        scale0_ = 50 * kappa;
    }

    double log_prior(const double *mu, const double *lambda)
    {
        auto lp = 0.0;
        for (std::size_t i = 0; i != c_; ++i) {
            auto r = (mu[i] - mu0_) / sd0_;
            lp += -0.5 * r * r;
            lp += (shape0_ - 1) * std::log(lambda[i]);
            lp += -lambda[i] / scale0_;
        }

        return lp;
    }

    double log_likelihood(
        const double *mu, const double *lambda, const double *omega)
    {
        auto n = obs_.size();

        Vector<double> log_lambda(c_);
        log(c_, lambda, log_lambda.data());

        auto ll = -0.5 * n * const_ln_pi_2<double>();
        for (std::size_t k = 0; k != n; ++k) {
            auto llc = 0.0;
            for (std::size_t i = 0; i != c_; ++i) {
                auto r = obs_[k] - mu[i];
                llc += omega[i] *
                    std::exp(0.5 * (log_lambda[i] - lambda[i] * r * r));
            }
            ll += std::log(llc + 1e-13);
        }

        return ll;
    }

    std::size_t c() const { return c_; }

    double alpha() const { return alpha_; }

    double alpha_inc() const { return alpha_inc_; }

    void alpha(double a)
    {
        alpha_inc_ = a - alpha_;
        alpha_ = a;

        auto scale = std::max(0.02, a);
        sd_mu_ = 0.15 / scale;
        sd_lambda_ = (1 + std::sqrt(1 / scale)) * 0.15;
        sd_omega_ = (1 + std::sqrt(1 / scale)) * 0.2;
    }

    double mu0() const { return mu0_; }
    double sd0() const { return sd0_; }
    double shape0() const { return shape0_; }
    double scale0() const { return scale0_; }

    double sd_mu() const { return sd_mu_; }
    double sd_lambda() const { return sd_lambda_; }
    double sd_omega() const { return sd_omega_; }

    private:
    std::size_t c_;
    double alpha_ = 0;
    double alpha_inc_ = 0;
    double mu0_ = 0;
    double sd0_ = 1;
    double shape0_ = 1;
    double scale0_ = 1;
    double sd_mu_ = 1;
    double sd_lambda_ = 1;
    double sd_omega_ = 1;
    Vector<double> obs_;
}; // class GMM

template <typename Derived>
class GMMEvalSMP : public SamplerEvalSMP<GMM, Derived>
{
    public:
    void operator()(std::size_t iter, Particle<GMM> &particle)
    {
        this->run(iter, particle, 64);
    }
}; // class GMMEvalSMP

class GMMInit : public GMMEvalSMP<GMMInit>
{
    public:
    void eval_range(std::size_t, const ParticleRange<GMM> &range)
    {
        auto c = range.particle().state().c();
        auto &rng = range.begin().rng();

        NormalDistribution<> rmu(
            range.particle().state().mu0(), range.particle().state().sd0());
        GammaDistribution<> rlambda(range.particle().state().shape0(),
            range.particle().state().scale0());
        DirichletDistribution<> romega(c, 1);

        for (auto idx : range) {
            rmu(rng, c, idx.mu());
            rlambda(rng, c, idx.lambda());
            romega(rng, idx.omega());
            idx.log_prior() =
                idx.particle().state().log_prior(idx.mu(), idx.lambda());
            idx.log_likelihood() = idx.particle().state().log_likelihood(
                idx.mu(), idx.lambda(), idx.omega());
        }
    }

    void eval_pre(std::size_t, Particle<GMM> &particle)
    {
        particle.state().alpha(0);
    }
}; // class GMMInit

class GMMMove
{
    public:
    GMMMove(double T) : T_(T) {}

    std::size_t operator()(std::size_t iter, Particle<GMM> &particle)
    {
        auto alpha = iter / T_;
        alpha = std::min(1.0, alpha * alpha);
        particle.state().alpha(alpha);

        w_.resize(particle.size());
        std::transform(particle.begin(), particle.end(), w_.begin(),
            [](ParticleIndex<GMM> idx) { return idx.log_likelihood(); });
        mul(w_.size(), particle.state().alpha_inc(), w_.data(), w_.data());
        particle.weight().add_log(w_.data());

        return 0;
    }

    private:
    double T_;
    Vector<double> w_;
}; // class GMMMove

class GMMMu : public GMMEvalSMP<GMMMu>
{
    public:
    void eval_range(std::size_t, const ParticleRange<GMM> &range)
    {
        auto alpha = range.particle().state().alpha();
        auto c = range.particle().state().c();
        auto &rng = range.begin().rng();

        NormalMVProposal<> proposal(c, range.particle().state().sd_mu(),
            -std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity());
        RandomWalk<> random_walk(c);

        for (auto idx : range) {
            auto lp = idx.log_prior();
            auto ll = idx.log_likelihood();
            auto lt = lp + alpha * ll;
            auto acc = random_walk(rng, idx.mu(), &lt,
                [idx, alpha, &lp, &ll](std::size_t, const double *x) {
                    lp = idx.particle().state().log_prior(x, idx.lambda());
                    ll = idx.particle().state().log_likelihood(
                        x, idx.lambda(), idx.omega());
                    return lp + alpha * ll;
                },
                proposal);
            if (acc != 0) {
                idx.log_prior() = lp;
                idx.log_likelihood() = ll;
            }
        }
    }
}; // class GMMMu

class GMMLambda : public GMMEvalSMP<GMMLambda>
{
    public:
    void eval_range(std::size_t, const ParticleRange<GMM> &range)
    {
        auto alpha = range.particle().state().alpha();
        auto c = range.particle().state().c();
        auto &rng = range.begin().rng();

        NormalMVProposal<> proposal(c, range.particle().state().sd_lambda(),
            0.0, std::numeric_limits<double>::infinity());
        RandomWalk<> random_walk(c);

        for (auto idx : range) {
            auto lp = idx.log_prior();
            auto ll = idx.log_likelihood();
            auto lt = lp + alpha * ll;
            auto acc = random_walk(rng, idx.lambda(), &lt,
                [idx, alpha, &lp, &ll](std::size_t, const double *x) {
                    lp = idx.particle().state().log_prior(idx.mu(), x);
                    ll = idx.particle().state().log_likelihood(
                        idx.mu(), x, idx.omega());
                    return lp + alpha * ll;
                },
                proposal);
            if (acc != 0) {
                idx.log_prior() = lp;
                idx.log_likelihood() = ll;
            }
        }
    }
}; // class GMMLambda

class GMMOmega : public GMMEvalSMP<GMMOmega>
{
    public:
    void eval_range(std::size_t, const ParticleRange<GMM> &range)
    {
        auto alpha = range.particle().state().alpha();
        auto c = range.particle().state().c();
        auto &rng = range.begin().rng();

        NormalMVLogitProposal<> proposal(
            c, range.particle().state().sd_omega());
        RandomWalk<> random_walk(c);

        for (auto idx : range) {
            auto ll = idx.log_likelihood();
            auto lt = alpha * ll;
            auto acc = random_walk(rng, idx.omega(), &lt,
                [idx, alpha, &ll](std::size_t, const double *x) {
                    ll = idx.particle().state().log_likelihood(
                        idx.mu(), idx.lambda(), x);
                    return alpha * ll;
                },
                proposal);
            if (acc != 0)
                idx.log_likelihood() = ll;
        }
    }
}; // class GMMOmega

inline void GMMNC(std::size_t, std::size_t, Particle<GMM> &particle, double *r)
{
    std::transform(particle.begin(), particle.end(), r,
        [](ParticleIndex<GMM> idx) { return idx.log_likelihood(); });
    mul(particle.size(), particle.state().alpha_inc(), r, r);
}

int main()
{
    const std::size_t N = 1000;
    const std::size_t c = 4;
    const double T = 100;

    Sampler<GMM> sampler(N, c);
    sampler.resample_method(Stratified, 0.5)
        .eval(GMMInit(), SamplerInit)
        .eval(GMMMove(T), SamplerMove)
        .eval(GMMMu(), SamplerMCMC)
        .eval(GMMLambda(), SamplerMCMC)
        .eval(GMMOmega(), SamplerMCMC)
        .monitor("nc", Monitor<GMM>(1, GMMNC));
    auto &nc = sampler.monitor("nc");

    sampler.initialize();
    vsmc::StopWatch watch;
    watch.start();
    double z = 0;
    while (sampler.particle().state().alpha() < 1) {
        sampler.iterate();
        z += nc.record(0);
    }
    watch.stop();
    std::cout << "log(Normalizing constant): " << z << std::endl;
    std::cout << "Time: " << watch.seconds() << std::endl;

    return 0;
}
