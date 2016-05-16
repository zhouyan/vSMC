#include <vsmc/vsmc.hpp>

using namespace vsmc;

class GMMModel
{
    public:
    GMMModel(std::size_t r, std::size_t n, const double *obs) : r_(r), obs_(n)
    {
        std::copy_n(obs, n, obs_.data());
        double ymin = *std::min_element(obs_.begin(), obs_.end());
        double ymax = *std::max_element(obs_.begin(), obs_.end());
        xi_ = 0.5 * (ymin + ymax);
        kappa_ = 1 / ((ymax - ymin) * (ymax - ymin));
        nu_ = 2;
        chi_ = 50 * kappa_;
        rho_ = 1;

        lp_const_ = 0;
        lp_const_ -= 0.5 * const_ln_pi_2<double>();
        lp_const_ += 0.5 * std::log(kappa_);
        lp_const_ -= std::lgamma(nu_);
        lp_const_ -= nu_ * std::log(chi_);
        lp_const_ += std::lgamma(r_ * rho_);
        lp_const_ -= r_ * std::lgamma(rho_);

        ll_const_ = -0.5 * obs_.size() * const_ln_pi_2<double>();
    }

    std::size_t r() const { return r_; }

    auto prior_mu() const
    {
        return NormalDistribution<double>(xi_, 1 / std::sqrt(kappa_));
    }

    auto prior_lambda() const { return GammaDistribution<double>(nu_, chi_); }

    auto prior_omega() const
    {
        return DirichletDistribution<double>(r_, rho_);
    }

    auto proposal_mu(double sd) const
    {
        return NormalMVProposal<double>(
            r_, sd, -const_inf<double>(), const_inf<double>());
    }

    auto proposal_lambda(double sd) const
    {
        return NormalMVProposal<double>(r_, sd, 0.0, const_inf<double>());
    }

    auto proposal_omega(double sd) const
    {
        return NormalMVLogitProposal<double>(r_, sd);
    }

    auto random_walk() const { return RandomWalk<double>(r_); }

    double log_prior(
        const double *mu, const double *lambda, const double *omega) const
    {
        double lp = lp_const_;
        for (std::size_t j = 0; j != r_; ++j) {
            double tmp = mu[j] - xi_;
            lp -= 0.5 * kappa_ * tmp * tmp;
        }
        for (std::size_t j = 0; j != r_; ++j) {
            lp += (nu_ - 1) * std::log(lambda[j]);
            lp -= lambda[j] / chi_;
        }
        if (rho_ > 1 || rho_ < 1) {
            for (std::size_t j = 0; j != r_; ++j)
                lp += (rho_ - 1) * std::log(omega[j]);
        }

        return lp;
    }

    double log_likelihood(
        const double *mu, const double *lambda, const double *omega) const
    {
        Vector<double> log_lambda(r_);
        log(r_, lambda, log_lambda.data());
        double ll = ll_const_;
        for (std::size_t i = 0; i != obs_.size(); ++i) {
            double y = obs_[i];
            double lli = 0;
            for (std::size_t j = 0; j != r_; ++j) {
                double tmp = y - mu[j];
                tmp = log_lambda[j] - lambda[j] * tmp * tmp;
                lli += omega[j] * std::exp(0.5 * tmp);
            }
            ll += std::log(lli + 1e-13);
        }

        return ll;
    }

    private:
    std::size_t r_;
    Vector<double> obs_;
    double xi_ = 0;
    double kappa_ = 1;
    double nu_ = 2;
    double chi_ = 1;
    double rho_ = 1;
    double lp_const_ = 0;
    double ll_const_ = 0;
}; // class GMMModel

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

        double *mu() const { return &this->at(0); }
        double *lambda() const { return &this->at(r()); }
        double *omega() const { return &this->at(r() * 2); }
        double &log_prior() const { return this->at(r() * 3); }
        double &log_likelihood() const { return this->at(r() * 3 + 1); }

        private:
        std::size_t r() const { return this->particle().state().model_.r(); }
    }; // class particle_index_type

    GMM(std::size_t N, const GMMModel &model)
        : GMMBase(N, model.r() * 3 + 2), model_(model)
    {
    }

    double phi() const { return phi_; }

    double phi_inc() const { return phi_inc_; }

    void phi(double p)
    {
        if (p > 0) {
            phi_inc_ = p - phi_;
            phi_ = p;
        } else {
            phi_inc_ = 0;
            phi_ = 0;
        }
    }

    private:
    GMMModel model_;
    double phi_ = 0;
    double phi_inc_ = 0;
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
    GMMInit(const GMMModel &model) : model_(model) {}

    void eval_range(std::size_t, const ParticleRange<GMM> &range)
    {
        auto r = model_.r();
        auto prior_mu = model_.prior_mu();
        auto prior_lambda = model_.prior_lambda();
        auto prior_omega = model_.prior_omega();
        auto &rng = range.begin().rng();
        for (auto idx : range) {
            prior_mu(rng, r, idx.mu());
            prior_lambda(rng, r, idx.lambda());
            prior_omega(rng, idx.omega());
            idx.log_prior() =
                model_.log_prior(idx.mu(), idx.lambda(), idx.omega());
            idx.log_likelihood() =
                model_.log_likelihood(idx.mu(), idx.lambda(), idx.omega());
        }
    }

    void eval_first(std::size_t, Particle<GMM> &particle)
    {
        particle.state().phi(0);
    }

    private:
    GMMModel model_;
}; // class GMMInit

class GMMMove
{
    public:
    GMMMove(double T) : T_(T) {}

    void operator()(std::size_t iter, Particle<GMM> &particle)
    {
        auto phi = iter / T_;
        phi = std::min(1.0, phi * phi);
        particle.state().phi(phi);

        w_.resize(particle.size());
        std::transform(particle.begin(), particle.end(), w_.begin(),
            [](ParticleIndex<GMM> idx) { return idx.log_likelihood(); });
        mul(w_.size(), particle.state().phi_inc(), w_.data(), w_.data());
        particle.weight().add_log(w_.data());
    }

    private:
    double T_;
    Vector<double> w_;
}; // class GMMMove

class GMMMu : public GMMEvalSMP<GMMMu>
{
    public:
    GMMMu(const GMMModel &model) : model_(model) {}

    void eval_range(std::size_t, const ParticleRange<GMM> &range)
    {
        auto phi = range.particle().state().phi();
        auto scale = std::max(0.02, phi);
        auto sd = 0.15 / scale;
        auto proposal = model_.proposal_mu(sd);
        auto random_walk = model_.random_walk();
        auto &rng = range.begin().rng();
        for (auto idx : range) {
            auto lp = idx.log_prior();
            auto ll = idx.log_likelihood();
            auto lt = lp + phi * ll;
            auto pi = [this, idx, phi, &lp, &ll](const double *x) {
                lp = model_.log_prior(x, idx.lambda(), idx.omega());
                ll = model_.log_likelihood(x, idx.lambda(), idx.omega());
                return lp + phi * ll;
            };
            auto acc = random_walk(rng, idx.mu(), &lt, pi, proposal);
            if (acc != 0) {
                idx.log_prior() = lp;
                idx.log_likelihood() = ll;
            }
        }
    }

    private:
    GMMModel model_;
    double sd_;
}; // class GMMMu

class GMMLambda : public GMMEvalSMP<GMMLambda>
{
    public:
    GMMLambda(const GMMModel &model) : model_(model) {}

    void eval_range(std::size_t, const ParticleRange<GMM> &range)
    {
        auto phi = range.particle().state().phi();
        auto scale = std::max(0.02, phi);
        auto sd = 0.15 * (1 + std::sqrt(1 / scale));
        auto proposal = model_.proposal_lambda(sd);
        auto random_walk = model_.random_walk();
        auto &rng = range.begin().rng();
        for (auto idx : range) {
            auto lp = idx.log_prior();
            auto ll = idx.log_likelihood();
            auto lt = lp + phi * ll;
            auto pi = [this, idx, phi, &lp, &ll](const double *x) {
                lp = model_.log_prior(idx.mu(), x, idx.omega());
                ll = model_.log_likelihood(idx.mu(), x, idx.omega());
                return lp + phi * ll;
            };
            auto acc = random_walk(rng, idx.lambda(), &lt, pi, proposal);
            if (acc != 0) {
                idx.log_prior() = lp;
                idx.log_likelihood() = ll;
            }
        }
    }

    private:
    GMMModel model_;
}; // class GMMLambda

class GMMOmega : public GMMEvalSMP<GMMOmega>
{
    public:
    GMMOmega(const GMMModel &model) : model_(model) {}

    void eval_range(std::size_t, const ParticleRange<GMM> &range)
    {
        auto phi = range.particle().state().phi();
        auto scale = std::max(0.02, phi);
        auto sd = 0.2 * (1 + std::sqrt(1 / scale));
        auto proposal = model_.proposal_omega(sd);
        auto random_walk = model_.random_walk();
        auto &rng = range.begin().rng();
        for (auto idx : range) {
            auto lp = idx.log_prior();
            auto ll = idx.log_likelihood();
            auto lt = lp + phi * ll;
            auto pi = [this, idx, phi, &lp, &ll](const double *x) {
                lp = model_.log_prior(idx.mu(), idx.lambda(), x);
                ll = model_.log_likelihood(idx.mu(), idx.lambda(), x);
                return lp + phi * ll;
            };
            auto acc = random_walk(rng, idx.omega(), &lt, pi, proposal);
            if (acc != 0) {
                idx.log_prior() = lp;
                idx.log_likelihood() = ll;
            }
        }
    }

    private:
    GMMModel model_;
}; // class GMMOmega

inline void GMMLogLikelihood(
    std::size_t, std::size_t, Particle<GMM> &particle, double *r)
{
    std::transform(particle.begin(), particle.end(), r,
        [](ParticleIndex<GMM> idx) { return idx.log_likelihood(); });
}

int main()
{
    const std::size_t r = 4;
    const std::size_t N = 1000;
    const double T = 100;

    Vector<double> obs;
    std::ifstream data("gmm.data");
    double y = 0;
    while (data >> y)
        obs.push_back(y);
    data.close();
    GMMModel model(r, obs.size(), obs.data());

    Sampler<GMM> sampler(N, model);
    sampler.resample_method(Stratified, 0.5)
        .eval(GMMInit(model), SamplerInit)
        .eval(GMMMove(T), SamplerMove)
        .eval(GMMMu(model), SamplerMCMC)
        .eval(GMMLambda(model), SamplerMCMC)
        .eval(GMMOmega(model), SamplerMCMC)
        .monitor("ll", Monitor<GMM>(1, GMMLogLikelihood));

    sampler.initialize();
    vsmc::StopWatch watch;
    watch.start();
    double lz = 0;
    auto &ll = sampler.monitor("ll");
    while (sampler.particle().state().phi() < 1) {
        sampler.iterate();
        lz += sampler.particle().state().phi_inc() * ll.record(0);
    }
    watch.stop();
    std::cout << "log(Normalizing constant): " << lz << std::endl;
    std::cout << "Time: " << watch.seconds() << std::endl;

    return 0;
}
