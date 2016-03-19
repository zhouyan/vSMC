#include <vsmc/vsmc.hpp>

static constexpr std::size_t N = 1000; // Number of particles
static constexpr std::size_t n = 100;  // Number of data points
static constexpr std::size_t PosX = 0;
static constexpr std::size_t PosY = 1;
static constexpr std::size_t VelX = 2;
static constexpr std::size_t VelY = 3;

using PFStateBase = vsmc::StateMatrix<vsmc::RowMajor, 4, double>;

class PFState : public PFStateBase
{
    public:
    using PFStateBase::PFStateBase;

    double log_likelihood(std::size_t t, size_type i) const
    {
        double llh_x = 10 * (this->state(i, PosX) - obs_x_[t]);
        double llh_y = 10 * (this->state(i, PosY) - obs_y_[t]);
        llh_x = std::log(1 + llh_x * llh_x / 10);
        llh_y = std::log(1 + llh_y * llh_y / 10);

        return -0.5 * (10 + 1) * (llh_x + llh_y);
    }

    void read_data(const char *param)
    {
        if (param == nullptr)
            return;

        obs_x_.resize(n);
        obs_y_.resize(n);
        std::ifstream data(param);
        for (std::size_t i = 0; i != n; ++i)
            data >> obs_x_[i] >> obs_y_[i];
        data.close();
    }

    private:
    vsmc::Vector<double> obs_x_;
    vsmc::Vector<double> obs_y_;
};

class PFInit
{
    public:
    std::size_t operator()(vsmc::Particle<PFState> &particle, void *param)
    {
        eval_param(particle, param);
        eval_pre(particle);
        std::size_t acc = 0;
        for (auto sp : particle)
            acc += eval_sp(sp);
        eval_post(particle);

        return acc;
    }

    void eval_param(vsmc::Particle<PFState> &particle, void *param)
    {
        particle.value().read_data(static_cast<const char *>(param));
    }

    void eval_pre(vsmc::Particle<PFState> &particle)
    {
        weight_.resize(particle.size());
    }

    std::size_t eval_sp(vsmc::SingleParticle<PFState> sp)
    {
        vsmc::NormalDistribution<double> norm_pos(0, 2);
        vsmc::NormalDistribution<double> norm_vel(0, 1);
        sp.state(PosX) = norm_pos(sp.rng());
        sp.state(PosY) = norm_pos(sp.rng());
        sp.state(VelX) = norm_vel(sp.rng());
        sp.state(VelY) = norm_vel(sp.rng());
        weight_[sp.id()] = sp.particle().value().log_likelihood(0, sp.id());

        return 0;
    }

    void eval_post(vsmc::Particle<PFState> &particle)
    {
        particle.weight().set_log(weight_.data());
    }

    private:
    vsmc::Vector<double> weight_;
};

class PFMove
{
    public:
    std::size_t operator()(std::size_t t, vsmc::Particle<PFState> &particle)
    {
        eval_pre(t, particle);
        std::size_t acc = 0;
        for (auto sp : particle)
            acc += eval_sp(t, sp);
        eval_post(t, particle);

        return 0;
    }

    void eval_pre(std::size_t t, vsmc::Particle<PFState> &particle)
    {
        weight_.resize(particle.size());
    }

    std::size_t eval_sp(std::size_t t, vsmc::SingleParticle<PFState> sp)
    {
        vsmc::NormalDistribution<double> norm_pos(0, std::sqrt(0.02));
        vsmc::NormalDistribution<double> norm_vel(0, std::sqrt(0.001));
        sp.state(PosX) += norm_pos(sp.rng()) + 0.1 * sp.state(VelX);
        sp.state(PosY) += norm_pos(sp.rng()) + 0.1 * sp.state(VelY);
        sp.state(VelX) += norm_vel(sp.rng());
        sp.state(VelY) += norm_vel(sp.rng());
        weight_[sp.id()] = sp.particle().value().log_likelihood(t, sp.id());

        return 0;
    }

    void eval_post(std::size_t t, vsmc::Particle<PFState> &particle)
    {
        particle.weight().add_log(weight_.data());
    }

    private:
    vsmc::Vector<double> weight_;
};

class PFEval
{
    public:
    void operator()(std::size_t t, std::size_t dim,
        vsmc::Particle<PFState> &particle, double *r)
    {
        eval_pre(t, particle);
        for (std::size_t i = 0; i != particle.size(); ++i, r += dim)
            eval_sp(t, dim, particle.sp(i), r);
        eval_post(t, particle);
    }

    void eval_pre(std::size_t t, vsmc::Particle<PFState> &particle) {}

    void eval_sp(std::size_t t, std::size_t dim,
        vsmc::SingleParticle<PFState> sp, double *r)
    {
        r[0] = sp.state(PosX);
        r[1] = sp.state(PosY);
    }

    void eval_post(std::size_t t, vsmc::Particle<PFState> &particle) {}
};

int main()
{
    vsmc::Sampler<PFState> sampler(N, vsmc::Multinomial, 0.5);
    sampler.init(PFInit()).move(PFMove(), false).monitor("pos", 2, PFEval());

    vsmc::StopWatch watch;
    watch.start();
    sampler.initialize(const_cast<char *>("pf.data")).iterate(n - 1);
    watch.stop();
    std::cout << "Time: " << watch.milliseconds() << std::endl;

    std::ofstream output("pf.out");
    output << sampler;
    output.close();

    return 0;
}
