#include <vsmc/vsmc.hpp>

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

        std::ifstream data(param);
        while (data.good()) {
            double x;
            double y;
            data >> x >> y;
            if (data.good()) {
                obs_x_.push_back(x);
                obs_y_.push_back(y);
            }
        }
        data.close();
    }

    std::size_t data_size() const { return obs_x_.size(); }

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
        auto &rng = particle.rng();
        const std::size_t size = particle.size();
        const double sd_pos = 2;
        const double sd_vel = 1;
        pos_x_.resize(size);
        pos_y_.resize(size);
        vel_x_.resize(size);
        vel_y_.resize(size);
        weight_.resize(size);
        vsmc::normal_distribution(rng, size, pos_x_.data(), 0.0, sd_pos);
        vsmc::normal_distribution(rng, size, pos_y_.data(), 0.0, sd_pos);
        vsmc::normal_distribution(rng, size, vel_x_.data(), 0.0, sd_vel);
        vsmc::normal_distribution(rng, size, vel_y_.data(), 0.0, sd_vel);
    }

    std::size_t eval_sp(vsmc::SingleParticle<PFState> sp)
    {
        sp.state(PosX) = pos_x_[sp.id()];
        sp.state(PosY) = pos_y_[sp.id()];
        sp.state(VelX) = vel_x_[sp.id()];
        sp.state(VelY) = vel_y_[sp.id()];
        weight_[sp.id()] = sp.particle().value().log_likelihood(0, sp.id());

        return 0;
    }

    void eval_post(vsmc::Particle<PFState> &particle)
    {
        particle.weight().set_log(weight_.data());
    }

    private:
    vsmc::Vector<double> pos_x_;
    vsmc::Vector<double> pos_y_;
    vsmc::Vector<double> vel_x_;
    vsmc::Vector<double> vel_y_;
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
        auto &rng = particle.rng();
        const std::size_t size = particle.size();
        const double sd_pos = sqrt(0.02);
        const double sd_vel = sqrt(0.001);
        pos_x_.resize(size);
        pos_y_.resize(size);
        vel_x_.resize(size);
        vel_y_.resize(size);
        weight_.resize(size);
        vsmc::normal_distribution(rng, size, pos_x_.data(), 0.0, sd_pos);
        vsmc::normal_distribution(rng, size, pos_y_.data(), 0.0, sd_pos);
        vsmc::normal_distribution(rng, size, vel_x_.data(), 0.0, sd_vel);
        vsmc::normal_distribution(rng, size, vel_y_.data(), 0.0, sd_vel);
    }

    std::size_t eval_sp(std::size_t t, vsmc::SingleParticle<PFState> sp)
    {
        sp.state(PosX) += pos_x_[sp.id()] + 0.1 * sp.state(VelX);
        sp.state(PosY) += pos_y_[sp.id()] + 0.1 * sp.state(VelY);
        sp.state(VelX) += vel_x_[sp.id()];
        sp.state(VelY) += vel_y_[sp.id()];
        weight_[sp.id()] = sp.particle().value().log_likelihood(t, sp.id());

        return 0;
    }

    void eval_post(std::size_t t, vsmc::Particle<PFState> &particle)
    {
        particle.weight().add_log(weight_.data());
    }

    private:
    vsmc::Vector<double> pos_x_;
    vsmc::Vector<double> pos_y_;
    vsmc::Vector<double> vel_x_;
    vsmc::Vector<double> vel_y_;
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

int main(int argc, char **argv)
{
    std::size_t N = 10000;
    if (argc > 1)
        N = static_cast<std::size_t>(std::atoi(argv[1]));

    vsmc::Sampler<PFState> sampler(N, vsmc::Multinomial, 0.5);
    sampler.init(PFInit()).move(PFMove(), false).monitor("pos", 2, PFEval());

    vsmc::StopWatch watch;
    watch.start();
    sampler.initialize(const_cast<char *>("pf.data"));
    sampler.iterate(sampler.particle().value().data_size() - 1);
    watch.stop();
    std::cout << "Time (ms): " << watch.milliseconds() << std::endl;

    std::ofstream output("pf.out");
    output << sampler;
    output.close();

    return 0;
}
