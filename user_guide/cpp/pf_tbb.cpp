#include <vsmc/vsmc.hpp>

static constexpr std::size_t N = 1000; // Number of particles
static constexpr std::size_t n = 100;  // Number of data points

using PFStateBase = vsmc::StateMatrix<vsmc::RowMajor, 4, double>;

template <typename T>
using PFStateSPBase = PFStateBase::single_particle_type<T>;

class PFState : public PFStateBase
{
    public:
    using PFStateBase::StateMatrix;

    template <typename S>
    class single_particle_type : public PFStateSPBase<S>
    {
        public:
        using PFStateSPBase<S>::single_particle_type;

        double &pos_x() { return this->state(0); }
        double &pos_y() { return this->state(1); }
        double &vel_x() { return this->state(2); }
        double &vel_y() { return this->state(3); }

        double log_likelihood(std::size_t t)
        {
            double llh_x = 10 * (pos_x() - obs_x(t));
            double llh_y = 10 * (pos_y() - obs_y(t));
            llh_x = std::log(1 + llh_x * llh_x / 10);
            llh_y = std::log(1 + llh_y * llh_y / 10);

            return -0.5 * (10 + 1) * (llh_x + llh_y);
        }

        private:
        double obs_x(std::size_t t)
        {
            return this->particle().value().obs_x_[t];
        }

        double obs_y(std::size_t t)
        {
            return this->particle().value().obs_y_[t];
        }
    };

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

class PFInit : public vsmc::InitializeTBB<PFState, PFInit>
{
    public:
    void eval_param(vsmc::Particle<PFState> &particle, void *param)
    {
        particle.value().read_data(static_cast<const char *>(param));
    }

    void eval_pre(vsmc::Particle<PFState> &particle)
    {
        w_.resize(particle.size());
    }

    std::size_t eval_sp(vsmc::SingleParticle<PFState> sp)
    {
        vsmc::NormalDistribution<double> norm_pos(0, 2);
        vsmc::NormalDistribution<double> norm_vel(0, 1);
        sp.pos_x() = norm_pos(sp.rng());
        sp.pos_y() = norm_pos(sp.rng());
        sp.vel_x() = norm_vel(sp.rng());
        sp.vel_y() = norm_vel(sp.rng());
        w_[sp.id()] = sp.log_likelihood(0);

        return 0;
    }

    void eval_post(vsmc::Particle<PFState> &particle)
    {
        particle.weight().set_log(w_.data());
    }

    private:
    vsmc::Vector<double> w_;
};

class PFMove : public vsmc::MoveTBB<PFState, PFMove>
{
    public:
    void eval_pre(std::size_t t, vsmc::Particle<PFState> &particle)
    {
        w_.resize(particle.size());
    }

    std::size_t eval_sp(std::size_t t, vsmc::SingleParticle<PFState> sp)
    {
        vsmc::NormalDistribution<double> norm_pos(0, std::sqrt(0.02));
        vsmc::NormalDistribution<double> norm_vel(0, std::sqrt(0.001));
        sp.pos_x() += norm_pos(sp.rng()) + 0.1 * sp.vel_x();
        sp.pos_y() += norm_pos(sp.rng()) + 0.1 * sp.vel_y();
        sp.vel_x() += norm_vel(sp.rng());
        sp.vel_y() += norm_vel(sp.rng());
        w_[sp.id()] = sp.log_likelihood(t);

        return 0;
    }

    void eval_post(std::size_t t, vsmc::Particle<PFState> &particle)
    {
        particle.weight().add_log(w_.data());
    }

    private:
    vsmc::Vector<double> w_;
};

class PFMEval : public vsmc::MonitorEvalTBB<PFState, PFMEval>
{
    public:
    void eval_sp(std::size_t t, std::size_t dim,
        vsmc::SingleParticle<PFState> sp, double *r)
    {
        r[0] = sp.pos_x();
        r[1] = sp.pos_y();
    }
};

int main()
{
    vsmc::Sampler<PFState> sampler(N, vsmc::Multinomial, 0.5);
    sampler.init(PFInit()).move(PFMove(), false).monitor("pos", 2, PFMEval());

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
