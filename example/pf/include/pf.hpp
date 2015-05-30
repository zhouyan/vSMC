//============================================================================
// vSMC/example/pf/include/pf.hpp
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
#ifndef VSMC_EXAMPLE_PF_@SMP@_HPP
#define VSMC_EXAMPLE_PF_@SMP@_HPP
// clang-format on

#include <vsmc/core/sampler.hpp>
#include <vsmc/core/state_matrix.hpp>
#include <vsmc/smp/backend_@smp@.hpp>
#if VSMC_HAS_HDF5
#include <vsmc/utility/hdf5io.hpp>
#endif
#ifdef VSMC_PF_MPI
#include <vsmc/mpi/backend_mpi.hpp>
#endif

#define PF_CV_DO(Res)                                                         \
    cv_do<vsmc::RowMajor>(vsmc::Res, argv, "." #Res ".row");                  \
    cv_do<vsmc::ColMajor>(vsmc::Res, argv, "." #Res ".col");

#define PF_MAIN                                                               \
    if (argc < 3) {                                                           \
        std::cout << "Usage: " << argv[0] << " <input file>"                  \
                  << " <output file>" << std::endl;                           \
        return -1;                                                            \
    }                                                                         \
    PF_CV_DO(Multinomial);                                                    \
    PF_CV_DO(Residual);                                                       \
    PF_CV_DO(Stratified);                                                     \
    PF_CV_DO(Systematic);                                                     \
    PF_CV_DO(ResidualStratified);                                             \
    PF_CV_DO(ResidualSystematic);                                             \
    return 0;

#define PF_MAIN_MPI                                                           \
    vsmc::MPIEnvironment env(argc, argv);                                     \
    PF_MAIN;

// clang-format off
template <typename T>
using StateSMP = vsmc::State@SMP@<T>;

template <typename T, typename Derived>
using InitializeSMP = vsmc::Initialize@SMP@<T, Derived>;

template <typename T, typename Derived>
using MoveSMP = vsmc::Move@SMP@<T, Derived>;

template <typename T, typename Derived>
using MonitorEvalSMP = vsmc::MonitorEval@SMP@<T, Derived>;
// clang-format on

static const std::size_t DataNum = 100;
static const std::size_t ParticleNum = 1000;
static const std::size_t PosX = 0;
static const std::size_t PosY = 1;
static const std::size_t VelX = 2;
static const std::size_t VelY = 3;
static const std::size_t LogL = 4;

#ifdef VSMC_PF_MPI
template <vsmc::MatrixOrder Order>
using StateBase =
    vsmc::StateMPI<StateSMP<vsmc::StateMatrix<Order, 5, double>>>;
#else
template <vsmc::MatrixOrder Order>
using StateBase = StateSMP<vsmc::StateMatrix<Order, 5, double>>;
#endif

template <vsmc::MatrixOrder Order>
class cv_state : public StateBase<Order>
{
    public:
    typedef typename StateBase<Order>::size_type size_type;

    cv_state(size_type N) : StateBase<Order>(N) {}

    double &obs_x(std::size_t iter) { return obs_x_[iter]; }
    double &obs_y(std::size_t iter) { return obs_y_[iter]; }

    double log_likelihood(std::size_t iter, size_type id) const
    {
        using std::log;

        const double scale = 10;
        const double nu = 10;

        double llh_x = scale * (this->state(id, PosX) - obs_x_[iter]);
        double llh_y = scale * (this->state(id, PosY) - obs_y_[iter]);

        llh_x = log(1 + llh_x * llh_x / nu);
        llh_y = log(1 + llh_y * llh_y / nu);

        return -0.5 * (nu + 1) * (llh_x + llh_y);
    }

    void read_data(const char *file)
    {
        if (!file)
            return;

        obs_x_.resize(DataNum);
        obs_y_.resize(DataNum);
        std::ifstream data(file);
        for (std::size_t i = 0; i != DataNum; ++i)
            data >> obs_x_[i] >> obs_y_[i];
        data.close();
    }

    private:
    vsmc::Vector<double> obs_x_;
    vsmc::Vector<double> obs_y_;
};

template <vsmc::MatrixOrder Order>
class cv_init : public InitializeSMP<cv_state<Order>, cv_init<Order>>
{
    public:
    typedef cv_state<Order> cv;

    std::size_t initialize_state(vsmc::SingleParticle<cv> sp) const
    {
        const double sd_pos0 = 2;
        const double sd_vel0 = 1;
        std::normal_distribution<> norm_pos(0, sd_pos0);
        std::normal_distribution<> norm_vel(0, sd_vel0);

        sp.state(PosX) = norm_pos(sp.rng());
        sp.state(PosY) = norm_pos(sp.rng());
        sp.state(VelX) = norm_vel(sp.rng());
        sp.state(VelY) = norm_vel(sp.rng());
        sp.state(LogL) = sp.particle().value().log_likelihood(0, sp.id());

        return 1;
    }

    void initialize_param(vsmc::Particle<cv> &particle, void *file) const
    {
        particle.value().read_data(static_cast<const char *>(file));
    }

    void post_processor(vsmc::Particle<cv> &particle)
    {
        log_weight_.resize(particle.size());
        particle.value().read_state(LogL, log_weight_.data());
        particle.weight_set().set_log_weight(log_weight_.data());
    }

    private:
    vsmc::Vector<double> log_weight_;
};

template <vsmc::MatrixOrder Order>
class cv_move : public MoveSMP<cv_state<Order>, cv_move<Order>>
{
    public:
    typedef cv_state<Order> cv;

    std::size_t move_state(std::size_t iter, vsmc::SingleParticle<cv> sp) const
    {
        using std::sqrt;

        const double sd_pos = sqrt(0.02);
        const double sd_vel = sqrt(0.001);
        const double delta = 0.1;
        std::normal_distribution<> norm_pos(0, sd_pos);
        std::normal_distribution<> norm_vel(0, sd_vel);

        sp.state(PosX) += norm_pos(sp.rng()) + delta * sp.state(VelX);
        sp.state(PosY) += norm_pos(sp.rng()) + delta * sp.state(VelY);
        sp.state(VelX) += norm_vel(sp.rng());
        sp.state(VelY) += norm_vel(sp.rng());
        sp.state(LogL) = sp.particle().value().log_likelihood(iter, sp.id());

        return 1;
    }

    void post_processor(std::size_t, vsmc::Particle<cv> &particle)
    {
        inc_weight_.resize(particle.size());
        particle.value().read_state(LogL, inc_weight_.data());
        particle.weight_set().add_log_weight(inc_weight_.data());
    }

    private:
    vsmc::Vector<double> inc_weight_;
};

template <vsmc::MatrixOrder Order>
class cv_meval : public MonitorEvalSMP<cv_state<Order>, cv_meval<Order>>
{
    public:
    typedef cv_state<Order> cv;

    void monitor_state(
        std::size_t, std::size_t, vsmc::SingleParticle<cv> sp, double *res)
    {
        res[0] = sp.state(PosX);
        res[1] = sp.state(PosY);
    }
};

template <vsmc::MatrixOrder Order>
inline void cv_do(
    vsmc::ResampleScheme res, char **argv, const std::string &name)
{
#ifdef VSMC_PF_MPI
    boost::mpi::communicator world;
    std::size_t N = ParticleNum / static_cast<std::size_t>(world.size());
    std::stringstream ss;
    ss << name << ".r" << world.rank();
    std::string rname(ss.str());
    std::string pf_txt(argv[2] + rname + ".txt");
    std::string pf_h5(argv[2] + rname + ".h5");
#else
    std::size_t N = ParticleNum;
    std::string pf_txt(argv[2] + name + ".txt");
    std::string pf_h5(argv[2] + name + ".h5");
#endif

    typedef cv_state<Order> cv;

    vsmc::Seed::instance().set(101);
    vsmc::Sampler<cv> sampler(N, res, 0.5);
    sampler.init(cv_init<Order>());
    sampler.move(cv_move<Order>(), false);
    sampler.monitor("pos", 2, cv_meval<Order>());
    sampler.monitor("pos").name(0) = "pos.x";
    sampler.monitor("pos").name(1) = "pos.y";
    sampler.initialize(argv[1]);
    sampler.iterate(DataNum - 1);

    std::ofstream pf_sampler(pf_txt);
    pf_sampler << sampler << std::endl;
    pf_sampler.close();

#if VSMC_HAS_HDF5
    vsmc::hdf5store(sampler, pf_h5, "Sampler");
#endif
}

#endif // VSMC_EXAMPLE_PF_@SMP@_HPP
