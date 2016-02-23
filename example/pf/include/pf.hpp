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
#include <vsmc/utility/stop_watch.hpp>
#if VSMC_HAS_HDF5
#include <vsmc/utility/hdf5io.hpp>
#endif

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

template <vsmc::MatrixLayout Layout>
using StateBase = StateSMP<vsmc::StateMatrix<Layout, 5, double>>;

template <vsmc::MatrixLayout Layout>
class pf_state : public StateBase<Layout>
{
    public:
    using size_type = typename StateBase<Layout>::size_type;

    pf_state(size_type N) : StateBase<Layout>(N) {}

    double &obs_x(std::size_t iter) { return obs_x_[iter]; }
    double &obs_y(std::size_t iter) { return obs_y_[iter]; }

    double log_likelihood(std::size_t iter, size_type id) const
    {
        const double scale = 10;
        const double nu = 10;

        double llh_x = scale * (this->state(id, PosX) - obs_x_[iter]);
        double llh_y = scale * (this->state(id, PosY) - obs_y_[iter]);

        llh_x = std::log(1 + llh_x * llh_x / nu);
        llh_y = std::log(1 + llh_y * llh_y / nu);

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

template <vsmc::MatrixLayout Layout>
class pf_init : public InitializeSMP<pf_state<Layout>, pf_init<Layout>>
{
    public:
    std::size_t eval_sp(vsmc::SingleParticle<pf_state<Layout>> sp) const
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

    void eval_param(
        vsmc::Particle<pf_state<Layout>> &particle, void *file) const
    {
        particle.value().read_data(static_cast<const char *>(file));
    }

    void eval_post(vsmc::Particle<pf_state<Layout>> &particle)
    {
        w_.resize(particle.size());
        particle.value().read_state(LogL, w_.data());
        particle.weight().set_log(w_.data());
    }

    private:
    vsmc::Vector<double> w_;
};

template <vsmc::MatrixLayout Layout>
class pf_move : public MoveSMP<pf_state<Layout>, pf_move<Layout>>
{
    public:
    std::size_t eval_sp(
        std::size_t iter, vsmc::SingleParticle<pf_state<Layout>> sp) const
    {
        const double sd_pos = std::sqrt(0.02);
        const double sd_vel = std::sqrt(0.001);
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

    void eval_post(std::size_t, vsmc::Particle<pf_state<Layout>> &particle)
    {
        w_.resize(particle.size());
        particle.value().read_state(LogL, w_.data());
        particle.weight().add_log(w_.data());
    }

    private:
    vsmc::Vector<double> w_;
};

template <vsmc::MatrixLayout Layout>
class pf_meval : public MonitorEvalSMP<pf_state<Layout>, pf_meval<Layout>>
{
    public:
    void eval_sp(std::size_t, std::size_t,
        vsmc::SingleParticle<pf_state<Layout>> sp, double *res)
    {
        res[0] = sp.state(PosX);
        res[1] = sp.state(PosY);
    }
};

template <vsmc::MatrixLayout Layout>
inline void pf_run(vsmc::ResampleScheme scheme, const std::string &datafile,
    const std::string &prog, const std::string &name)
{
    std::size_t N = ParticleNum;
    std::string pf_txt(prog + name + ".txt");
    std::string pf_h5(prog + name + ".h5");

    vsmc::Seed::instance().set(101);
    vsmc::Sampler<pf_state<Layout>> sampler(N, scheme, 0.5);
    sampler.init(pf_init<Layout>());
    sampler.move(pf_move<Layout>(), false);
    sampler.monitor("pos", 2, pf_meval<Layout>());
    sampler.monitor("pos").name(0) = "pos.x";
    sampler.monitor("pos").name(1) = "pos.y";

    vsmc::StopWatch watch;
    watch.start();
    sampler.initialize(const_cast<char *>(datafile.c_str()));
    sampler.iterate(DataNum - 1);
    watch.stop();
    std::cout << std::setw(40) << std::left << name;
    std::cout << watch.milliseconds() << std::endl;

    std::ofstream pf_sampler(pf_txt);
    pf_sampler << sampler << std::endl;
    pf_sampler.close();

#if VSMC_HAS_HDF5
    vsmc::hdf5store(sampler, pf_h5, "Sampler");
#endif
}

inline void pf_run(vsmc::ResampleScheme scheme, char **argv)
{
    std::string resname;
    switch (scheme) {
        case vsmc::Multinomial: resname = "Multinomial"; break;
        case vsmc::Stratified: resname = "Stratified"; break;
        case vsmc::Systematic: resname = "Systematic"; break;
        case vsmc::Residual: resname = "Residual"; break;
        case vsmc::ResidualStratified: resname = "ResidualStratified"; break;
        case vsmc::ResidualSystematic: resname = "ResidualSystematic"; break;
    }
    pf_run<vsmc::RowMajor>(scheme, argv[1], argv[2], "." + resname + ".row");
    pf_run<vsmc::ColMajor>(scheme, argv[1], argv[2], "." + resname + ".col");
}

inline int pf_main(int argc, char **argv)
{
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <input file>"
                  << " <output file>" << std::endl;
        return -1;
    }

    pf_run(vsmc::Multinomial, argv);
    pf_run(vsmc::Stratified, argv);
    pf_run(vsmc::Systematic, argv);
    pf_run(vsmc::Residual, argv);
    pf_run(vsmc::ResidualStratified, argv);
    pf_run(vsmc::ResidualSystematic, argv);

    return 0;
}

#endif // VSMC_EXAMPLE_PF_@SMP@_HPP
