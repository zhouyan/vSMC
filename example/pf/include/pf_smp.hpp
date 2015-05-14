//============================================================================
// vSMC/example/pf/include/pf_smp.hpp
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

#ifndef VSMC_EXAMPLE_PF_SMP_HPP
#define VSMC_EXAMPLE_PF_SMP_HPP

#ifdef VSMC_EXAMPLE_PF_TBB_HPP
#define VSMC_EXAMPLE_PF_USE_TBB 1
#else
#define VSMC_EXAMPLE_PF_USE_TBB 0
#endif

#include <vsmc/core/sampler.hpp>
#include <vsmc/rng/threefry.hpp>

#if VSMC_HAS_HDF5
#include <vsmc/utility/hdf5io.hpp>
#endif

#include <fstream>

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

static const std::size_t DataNum = 100;
static const std::size_t ParticleNum = 1000;
static const std::size_t PosX = 0;
static const std::size_t PosY = 1;
static const std::size_t VelX = 2;
static const std::size_t VelY = 3;
static const std::size_t LogL = 4;

template <vsmc::MatrixOrder Order>
class cv_state : public StateBase<Order>
{
    public:
    typedef vsmc::RngSetScalar<vsmc::Threefry4x32> rng_set_type;
    typedef typename StateBase<Order>::size_type size_type;

    cv_state(size_type N) : StateBase<Order>(N) {}

    double &obs_x(std::size_t iter) { return obs_x_[iter]; }
    double &obs_y(std::size_t iter) { return obs_y_[iter]; }

    double log_likelihood(std::size_t iter, size_type id) const
    {
        using std::log;

        const double scale = 10;
        const double nu = 10;

        double llh_x =
            scale * (this->state(id, vsmc::Position<PosX>()) - obs_x_[iter]);
        double llh_y =
            scale * (this->state(id, vsmc::Position<PosY>()) - obs_y_[iter]);

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
    std::vector<double> obs_x_;
    std::vector<double> obs_y_;
};

template <vsmc::MatrixOrder Order>
class cv_init : public InitializeSMP<cv_state<Order>, cv_init<Order>>
{
    public:
    typedef cv_state<Order> cv;

#if VSMC_EXAMPLE_PF_USE_TBB
    std::size_t operator()(vsmc::Particle<cv> &particle, void *param)
    {
        return this->parallel_run(particle, param,
            tbb::blocked_range<typename vsmc::Particle<cv>::size_type>(0,
                                      particle.size(), particle.size() / 20),
            tbb::simple_partitioner());
    }
#endif

    std::size_t initialize_state(vsmc::SingleParticle<cv> sp) const
    {
        const double sd_pos0 = 2;
        const double sd_vel0 = 1;
        std::normal_distribution<> norm_pos(0, sd_pos0);
        std::normal_distribution<> norm_vel(0, sd_vel0);

        typedef typename vsmc::Particle<cv>::rng_type rng_type;
        rng_type rng(sp.rng());
        typename rng_type::ctr_type ctr;
        ctr.fill(0);
        ctr.back() =
            static_cast<typename rng_type::ctr_type::value_type>(sp.id());
        ctr.back() <<= 16;
        rng.ctr(ctr);
        sp.template state<PosX>() = norm_pos(rng);
        sp.template state<PosY>() = norm_pos(rng);
        sp.template state<VelX>() = norm_vel(rng);
        sp.template state<VelY>() = norm_vel(rng);
        sp.template state<LogL>() =
            sp.particle().value().log_likelihood(0, sp.id());

        return 1;
    }

    void initialize_param(vsmc::Particle<cv> &particle, void *file) const
    {
        particle.value().read_data(static_cast<const char *>(file));
    }

    void post_processor(vsmc::Particle<cv> &particle)
    {
        log_weight_.resize(particle.size());
        particle.value().read_state(
            vsmc::Position<LogL>(), log_weight_.data());
        particle.weight_set().set_log_weight(log_weight_.data());
    }

    private:
    std::vector<double> log_weight_;
};

template <vsmc::MatrixOrder Order>
class cv_move : public MoveSMP<cv_state<Order>, cv_move<Order>>
{
    public:
    typedef cv_state<Order> cv;

#if VSMC_EXAMPLE_PF_USE_TBB
    std::size_t operator()(std::size_t iter, vsmc::Particle<cv> &particle)
    {
        static tbb::affinity_partitioner partitioner;
        return this->parallel_run(iter, particle,
            tbb::blocked_range<typename vsmc::Particle<cv>::size_type>(
                                      0, particle.size()),
            partitioner);
    }
#endif

    std::size_t move_state(std::size_t iter, vsmc::SingleParticle<cv> sp) const
    {
        using std::sqrt;

        const double sd_pos = sqrt(0.02);
        const double sd_vel = sqrt(0.001);
        const double delta = 0.1;
        std::normal_distribution<> norm_pos(0, sd_pos);
        std::normal_distribution<> norm_vel(0, sd_vel);

        typedef typename vsmc::Particle<cv>::rng_type rng_type;
        rng_type rng(sp.rng());
        typename rng_type::ctr_type ctr;
        ctr.fill(0);
        ctr.back() =
            static_cast<typename rng_type::ctr_type::value_type>(sp.id());
        ctr.back() <<= 16;
        ctr.back() +=
            static_cast<typename rng_type::ctr_type::value_type>(iter);
        rng.ctr(ctr);
        sp.state(vsmc::Position<PosX>()) +=
            norm_pos(rng) + delta * sp.state(vsmc::Position<VelX>());
        sp.state(vsmc::Position<PosY>()) +=
            norm_pos(rng) + delta * sp.state(vsmc::Position<VelY>());
        sp.state(vsmc::Position<VelX>()) += norm_vel(rng);
        sp.state(vsmc::Position<VelY>()) += norm_vel(rng);
        sp.state(vsmc::Position<LogL>()) =
            sp.particle().value().log_likelihood(iter, sp.id());

        return 1;
    }

    void post_processor(std::size_t, vsmc::Particle<cv> &particle)
    {
        inc_weight_.resize(particle.size());
        particle.value().read_state(
            vsmc::Position<LogL>(), inc_weight_.data());
        particle.weight_set().add_log_weight(inc_weight_.data());
    }

    private:
    std::vector<double> inc_weight_;
};

template <vsmc::MatrixOrder Order>
class cv_est : public MonitorEvalSMP<cv_state<Order>, cv_est<Order>>
{
    public:
    typedef cv_state<Order> cv;

    void monitor_state(std::size_t, std::size_t,
        vsmc::ConstSingleParticle<cv> csp, double *res)
    {
        res[0] = csp.state(vsmc::Position<PosX>());
        res[1] = csp.state(vsmc::Position<PosY>());
    }
};

#endif // VSMC_EXAMPLE_PF_SMP_HPP
