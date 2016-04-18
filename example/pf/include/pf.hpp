//============================================================================
// vSMC/example/pf/include/pf.hpp
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

// clang-format off
#ifndef VSMC_EXAMPLE_PF_@SMP@_HPP
#define VSMC_EXAMPLE_PF_@SMP@_HPP
// clang-format on

#include <vsmc/core/sampler.hpp>
#include <vsmc/core/state_matrix.hpp>
#include <vsmc/rng/normal_distribution.hpp>
#include <vsmc/smp/backend_@smp@.hpp>
#include <vsmc/utility/stop_watch.hpp>
#if VSMC_HAS_HDF5
#include <vsmc/utility/hdf5io.hpp>
#endif

// clang-format off
template <typename T, typename Derived>
using InitializeSMP = vsmc::Initialize@SMP@<T, Derived>;

template <typename T, typename Derived>
using MoveSMP = vsmc::Move@SMP@<T, Derived>;

template <typename T, typename Derived>
using MonitorEvalSMP = vsmc::MonitorEval@SMP@<T, Derived>;
// clang-format on

static const std::size_t DataNum = 100;
static const std::size_t ParticleNum = 10000;
static const std::size_t PosX = 0;
static const std::size_t PosY = 1;
static const std::size_t VelX = 2;
static const std::size_t VelY = 3;
static const std::size_t LogL = 4;

template <vsmc::MatrixLayout Layout>
using StateBase = vsmc::StateMatrix<Layout, 5, double>;

template <vsmc::MatrixLayout Layout, typename RNGSetType>
class pf_state : public StateBase<Layout>
{
    public:
    using size_type = typename StateBase<Layout>::size_type;
    using rng_set_type = RNGSetType;

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

template <vsmc::MatrixLayout Layout, typename RNGSetType>
class pf_init : public InitializeSMP<pf_state<Layout, RNGSetType>,
                    pf_init<Layout, RNGSetType>>
{
    public:
    std::size_t eval_sp(
        vsmc::SingleParticle<pf_state<Layout, RNGSetType>> sp) const
    {
        const double sd_pos0 = 2;
        const double sd_vel0 = 1;
        vsmc::NormalDistribution<double> norm_pos(0, sd_pos0);
        vsmc::NormalDistribution<double> norm_vel(0, sd_vel0);

        auto &rng = sp.rng();
        sp.state(PosX) = norm_pos(rng);
        sp.state(PosY) = norm_pos(rng);
        sp.state(VelX) = norm_vel(rng);
        sp.state(VelY) = norm_vel(rng);
        sp.state(LogL) = sp.particle().value().log_likelihood(0, sp.id());

        return 1;
    }

    void eval_param(vsmc::Particle<pf_state<Layout, RNGSetType>> &particle,
        void *file) const
    {
        particle.value().read_data(static_cast<const char *>(file));
    }

    void eval_post(vsmc::Particle<pf_state<Layout, RNGSetType>> &particle)
    {
        w_.resize(particle.size());
        particle.value().read_state(LogL, w_.data());
        particle.weight().set_log(w_.data());
    }

    private:
    vsmc::Vector<double> w_;
};

template <vsmc::MatrixLayout Layout, typename RNGSetType>
class pf_move
    : public MoveSMP<pf_state<Layout, RNGSetType>, pf_move<Layout, RNGSetType>>
{
    public:
    std::size_t eval_sp(std::size_t iter,
        vsmc::SingleParticle<pf_state<Layout, RNGSetType>> sp) const
    {
        const double sd_pos = std::sqrt(0.02);
        const double sd_vel = std::sqrt(0.001);
        const double delta = 0.1;
        vsmc::NormalDistribution<double> norm_pos(0, sd_pos);
        vsmc::NormalDistribution<double> norm_vel(0, sd_vel);

        auto &rng = sp.rng();
        sp.state(PosX) += norm_pos(rng) + delta * sp.state(VelX);
        sp.state(PosY) += norm_pos(rng) + delta * sp.state(VelY);
        sp.state(VelX) += norm_vel(rng);
        sp.state(VelY) += norm_vel(rng);
        sp.state(LogL) = sp.particle().value().log_likelihood(iter, sp.id());

        return 1;
    }

    void eval_post(
        std::size_t, vsmc::Particle<pf_state<Layout, RNGSetType>> &particle)
    {
        w_.resize(particle.size());
        particle.value().read_state(LogL, w_.data());
        particle.weight().add_log(w_.data());
    }

    private:
    vsmc::Vector<double> w_;
};

template <vsmc::MatrixLayout Layout, typename RNGSetType>
class pf_eval : public MonitorEvalSMP<pf_state<Layout, RNGSetType>,
                    pf_eval<Layout, RNGSetType>>
{
    public:
    void eval_sp(std::size_t, std::size_t,
        vsmc::SingleParticle<pf_state<Layout, RNGSetType>> sp, double *res)
    {
        res[0] = sp.state(PosX);
        res[1] = sp.state(PosY);
    }
};

template <vsmc::MatrixLayout Layout, typename RNGSetType, typename CharT,
    typename Traits>
inline void pf_run(std::size_t N, vsmc::ResampleScheme scheme,
    const std::string &datafile, const std::string &implname,
    const std::string &res, const std::string &rc, const std::string &rs,
    bool store, std::basic_ostream<CharT, Traits> &os)
{
    std::string basename;
    std::string pf_h5;
    std::string pf_txt;
    if (store) {
        basename = "pf." + implname + "." + res + "." + rc + "." + rs;
        pf_h5 = basename + ".h5";
        pf_txt = basename + ".txt";
    }

    vsmc::Seed::instance().set(101);
    vsmc::Sampler<pf_state<Layout, RNGSetType>> sampler(N, scheme, 0.5);
    sampler.init(pf_init<Layout, RNGSetType>());
    sampler.move(pf_move<Layout, RNGSetType>(), false);
    sampler.monitor("pos", 2, pf_eval<Layout, RNGSetType>());
    sampler.monitor("pos").name(0) = "pos.x";
    sampler.monitor("pos").name(1) = "pos.y";

#if VSMC_HAS_HDF5
    if (store) {
        vsmc::hdf5store(pf_h5);
        vsmc::hdf5store(pf_h5, "Particle", true);
    }
#endif
    vsmc::StopWatch watch;
    watch.start();
    for (std::size_t i = 0; i != DataNum; ++i) {
        if (i == 0)
            sampler.initialize(const_cast<char *>(datafile.c_str()));
        else
            sampler.iterate();
#if VSMC_HAS_HDF5
        if (store) {
            std::stringstream ss;
            ss << "Iter." << i;
            vsmc::hdf5store(
                sampler.particle(), pf_h5, "Particle/" + ss.str(), true);
        }
#endif
    }
    watch.stop();
#if VSMC_HAS_HDF5
    if (store) {
        vsmc::hdf5store(sampler, pf_h5, "Sampler", true);
        vsmc::hdf5store(sampler.monitor("pos"), pf_h5, "Monitor", true);
    }
#endif
    if (store) {
        std::ofstream txt(pf_txt);
        txt << sampler << std::endl;
        txt.close();
    } else {
        os << std::setw(20) << std::left << N;
        os << std::setw(20) << std::left << implname;
        os << std::setw(20) << std::left << res;
        os << std::setw(20) << std::left << rc;
        os << std::setw(20) << std::left << rs;
        os << std::setw(20) << std::left << std::fixed << watch.milliseconds();
        os << std::endl;
    }
}

template <typename CharT, typename Traits>
inline void pf_run(std::size_t N, vsmc::ResampleScheme scheme,
    const std::string &datafile, const std::string &implname, bool store,
    std::basic_ostream<CharT, Traits> &os)
{
    std::string res;
    switch (scheme) {
        case vsmc::Multinomial: res = "Multinomial"; break;
        case vsmc::Stratified: res = "Stratified"; break;
        case vsmc::Systematic: res = "Systematic"; break;
        case vsmc::Residual: res = "Residual"; break;
        case vsmc::ResidualStratified: res = "ResidualStratified"; break;
        case vsmc::ResidualSystematic: res = "ResidualSystematic"; break;
    }
    pf_run<vsmc::RowMajor, vsmc::RNGSetVector<vsmc::RNG>>(
        N, scheme, datafile, implname, res, "RowMajor", "Vector", store, os);
    pf_run<vsmc::ColMajor, vsmc::RNGSetVector<vsmc::RNG>>(
        N, scheme, datafile, implname, res, "ColMajor", "Vector", store, os);
#if VSMC_HAS_TBB
    pf_run<vsmc::RowMajor, vsmc::RNGSetTBB<vsmc::RNG>>(
        N, scheme, datafile, implname, res, "RowMajor", "TBB", store, os);
    pf_run<vsmc::ColMajor, vsmc::RNGSetTBB<vsmc::RNG>>(
        N, scheme, datafile, implname, res, "ColMajor", "TBB", store, os);
#endif
}

template <typename CharT, typename Traits>
inline void pf_run(std::size_t N, const std::string &datafile,
    const std::string &implname, bool store,
    std::basic_ostream<CharT, Traits> &os)
{

    pf_run(N, vsmc::Multinomial, datafile, implname, store, os);
    pf_run(N, vsmc::Stratified, datafile, implname, store, os);
    pf_run(N, vsmc::Systematic, datafile, implname, store, os);
    pf_run(N, vsmc::Residual, datafile, implname, store, os);
    pf_run(N, vsmc::ResidualStratified, datafile, implname, store, os);
    pf_run(N, vsmc::ResidualSystematic, datafile, implname, store, os);
}

#endif // VSMC_EXAMPLE_PF_@SMP@_HPP
