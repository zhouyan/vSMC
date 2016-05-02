//============================================================================
// vSMC/example/pf/include/pf_cv.hpp
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

#ifndef VSMC_EXAMPLE_PFCV_HPP
#define VSMC_EXAMPLE_PFCV_HPP

#include "pf.hpp"

template <vsmc::MatrixLayout Layout>
using PFCVBase = vsmc::StateMatrix<Layout, 4, double>;

template <vsmc::MatrixLayout Layout, typename RNGSetType>
class PFCV : public PFCVBase<Layout>
{
    public:
    using rng_set_type = RNGSetType;

    using base = PFCVBase<Layout>;

    template <typename T>
    using base_sp = typename base::template single_particle_type<T>;

    template <typename T>
    class single_particle_type : public base_sp<T>
    {
        public:
        single_particle_type(std::size_t id, vsmc::Particle<T> *pptr)
            : base_sp<T>(id, pptr)
        {
        }

        double &pos_x() { return this->state(0); }
        double &pos_y() { return this->state(1); }
        double &vel_x() { return this->state(2); }
        double &vel_y() { return this->state(3); }

        double log_likelihood(std::size_t iter)
        {
            const double scale = 10;
            const double nu = 10;

            double llh_x =
                scale * (pos_x() - this->particle().value().obs_x_[iter]);
            double llh_y =
                scale * (pos_y() - this->particle().value().obs_y_[iter]);

            llh_x = std::log(1 + llh_x * llh_x / nu);
            llh_y = std::log(1 + llh_y * llh_y / nu);

            return -0.5 * (nu + 1) * (llh_x + llh_y);
        }
    }; // class single_particle_type

    PFCV(std::size_t N) : PFCVBase<Layout>(N) {}

    std::size_t n() const { return obs_x_.size(); }

    void initialize()
    {
        obs_x_.clear();
        obs_y_.clear();
        double x = 0;
        double y = 0;
        std::ifstream data("pf_cv.data");
        while (data >> x >> y) {
            obs_x_.push_back(x);
            obs_y_.push_back(y);
        }
        data.close();
    }

    private:
    vsmc::Vector<double> obs_x_;
    vsmc::Vector<double> obs_y_;
}; // class PFCV

template <vsmc::MatrixLayout Layout, typename RNGSetType, vsmc::SMPBackend SMP>
class PFCVInit : public vsmc::InitializeSMP<SMP, PFCV<Layout, RNGSetType>,
                     PFCVInit<Layout, RNGSetType, SMP>>
{
    public:
    void eval_pre(vsmc::Particle<PFCV<Layout, RNGSetType>> &particle)
    {
        particle.value().initialize();
        w_.resize(particle.size());
    }

    std::size_t eval_sp(vsmc::SingleParticle<PFCV<Layout, RNGSetType>> sp)
    {
        const double sd_pos0 = 2;
        const double sd_vel0 = 1;
        vsmc::NormalDistribution<double> normal_pos(0, sd_pos0);
        vsmc::NormalDistribution<double> normal_vel(0, sd_vel0);

        auto &rng = sp.rng();
        sp.pos_x() = normal_pos(rng);
        sp.pos_y() = normal_pos(rng);
        sp.vel_x() = normal_vel(rng);
        sp.vel_y() = normal_vel(rng);
        w_[sp.id()] = sp.log_likelihood(0);

        return 0;
    }

    void eval_post(vsmc::Particle<PFCV<Layout, RNGSetType>> &particle)
    {
        particle.weight().set_log(w_.data());
    }

    private:
    vsmc::Vector<double> w_;
}; // PFCVInit

template <vsmc::MatrixLayout Layout, typename RNGSetType, vsmc::SMPBackend SMP>
class PFCVMove : public vsmc::MoveSMP<SMP, PFCV<Layout, RNGSetType>,
                     PFCVMove<Layout, RNGSetType, SMP>>
{
    public:
    void eval_pre(
        std::size_t, vsmc::Particle<PFCV<Layout, RNGSetType>> &particle)
    {
        w_.resize(particle.size());
    }

    std::size_t eval_sp(
        std::size_t iter, vsmc::SingleParticle<PFCV<Layout, RNGSetType>> sp)
    {
        const double sd_pos = std::sqrt(0.02);
        const double sd_vel = std::sqrt(0.001);
        const double delta = 0.1;
        vsmc::NormalDistribution<double> normal_pos(0, sd_pos);
        vsmc::NormalDistribution<double> normal_vel(0, sd_vel);

        auto &rng = sp.rng();
        sp.pos_x() += normal_pos(rng) + delta * sp.vel_x();
        sp.pos_y() += normal_pos(rng) + delta * sp.vel_y();
        sp.vel_x() += normal_vel(rng);
        sp.vel_y() += normal_vel(rng);
        w_[sp.id()] = sp.log_likelihood(iter);

        return 0;
    }

    void eval_post(
        std::size_t, vsmc::Particle<PFCV<Layout, RNGSetType>> &particle)
    {
        particle.weight().add_log(w_.data());
    }

    private:
    vsmc::Vector<double> w_;
}; // class PFCVMove

template <vsmc::MatrixLayout Layout, typename RNGSetType, vsmc::SMPBackend SMP>
class PFCVEval : public vsmc::MonitorEvalSMP<SMP, PFCV<Layout, RNGSetType>,
                     PFCVEval<Layout, RNGSetType, SMP>>
{
    public:
    void eval_sp(std::size_t, std::size_t,
        vsmc::SingleParticle<PFCV<Layout, RNGSetType>> sp, double *res)
    {
        res[0] = sp.pos_x();
        res[1] = sp.pos_y();
    }
}; // class PFCVEval

template <vsmc::SMPBackend SMP, vsmc::ResampleScheme Scheme,
    vsmc::MatrixLayout Layout, typename RNGSetType>
inline void pf_cv_run(std::size_t N, int nwid, int twid)
{
    vsmc::Seed::instance().set(101);
    vsmc::Sampler<PFCV<Layout, RNGSetType>> sampler(N, Scheme, 0.5);
    sampler.init(PFCVInit<Layout, RNGSetType, SMP>());
    sampler.move(PFCVMove<Layout, RNGSetType, SMP>(), false);
    sampler.monitor("pos", 2, PFCVEval<Layout, RNGSetType, SMP>());
    sampler.monitor("pos").name(0) = "pos.x";
    sampler.monitor("pos").name(1) = "pos.y";
    sampler.initialize();

    const std::size_t n = sampler.particle().value().n();
    const std::string smp(pf_smp_name<SMP>());
    const std::string res(pf_res_name<Scheme>());
    const std::string rc(pf_rc_name<Layout>());
    const std::string rs(pf_rs_name<RNGSetType>());

    std::string basename = "pf_cv." + smp + "." + res + "." + rc + "." + rs;

#if VSMC_HAS_HDF5
    std::string h5file = basename + ".h5";
    vsmc::hdf5store(h5file);
    vsmc::hdf5store(h5file, "Particle", true);
    vsmc::hdf5store(sampler.particle(), h5file, "Particle/Iter.0", true);
#endif

    vsmc::StopWatch watch;
    for (std::size_t i = 1; i <= n; ++i) {
        watch.start();
        sampler.iterate();
        watch.stop();
#if VSMC_HAS_HDF5
        vsmc::hdf5store(sampler.particle(), h5file,
            "Particle/Iter." + std::to_string(i), true);
#endif
    }

#if VSMC_HAS_HDF5
    vsmc::hdf5store(sampler, h5file, "Sampler", true);
    vsmc::hdf5store(sampler.monitor("pos"), h5file, "Monitor", true);
#endif
    std::ofstream txt(basename + ".txt");
    txt << sampler << std::endl;
    txt.close();

    vsmc::Vector<double> rx(n);
    vsmc::Vector<double> ry(n);
    sampler.monitor("pos").read_record(0, rx.data());
    sampler.monitor("pos").read_record(1, ry.data());

    vsmc::Vector<double> tx;
    vsmc::Vector<double> ty;
    double x = 0;
    double y = 0;
    std::ifstream truth("pf_cv.truth");
    while (truth >> x >> y) {
        tx.push_back(x);
        ty.push_back(y);
    }
    vsmc::sub(n, tx.data(), rx.data(), rx.data());
    vsmc::sub(n, ty.data(), ry.data(), ry.data());
    vsmc::sqr(n, rx.data(), rx.data());
    vsmc::sqr(n, ry.data(), ry.data());
    vsmc::add(n, rx.data(), ry.data(), rx.data());
    vsmc::sqrt(n, rx.data(), rx.data());
    double error = std::accumulate(rx.begin(), rx.end(), 0.0) / n;
    double time = watch.seconds();

    std::cout << std::setw(nwid) << std::left << N;
    std::cout << std::setw(twid) << std::left << smp;
    std::cout << std::setw(twid + 5) << std::left << res;
    std::cout << std::setw(twid) << std::left << rc;
    std::cout << std::setw(twid) << std::left << rs;
    std::cout << std::setw(twid) << std::right << std::fixed << error;
    std::cout << std::setw(twid) << std::right << std::fixed << time;
    std::cout << std::endl;
}

template <vsmc::SMPBackend SMP, vsmc::ResampleScheme Scheme,
    vsmc::MatrixLayout Layout>
inline void pf_cv_run(std::size_t N, int nwid, int twid)
{
    pf_cv_run<SMP, Scheme, Layout, vsmc::RNGSetVector<>>(N, nwid, twid);
#if VSMC_HAS_TBB
    pf_cv_run<SMP, Scheme, Layout, vsmc::RNGSetTBB<>>(N, nwid, twid);
#endif
}

template <vsmc::SMPBackend SMP, vsmc::ResampleScheme Scheme>
inline void pf_cv_run(std::size_t N, int nwid, int twid)
{
    pf_cv_run<SMP, Scheme, vsmc::RowMajor>(N, nwid, twid);
    pf_cv_run<SMP, Scheme, vsmc::ColMajor>(N, nwid, twid);
}

template <vsmc::SMPBackend SMP>
inline void pf_cv_run(std::size_t N, int nwid, int twid)
{
    pf_cv_run<SMP, vsmc::Multinomial>(N, nwid, twid);
    pf_cv_run<SMP, vsmc::Residual>(N, nwid, twid);
    pf_cv_run<SMP, vsmc::ResidualStratified>(N, nwid, twid);
    pf_cv_run<SMP, vsmc::ResidualSystematic>(N, nwid, twid);
    pf_cv_run<SMP, vsmc::Stratified>(N, nwid, twid);
    pf_cv_run<SMP, vsmc::Systematic>(N, nwid, twid);
}

inline void pf_cv_run(std::size_t N, int nwid, int twid)
{
    pf_cv_run<vsmc::SEQ>(N, nwid, twid);
#if VSMC_HAS_OMP
    pf_cv_run<vsmc::OMP>(N, nwid, twid);
#endif
#if VSMC_HAS_TBB
    pf_cv_run<vsmc::TBB>(N, nwid, twid);
#endif
}

inline void pf_cv(std::size_t N)
{
    const int nwid = 10;
    const int twid = 15;
    const std::size_t lwid = nwid + twid * 6 + 5;

    std::cout << std::string(lwid, '=') << std::endl;
    std::cout << std::setw(nwid) << std::left << "N";
    std::cout << std::setw(twid) << std::left << "SMPBackend";
    std::cout << std::setw(twid + 5) << std::left << "ResampleScheme";
    std::cout << std::setw(twid) << std::left << "MatrixLayout";
    std::cout << std::setw(twid) << std::left << "rng_set_type";
    std::cout << std::setw(twid) << std::right << "Error";
    std::cout << std::setw(twid) << std::right << "Time (s)";
    std::cout << std::endl;
    std::cout << std::string(lwid, '-') << std::endl;
    pf_cv_run(N, nwid, twid);
    std::cout << std::string(lwid, '-') << std::endl;
}

#endif // VSMC_EXAMPLE_PFCV_HPP
