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
    using base_idx = typename base::template particle_index_type<T>;

    template <typename T>
    class particle_index_type : public base_idx<T>
    {
        public:
        particle_index_type(std::size_t id, vsmc::Particle<T> *pptr)
            : base_idx<T>(id, pptr)
        {
        }

        double &pos_x() { return this->at(0); }
        double &pos_y() { return this->at(1); }
        double &vel_x() { return this->at(2); }
        double &vel_y() { return this->at(3); }

        double log_likelihood(std::size_t iter)
        {
            const double scale = 10;
            const double nu = 10;

            double llh_x =
                scale * (pos_x() - this->particle().state().obs_x(iter));
            double llh_y =
                scale * (pos_y() - this->particle().state().obs_y(iter));

            llh_x = std::log(1 + llh_x * llh_x / nu);
            llh_y = std::log(1 + llh_y * llh_y / nu);

            return -0.5 * (nu + 1) * (llh_x + llh_y);
        }
    }; // class particle_index_type

    PFCV(std::size_t N) : PFCVBase<Layout>(N) {}

    std::size_t n() const { return obs_x_.size(); }

    double obs_x(std::size_t iter) { return obs_x_[iter]; }
    double obs_y(std::size_t iter) { return obs_y_[iter]; }

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

template <typename Backend, vsmc::MatrixLayout Layout, typename RNGSetType>
class PFCVInit : public vsmc::SamplerEvalSMP<PFCV<Layout, RNGSetType>,
                     PFCVInit<Backend, Layout, RNGSetType>, Backend>
{
    public:
    using T = PFCV<Layout, RNGSetType>;

    void eval_pre(std::size_t, vsmc::Particle<T> &particle)
    {
        particle.state().initialize();
    }

    std::size_t eval(std::size_t, vsmc::ParticleIndex<T> idx)
    {
        const double sd_pos0 = 2;
        const double sd_vel0 = 1;
        vsmc::NormalDistribution<double> normal_pos(0, sd_pos0);
        vsmc::NormalDistribution<double> normal_vel(0, sd_vel0);

        auto &rng = idx.rng();
        idx.pos_x() = normal_pos(rng);
        idx.pos_y() = normal_pos(rng);
        idx.vel_x() = normal_vel(rng);
        idx.vel_y() = normal_vel(rng);

        return 0;
    }
}; // PFCVInit

template <typename Backend, vsmc::MatrixLayout Layout, typename RNGSetType>
class PFCVMove : public vsmc::SamplerEvalSMP<PFCV<Layout, RNGSetType>,
                     PFCVMove<Backend, Layout, RNGSetType>, Backend>
{
    public:
    using T = PFCV<Layout, RNGSetType>;

    std::size_t eval(std::size_t, vsmc::ParticleIndex<T> idx)
    {
        const double sd_pos = std::sqrt(0.02);
        const double sd_vel = std::sqrt(0.001);
        const double delta = 0.1;
        vsmc::NormalDistribution<double> normal_pos(0, sd_pos);
        vsmc::NormalDistribution<double> normal_vel(0, sd_vel);

        auto &rng = idx.rng();
        idx.pos_x() += normal_pos(rng) + delta * idx.vel_x();
        idx.pos_y() += normal_pos(rng) + delta * idx.vel_y();
        idx.vel_x() += normal_vel(rng);
        idx.vel_y() += normal_vel(rng);

        return 0;
    }
}; // class PFCVMove

template <typename Backend, typename RNGSetType>
class PFCVMove<Backend, vsmc::ColMajor, RNGSetType>
    : public vsmc::SamplerEvalSMP<PFCV<vsmc::ColMajor, RNGSetType>,
          PFCVMove<Backend, vsmc::ColMajor, RNGSetType>, Backend>
{
    public:
    using T = PFCV<vsmc::ColMajor, RNGSetType>;

#if VSMC_HAS_TBB
    std::size_t operator()(std::size_t iter,
        vsmc::Particle<PFCV<vsmc::ColMajor, RNGSetType>> &particle)
    {
        return run_dispatch(
            iter, particle, std::is_same<Backend, vsmc::BackendTBB>());
    }
#endif

    void eval_pre(std::size_t, vsmc::Particle<T> &particle)
    {
        w_.resize(particle.size());
        v_.resize(particle.size());
    }

    std::size_t eval_range(std::size_t, const vsmc::ParticleRange<T> &range)
    {
        const double sd_pos = std::sqrt(0.02);
        const double sd_vel = std::sqrt(0.001);
        const double delta = 0.1;
        vsmc::NormalDistribution<double> normal_pos(0, sd_pos);
        vsmc::NormalDistribution<double> normal_vel(0, sd_vel);

        double *const pos_x =
            range.particle().state().col_data(0) + range.first();
        double *const pos_y =
            range.particle().state().col_data(1) + range.first();
        double *const vel_x =
            range.particle().state().col_data(2) + range.first();
        double *const vel_y =
            range.particle().state().col_data(3) + range.first();
        double *const w = w_.data() + range.first();
        double *const v = v_.data() + range.first();

        const std::size_t n = range.size();
        auto &rng = range.begin().rng();
        normal_pos(rng, n, w);
        normal_pos(rng, n, v);
        vsmc::add(n, w, pos_x, pos_x);
        vsmc::add(n, v, pos_y, pos_y);
        vsmc::fma(n, delta, vel_x, pos_x, pos_x);
        vsmc::fma(n, delta, vel_y, pos_y, pos_y);
        normal_vel(rng, n, w);
        normal_vel(rng, n, v);
        vsmc::add(n, w, vel_x, vel_x);
        vsmc::add(n, v, vel_y, vel_y);

        return 0;
    }

    private:
    vsmc::Vector<double> w_;
    vsmc::Vector<double> v_;

#if VSMC_HAS_TBB
    std::size_t run_dispatch(
        std::size_t iter, vsmc::Particle<T> &particle, std::true_type)
    {
        return this->run(iter, particle, 512);
    }

    std::size_t run_dispatch(
        std::size_t iter, vsmc::Particle<T> &particle, std::false_type)
    {
        return vsmc::SamplerEvalSMP<T,
            PFCVMove<Backend, vsmc::ColMajor, RNGSetType>,
            Backend>::operator()(iter, particle);
    }
#endif
}; // class PFCVMove

template <typename Backend, vsmc::MatrixLayout Layout, typename RNGSetType>
class PFCVWeight : public vsmc::SamplerEvalSMP<PFCV<Layout, RNGSetType>,
                       PFCVWeight<Backend, Layout, RNGSetType>, Backend>
{
    public:
    using T = PFCV<Layout, RNGSetType>;

    void eval_pre(std::size_t, vsmc::Particle<T> &particle)
    {
        w_.resize(particle.size());
    }

    std::size_t eval(std::size_t iter, vsmc::ParticleIndex<T> idx)
    {
        w_[idx.id()] = idx.log_likelihood(iter);

        return 0;
    }

    void eval_post(std::size_t, vsmc::Particle<T> &particle)
    {
        particle.weight().add_log(w_.data());
    }

    private:
    vsmc::Vector<double> w_;
}; // class PFCVWeight

template <typename Backend, vsmc::MatrixLayout Layout, typename RNGSetType>
class PFCVEval : public vsmc::MonitorEvalSMP<PFCV<Layout, RNGSetType>,
                     PFCVEval<Backend, Layout, RNGSetType>, Backend>
{
    public:
    void eval(std::size_t, std::size_t,
        vsmc::ParticleIndex<PFCV<Layout, RNGSetType>> idx, double *res)
    {
        res[0] = idx.pos_x();
        res[1] = idx.pos_y();
    }
}; // class PFCVEval

template <typename Backend, vsmc::ResampleScheme Scheme,
    vsmc::MatrixLayout Layout, typename RNGSetType>
inline void pf_cv_run(std::size_t N, int nwid, int twid)
{
    vsmc::Seed::instance().set(101);
    vsmc::Sampler<PFCV<Layout, RNGSetType>> sampler(N);
    sampler.resample_method(Scheme, 0.5);
    sampler.eval(PFCVInit<Backend, Layout, RNGSetType>(), vsmc::SamplerInit);
    sampler.eval(PFCVMove<Backend, Layout, RNGSetType>(), vsmc::SamplerMove);
    sampler.eval(PFCVWeight<Backend, Layout, RNGSetType>(),
        vsmc::SamplerInit | vsmc::SamplerMove);
    sampler.monitor("pos", vsmc::Monitor<PFCV<Layout, RNGSetType>>(
                               2, PFCVEval<Backend, Layout, RNGSetType>()));
    sampler.monitor("pos").name(0) = "pos.x";
    sampler.monitor("pos").name(1) = "pos.y";
    sampler.initialize();

    const std::size_t n = sampler.particle().state().n();
    const std::string smp(pf_backend_name<Backend>());
    const std::string res(pf_scheme_name<Scheme>());
    const std::string rc(pf_layout_name<Layout>());
    const std::string rs(pf_rng_set_name<RNGSetType>());

    std::string basename = "pf_cv." + smp + "." + res + "." + rc + "." + rs;

#if VSMC_HAS_HDF5
    std::string h5file = basename + ".h5";
    vsmc::hdf5store(h5file);
    vsmc::hdf5store(h5file, "Particle", true);
    vsmc::hdf5store(sampler.particle(), h5file, "Particle/Iter.0", true);
#endif

    vsmc::StopWatch watch;
    for (std::size_t i = 1; i < n; ++i) {
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

template <typename Backend, vsmc::ResampleScheme Scheme,
    vsmc::MatrixLayout Layout>
inline void pf_cv_run(std::size_t N, int nwid, int twid)
{
    pf_cv_run<Backend, Scheme, Layout, vsmc::RNGSetVector<>>(N, nwid, twid);
#if VSMC_HAS_TBB
    pf_cv_run<Backend, Scheme, Layout, vsmc::RNGSetTBB<>>(N, nwid, twid);
#endif
}

template <typename Backend, vsmc::ResampleScheme Scheme>
inline void pf_cv_run(std::size_t N, int nwid, int twid)
{
    pf_cv_run<Backend, Scheme, vsmc::RowMajor>(N, nwid, twid);
    pf_cv_run<Backend, Scheme, vsmc::ColMajor>(N, nwid, twid);
}

template <typename Backend>
inline void pf_cv_run(std::size_t N, int nwid, int twid)
{
    pf_cv_run<Backend, vsmc::Multinomial>(N, nwid, twid);
    pf_cv_run<Backend, vsmc::Residual>(N, nwid, twid);
    pf_cv_run<Backend, vsmc::ResidualStratified>(N, nwid, twid);
    pf_cv_run<Backend, vsmc::ResidualSystematic>(N, nwid, twid);
    pf_cv_run<Backend, vsmc::Stratified>(N, nwid, twid);
    pf_cv_run<Backend, vsmc::Systematic>(N, nwid, twid);
}

inline void pf_cv_run(std::size_t N, int nwid, int twid)
{
    pf_cv_run<vsmc::BackendSEQ>(N, nwid, twid);
    pf_cv_run<vsmc::BackendSTD>(N, nwid, twid);
#if VSMC_HAS_OMP
    pf_cv_run<vsmc::BackendOMP>(N, nwid, twid);
#endif
#if VSMC_HAS_TBB
    pf_cv_run<vsmc::BackendTBB>(N, nwid, twid);
#endif
}

inline void pf_cv(std::size_t N)
{
    const int nwid = 10;
    const int twid = 15;
    const std::size_t lwid = nwid + twid * 6 + 5;

    std::cout << std::string(lwid, '=') << std::endl;
    std::cout << std::setw(nwid) << std::left << "N";
    std::cout << std::setw(twid) << std::left << "Backend";
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
