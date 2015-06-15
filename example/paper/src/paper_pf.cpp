//============================================================================
// vSMC/example/paper/src/paper_pf.cpp
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

#include <cassert>
#include <cmath>
#include <fstream>
#include <vsmc/core/sampler.hpp>
#include <vsmc/core/state_matrix.hpp>
#include <vsmc/smp/backend_seq.hpp>

const std::size_t DataNum = 100;
const std::size_t ParticleNum = 1000;
const std::size_t Dim = 4;

class cv : public vsmc::StateMatrix<vsmc::RowMajor, Dim, double>
{
    public:
    cv(size_type N)
        : vsmc::StateMatrix<vsmc::RowMajor, Dim, double>(N)
        , x_obs_(DataNum)
        , y_obs_(DataNum)
    {
    }

    double log_likelihood(std::size_t iter, size_type id) const
    {
        const double scale = 10;
        const double nu = 10;
        double llh_x = scale * (state(id, 0) - x_obs_[iter]);
        double llh_y = scale * (state(id, 1) - y_obs_[iter]);
        llh_x = std::log(1 + llh_x * llh_x / nu);
        llh_y = std::log(1 + llh_y * llh_y / nu);

        return -0.5 * (nu + 1) * (llh_x + llh_y);
    }

    void read_data(const char *filename)
    {
        std::ifstream data(filename);
        for (std::size_t i = 0; i != DataNum; ++i)
            data >> x_obs_[i] >> y_obs_[i];
        data.close();
        data.clear();
    }

    private:
    vsmc::Vector<double> x_obs_;
    vsmc::Vector<double> y_obs_;
};

inline std::size_t cv_init(vsmc::Particle<cv> &particle, void *filename)
{
    if (filename)
        particle.value().read_data(static_cast<const char *>(filename));

    const double sd_pos0 = 2;
    const double sd_vel0 = 1;
    std::normal_distribution<double> norm_pos(0, sd_pos0);
    std::normal_distribution<double> norm_vel(0, sd_vel0);
    vsmc::Vector<double> w(particle.size());

    for (vsmc::Particle<cv>::size_type i = 0; i != particle.size(); ++i) {
        particle.value().state(i, 0) = norm_pos(particle.rng(i));
        particle.value().state(i, 1) = norm_pos(particle.rng(i));
        particle.value().state(i, 2) = norm_vel(particle.rng(i));
        particle.value().state(i, 3) = norm_vel(particle.rng(i));
        w[i] = particle.value().log_likelihood(0, i);
    }
    particle.weight().set_log(w.begin());

    return 0;
}

class cv_move : public vsmc::MoveSEQ<cv>
{
    public:
    void eval_pre(std::size_t, vsmc::Particle<cv> &particle)
    {
        w_.resize(particle.size());
    }

    std::size_t eval_sp(std::size_t iter, vsmc::SingleParticle<cv> sp)
    {
        const double sd_pos = std::sqrt(0.02);
        const double sd_vel = std::sqrt(0.001);
        const double delta = 0.1;
        std::normal_distribution<double> norm_pos(0, sd_pos);
        std::normal_distribution<double> norm_vel(0, sd_vel);

        sp.state(0) += norm_pos(sp.rng()) + delta * sp.state(2);
        sp.state(1) += norm_pos(sp.rng()) + delta * sp.state(3);
        sp.state(2) += norm_vel(sp.rng());
        sp.state(3) += norm_vel(sp.rng());
        w_[sp.id()] = sp.particle().value().log_likelihood(iter, sp.id());

        return 0;
    }

    void eval_post(std::size_t, vsmc::Particle<cv> &particle)
    {
        particle.weight().add_log(w_.begin());
    }

    private:
    vsmc::Vector<double> w_;
};

class cv_est : public vsmc::MonitorEvalSEQ<cv>
{
    public:
    void eval_sp(
        std::size_t, std::size_t dim, vsmc::SingleParticle<cv> sp, double *res)
    {
        assert(dim <= Dim);
        for (std::size_t d = 0; d != dim; ++d)
            res[d] = sp.state(d);
    }
};

int main()
{
    vsmc::Sampler<cv> sampler(ParticleNum, vsmc::Stratified, 0.5);
    sampler.init(cv_init).move(cv_move(), false).monitor("pos", 2, cv_est());

    char data_file[] = "paper_pf.data";
    sampler.initialize(data_file);
    sampler.iterate(DataNum - 1);

    std::ofstream est("paper_pf.est");
    est << sampler << std::endl;
    est.close();
    est.clear();

    return 0;
}
