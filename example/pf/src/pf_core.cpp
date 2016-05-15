//============================================================================
// vSMC/example/pf/src/pf_core.hpp
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

#include <vsmc/vsmc.hpp>

using namespace vsmc;

using PFBase = StateMatrix<RowMajor, 4, double>;

template <typename T>
using PFIndexBase = PFBase::particle_index_type<T>;

class PF : public PFBase
{
    public:
    template <typename T>
    class particle_index_type : public PFIndexBase<T>
    {
        public:
        particle_index_type(std::size_t i, Particle<T> *pptr)
            : PFIndexBase<T>(i, pptr)
        {
        }

        double &pos_x() { return this->at(0); }
        double &pos_y() { return this->at(1); }
        double &vel_x() { return this->at(2); }
        double &vel_y() { return this->at(3); }

        double log_likelihood(std::size_t iter)
        {
            const double x = this->particle().state().obs_x_[iter];
            const double y = this->particle().state().obs_y_[iter];
            const double scale = 10;
            const double nu = 10;

            double llh_x = scale * (pos_x() - x);
            double llh_y = scale * (pos_y() - y);
            llh_x = std::log(1 + llh_x * llh_x / nu);
            llh_y = std::log(1 + llh_y * llh_y / nu);

            return -0.5 * (nu + 1) * (llh_x + llh_y);
        }
    }; // class particle_index_type

    PF(std::size_t N) : PFBase(N)
    {
        double x = 0;
        double y = 0;
        std::ifstream data("pf_cv.data");
        while (data >> x >> y) {
            obs_x_.push_back(x);
            obs_y_.push_back(y);
        }
        data.close();
    }

    std::size_t n() const { return obs_x_.size(); }

    private:
    Vector<double> obs_x_;
    Vector<double> obs_y_;
}; // class PF

inline void PFInit(std::size_t, Particle<PF> &particle)
{
    std::normal_distribution<double> rpos(0, 2);
    std::normal_distribution<double> rvel(0, 1);
    auto &rng = particle.rng();

    for (auto idx : particle) {
        idx.pos_x() = rpos(rng);
        idx.pos_y() = rpos(rng);
        idx.vel_x() = rvel(rng);
        idx.vel_y() = rvel(rng);
    }
}

inline void PFMove(std::size_t, Particle<PF> &particle)
{
    std::normal_distribution<double> rpos(0, std::sqrt(0.02));
    std::normal_distribution<double> rvel(0, std::sqrt(0.001));
    auto &rng = particle.rng();
    const double delta = 0.1;

    for (auto idx : particle) {
        idx.pos_x() += rpos(rng) + delta * idx.vel_x();
        idx.pos_y() += rpos(rng) + delta * idx.vel_y();
        idx.vel_x() += rvel(rng);
        idx.vel_y() += rvel(rng);
    }
}

inline void PFWeight(std::size_t iter, Particle<PF> &particle)
{
    Vector<double> weight(particle.size());
    std::transform(particle.begin(), particle.end(), weight.begin(),
        [iter](ParticleIndex<PF> idx) { return idx.log_likelihood(iter); });
    particle.weight().add_log(weight.data());
}

inline void PFEstimate(
    std::size_t, std::size_t, Particle<PF> &particle, double *r)
{
    for (auto idx : particle) {
        *r++ = idx.pos_x();
        *r++ = idx.pos_y();
    }
}

int main()
{
    const std::size_t N = 1000;

    Sampler<PF> sampler(N);

    sampler.resample_method(Stratified, 0.5)
        .eval(PFInit, SamplerInit)
        .eval(PFMove, SamplerMove)
        .eval(PFWeight, SamplerInit | SamplerMove)
        .monitor("pos", Monitor<PF>(2, PFEstimate));

    sampler.initialize().iterate(sampler.particle().state().n() - 1);

    std::ofstream out("pf.out");
    out << sampler;
    out.close();

    return 0;
}
