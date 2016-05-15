//============================================================================
// vSMC/example/pf/src/pf_smp.hpp
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

class PFInit : public SamplerEvalSMP<PF, PFInit>
{
    public:
    void eval(std::size_t, ParticleIndex<PF> idx)
    {
        std::normal_distribution<double> rpos(0, 2);
        std::normal_distribution<double> rvel(0, 1);
        auto &rng = idx.rng();

        idx.pos_x() = rpos(rng);
        idx.pos_y() = rpos(rng);
        idx.vel_x() = rvel(rng);
        idx.vel_y() = rvel(rng);
    }
}; // class PFInit

class PFMove : public SamplerEvalSMP<PF, PFMove>
{
    public:
    void eval(std::size_t, ParticleIndex<PF> idx)
    {
        std::normal_distribution<double> rpos(0, std::sqrt(0.02));
        std::normal_distribution<double> rvel(0, std::sqrt(0.001));
        auto &rng = idx.rng();
        const double delta = 0.1;

        idx.pos_x() += rpos(rng) + delta * idx.vel_x();
        idx.pos_y() += rpos(rng) + delta * idx.vel_y();
        idx.vel_x() += rvel(rng);
        idx.vel_y() += rvel(rng);
    }
}; // class PFMove

class PFWeight : public SamplerEvalSMP<PF, PFWeight>
{
    public:
    void eval(std::size_t iter, ParticleIndex<PF> idx)
    {
        weight_[idx.id()] = idx.log_likelihood(iter);
    }

    void eval_pre(std::size_t, Particle<PF> &particle)
    {
        weight_.resize(particle.size());
    }

    void eval_post(std::size_t, Particle<PF> &particle)
    {
        particle.weight().add_log(weight_.data());
    }

    private:
    Vector<double> weight_;
}; // class PFWeight

class PFEstimate : public MonitorEvalSMP<PF, PFEstimate>
{
    public:
    void eval(std::size_t, std::size_t, ParticleIndex<PF> idx, double *r)
    {
        *r++ = idx.pos_x();
        *r++ = idx.pos_y();
    }
}; // class PFEstimate

int main()
{
    const std::size_t N = 1000;

    Sampler<PF> sampler(N);
    sampler.eval(PFInit(), SamplerInit);
    sampler.eval(PFMove(), SamplerMove);
    sampler.eval(PFWeight(), SamplerInit | SamplerMove);
    sampler.monitor("pos", Monitor<PF>(2, PFEstimate()));

    sampler.initialize().iterate(sampler.particle().state().n() - 1);

    std::ofstream out("pf.out");
    out << sampler;
    out.close();

    return 0;
}
