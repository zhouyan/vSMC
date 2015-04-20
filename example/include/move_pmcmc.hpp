//============================================================================
// vSMC/example/include/move_pmcmc.hpp
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

#ifndef VSMC_EXAMPLE_MOVE_PMCMC_HPP
#define VSMC_EXAMPLE_MOVE_PMCMC_HPP

template <typename T, typename Alpha, typename Proposal, typename InitType>
class init_pmcmc
    : public BASE_INIT<T, init_pmcmc<T, Alpha, Proposal, InitType>>
{
    public:
    typedef Alpha alpha_type;
    typedef Proposal proposal_type;

    init_pmcmc() : alpha_(0), proposal_(0) {}

    void pre_processor(vsmc::Particle<T> &particle)
    {
        init_.pre_processor(particle);
        alpha_.alpha_init(particle);
        proposal_.proposal_init(particle);
        const std::size_t nchain = static_cast<std::size_t>(particle.size());
        for (std::size_t i = 1; i != nchain; ++i) {
            particle.value().state(i, 0).alpha_inc() =
                particle.value().state(i, 0).alpha() -
                particle.value().state(i - 1, 0).alpha();
        }
    }

    void initialize_param(vsmc::Particle<T> &particle, void *info)
    {
        init_.initialize_param(particle, info);
    }

    std::size_t initialize_state(vsmc::SingleParticle<T> part)
    {
        return init_.initialize_state(part);
    }

    private:
    Alpha alpha_;
    Proposal proposal_;
    InitType init_;
};

template <typename T, typename MoveType>
class pmcmc_local_serial : public MoveType
{
    public:
    typedef typename vsmc::Particle<T>::size_type size_type;

    pmcmc_local_serial() : size_(0) {}

    std::size_t operator()(std::size_t iter, vsmc::Particle<T> &particle)
    {
        if (size_ != particle.size()) {
            size_ = particle.size();
            sample_ = std::uniform_int_distribution<size_type>(0, size_ - 1);
        }

        size_type c = sample_(particle.rng(0));
        return this->move_state(iter, vsmc::SingleParticle<T>(c, &particle));
    }

    private:
    size_type size_;
    std::uniform_int_distribution<size_type> sample_;
};

template <typename MoveType>
class pmcmc_local_parallel : public MoveType
{
};

template <typename T>
class pmcmc_global
{
    public:
    typedef typename vsmc::Particle<T>::size_type size_type;

    pmcmc_global() : size_(0), runif_(0, 1) {}

    std::size_t operator()(std::size_t, vsmc::Particle<T> &particle)
    {
        using std::log;

        if (size_ != particle.size()) {
            size_ = particle.size();
            sample_ = std::uniform_int_distribution<size_type>(0, size_ - 2);
        }

        size_type c1 = sample_(particle.rng(0));
        size_type c2 = c1 + 1;

        // save old values
        particle.value().state(c1, 0).save_old();
        particle.value().state(c2, 0).save_old();

        // propose swap
        particle.value().state(c1, 0).swap(particle.value().state(c2, 0));
        particle.value().log_target(particle.value().state(c1, 0));
        particle.value().log_target(particle.value().state(c2, 0));
        double p = particle.value().state(c1, 0).log_target_diff() +
            particle.value().state(c2, 0).log_target_diff();
        double u = log(runif_(particle.rng(1)));
        if (p < u)
            particle.value().state(c1, 0).swap(particle.value().state(c2, 0));

        particle.value().state(c1, 0).mh_reject_common(p, u);
        particle.value().state(c2, 0).mh_reject_common(p, u);

        return p < u ? 0 : 1;
    }

    private:
    size_type size_;
    std::uniform_int_distribution<size_type> sample_;
    std::uniform_real_distribution<> runif_;
};

template <typename T>
class alpha_pair
{
    public:
    typedef double *value_type;
    alpha_pair(value_type config = 0) : pair_(config) {}
    void alpha_iter(std::size_t, vsmc::Particle<T> &) {}

    void alpha_init(vsmc::Particle<T> &particle)
    {
        if (pair_) {
            particle.value().state(0, 0).alpha() = pair_[0];
            particle.value().state(1, 0).alpha() = pair_[1];
        }
    }

    private:
    double *pair_;
};

template <typename T, typename Proposal, typename InitType>
inline std::vector<double> optimal_pmcmc_alpha(vsmc::Sampler<T> &sampler,
    std::size_t burnin = 1000, std::size_t iter_num = 1000)
{
    using std::exp;
    using std::sqrt;

    if (sampler.size() != 2)
        throw std::runtime_error("Use two chains to find optimal alpha");

    const std::size_t max_attempt = 100;
    const double min_alpha = 1e-6;

    std::vector<double> alpha;
    double a = 1;
    double b = 0;

    while (a > min_alpha) {
        alpha.push_back(a);
        double rho = 2;
        double accept = 0;
        std::size_t attempt = 1;
        while (attempt < max_attempt && (accept < 0.23 || accept > 0.24)) {
            b = a / (1 + exp(rho));
            if (b < min_alpha)
                break;
            double pair[2] = {a, b};
            sampler.init(
                init_pmcmc<T, alpha_pair<T>, Proposal, InitType>(pair));
            sampler.initialize();
            sampler.iterate(burnin);
            accept = 0;
            for (std::size_t i = 0; i != iter_num; ++i) {
                sampler.iterate();
                std::size_t in = sampler.iter_size();
                std::size_t im = sampler.move_num(in - 1);
                accept += sampler.accept_history(in - 1, im - 1);
            }
            accept /= iter_num;
            rho += (accept - 0.235) / sqrt(static_cast<double>(attempt));
            ++attempt;
        }
        if (accept > 0.5)
            break;
        else
            a = b;
    }
    alpha.push_back(0);
    std::reverse(alpha.begin(), alpha.end());

    return alpha;
}

#endif // VSMC_EXAMPLE_MOVE_PMCMC_HPP
