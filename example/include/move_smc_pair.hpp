//============================================================================
// vSMC/example/include/move_smc_pair.hpp
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

#ifndef VSMC_EXAMPLE_MOVE_SMC_PAIR_HPP
#define VSMC_EXAMPLE_MOVE_SMC_PAIR_HPP

template <typename T, typename Alpha> class move_smc_pair
{
    public:
    typedef Alpha alpha_type;

    move_smc_pair(std::size_t min_comp, std::size_t max_comp,
        typename Alpha::value_type alpha_config = 0)
        : min_comp_(min_comp),
          max_comp_(max_comp),
          iter_(0),
          alpha_(alpha_config),
          dummy_particle_(1)
    {
    }

    std::size_t operator()(std::size_t iter, vsmc::Particle<T> &particle)
    {
        using std::log;

        if (particle.value().state(0, 0).beta() <= 0) {
            dummy_particle_.value().alpha(0);
            dummy_particle_.value().alpha_inc(0);
            particle.value().zconst() = 0;
            iter_ = iter - 1;
        }
        alpha_.alpha_iter(iter - iter_, dummy_particle_);
        particle.value().beta(dummy_particle_.value().state(0, 0).alpha());
        exp_weight_.resize(particle.size());

        double beta = particle.value().state(0, 0).beta();
        double beta_inc = particle.value().state(0, 0).beta_inc();
        double min_comp_inc = (1 - beta) / (1 - beta + beta_inc);
        double max_comp_inc = beta / (beta - beta_inc);
        for (typename vsmc::Particle<T>::size_type i = 0;
             i != particle.size(); ++i) {
            if (particle.value().state(i, 0).comp_num() == min_comp_)
                exp_weight_[i] = min_comp_inc;
            else if (particle.value().state(i, 0).comp_num() == max_comp_)
                exp_weight_[i] = max_comp_inc;
            else
                throw std::runtime_error("Invalid component number");
        }

        weight_.resize(particle.size());
        particle.weight_set().read_weight(&weight_[0]);
        double sum = 0;
        for (typename vsmc::Particle<T>::size_type i = 0;
             i != particle.size(); ++i)
            sum += weight_[i] * exp_weight_[i];
        particle.value().zconst() += log(sum);
        particle.weight_set().mul_weight(&exp_weight_[0]);

        return 0;
    }

    private:
    std::size_t min_comp_;
    std::size_t max_comp_;
    std::size_t iter_;
    Alpha alpha_;
    std::vector<double> weight_;
    std::vector<double> exp_weight_;
    vsmc::Particle<T> dummy_particle_;
};

#endif // VSMC_EXAMPLE_MOVE_SMC_PAIR_HPP
