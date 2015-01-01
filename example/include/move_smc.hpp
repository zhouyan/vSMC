//============================================================================
// vSMC/example/include/move_smc.hpp
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

#ifndef VSMC_EXAMPLE_MOVE_SMC_HPP
#define VSMC_EXAMPLE_MOVE_SMC_HPP

template <typename T, typename Alpha, typename Proposal>
class move_smc
{
    public :

    typedef Alpha alpha_type;
    typedef Proposal proposal_type;

    move_smc (
            typename Alpha::value_type alpha_config = 0,
            typename Proposal::value_type proposal_config = 0) :
        alpha_(alpha_config), proposal_(proposal_config) {}

    move_smc (const Alpha &alpha, const Proposal &proposal) :
        alpha_(alpha), proposal_(proposal) {}

    std::size_t operator() (std::size_t iter, vsmc::Particle<T> &particle)
    {
        using std::exp;
        using std::log;

        if (particle.value().state(0, 0).alpha() <= 0)
            particle.value().zconst() = 0;

        alpha_.alpha_iter(iter, particle);
        proposal_.proposal_iter(iter, particle);
        inc_weight_.resize(particle.size());
        weight_.resize(particle.size());
        particle.weight_set().read_weight(&weight_[0]);
        double sum = 0;
        for (typename vsmc::Particle<T>::size_type i = 0;
                i != particle.size(); ++i) {
            inc_weight_[i] = particle.value().state(i, 0).log_likelihood() *
                particle.value().state(0, 0).alpha_inc();
            sum += weight_[i] * exp(inc_weight_[i]);
        }
        particle.value().zconst() += log(sum);
        particle.weight_set().add_log_weight(&inc_weight_[0]);

        return 0;
    }

    private :

    Alpha alpha_;
    Proposal proposal_;
    std::vector<double> weight_;
    std::vector<double> inc_weight_;
    std::vector<double> exp_weight_;
};

#endif // VSMC_EXAMPLE_MOVE_SMC_HPP
