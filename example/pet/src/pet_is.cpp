//============================================================================
// vSMC/example/pet/src/pet_is.cpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013,2014, Yan Zhou
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

#include "pet_@smp@.hpp"
#include "smc.hpp"

int main (int argc, char **argv)
{
#include "options_main.hpp"
#include "options_smc.hpp"
#include "pet_options.hpp"
#include "options_process.hpp"
#include "pet_data.hpp"

    vsmc::Sampler<pet_state> sampler(ParticleNum);
    sampler.init(pet_init());
    sampler.initialize(&info);
    info.read_time  = false;
    info.read_conv  = false;
    info.read_prior = false;
    info.read_sd    = false;
    info.read_model = false;

    std::ofstream output("is.save");
    output << "Model." << SM << '\t' << "Model." << CM << '\n';
    for (std::size_t r = 0; r != Repeat; ++r) {
        double py = 0;

        sampler.particle().value().comp_num(SM);
        sampler.initialize();
        py = 0;
        for (std::size_t i = 0; i != ParticleNum; ++i)
            py += std::exp(
                    sampler.particle().value().state(i, 0).log_likelihood());
        py /= static_cast<double>(ParticleNum);
        py = std::log(py);
        py += sampler.particle().value().log_likelihood_const();
        output << py << '\t';

        sampler.particle().value().comp_num(CM);
        sampler.initialize();
        py = 0;
        for (std::size_t i = 0; i != ParticleNum; ++i)
            py += std::exp(
                    sampler.particle().value().state(i, 0).log_likelihood());
        py /= static_cast<double>(ParticleNum);
        py = std::log(py);
        py += sampler.particle().value().log_likelihood_const();
        output << py << '\n';
    }
    output.close();
    output.clear();

    return 0;
}
