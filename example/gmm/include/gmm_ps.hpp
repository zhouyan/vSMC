//============================================================================
// vSMC/example/gmm/include/gmm_ps.hpp
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

#ifndef VSMC_EXAMPLE_GMM_PS_HPP
#define VSMC_EXAMPLE_GMM_PS_HPP

#include "gmm.hpp"

template <typename Backend>
inline void gmm_ps_run(
    std::size_t N, std::size_t n, std::size_t c, std::size_t power, int twid)
{
    GMMMoveSMC::alpha_setter_type alpha_setter;
    if (power == 0)
        alpha_setter = GMMAlphaLinear(n);
    else
        alpha_setter = GMMAlphaPrior(n, power);

    vsmc::Seed::instance().set(101);
    vsmc::Sampler<GMM> sampler(N);
    sampler.resample_method(vsmc::Stratified, 0.5);
    sampler.particle().state().comp_num(c);
    sampler.eval(GMMInit<Backend>(), vsmc::SamplerInit);
    sampler.eval(GMMMoveSMC(alpha_setter), vsmc::SamplerMove);
    sampler.eval(GMMMoveMu<Backend>(), vsmc::SamplerMCMC);
    sampler.eval(GMMMoveLambda<Backend>(), vsmc::SamplerMCMC);
    sampler.eval(GMMMoveWeight<Backend>(), vsmc::SamplerMCMC);
    sampler.monitor(
        "path_integrand", vsmc::Monitor<GMM>(1, GMMPathIntegrand<Backend>()));
    sampler.monitor("path_grid", vsmc::Monitor<GMM>(1, GMMPathGrid(), true));
    sampler.initialize();

    vsmc::StopWatch watch;
    watch.start();
    sampler.iterate(n);
    watch.stop();
    double time = watch.seconds();

    double ps = 0;
    auto ps_integrand = sampler.monitor("path_integrand");
    auto ps_grid = sampler.monitor("path_grid");
    for (std::size_t iter = 1; iter < sampler.iter_size(); ++iter) {
        ps += 0.5 *
            (ps_integrand.record(0, iter) + ps_integrand.record(0, iter - 1)) *
            (ps_grid.record(0, iter) - ps_grid.record(0, iter - 1));
    }

    std::cout << std::setw(twid) << std::left << gmm_backend_name<Backend>();
    std::cout << std::setw(twid) << std::right << std::fixed << ps;
    std::cout << std::setw(twid) << std::right << std::fixed << time;
    std::cout << std::endl;
}

inline void gmm_ps_run(
    std::size_t N, std::size_t n, std::size_t c, std::size_t power, int twid)
{
    gmm_ps_run<vsmc::BackendSEQ>(N, n, c, power, twid);
    gmm_ps_run<vsmc::BackendSTD>(N, n, c, power, twid);
#if VSMC_HAS_OMP
    gmm_ps_run<vsmc::BackendOMP>(N, n, c, power, twid);
#endif
#if VSMC_HAS_TBB
    gmm_ps_run<vsmc::BackendTBB>(N, n, c, power, twid);
#endif
}

inline void gmm_ps(
    std::size_t N, std::size_t n, std::size_t c, std::size_t power)
{
    const int twid = 20;
    const std::size_t lwid = twid * 3;

    std::cout << std::string(lwid, '=') << std::endl;
    std::cout << std::setw(twid) << std::left << "Backend";
    std::cout << std::setw(twid) << std::right << "Path sampling";
    std::cout << std::setw(twid) << std::right << "Time (s)";
    std::cout << std::endl;
    std::cout << std::string(lwid, '-') << std::endl;
    gmm_ps_run(N, n, c, power, twid);
    std::cout << std::string(lwid, '-') << std::endl;
}

#endif // VSMC_EXAMPLE_GMM_PS_HPP
