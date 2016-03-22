//============================================================================
// vSMC/lib/src/rng/random_walk.cpp
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

#include "libvsmc.hpp"

extern "C" {

int vsmc_random_walk(vsmc_rng *rng_ptr, int dim, double *x, double *ltx,
    double (*log_target)(int, const double *),
    double (*proposal)(vsmc_rng *, int, const double *, double *))
{
    ::vsmc::RandomWalk<double, ::vsmc::Dynamic> rw(
        static_cast<std::size_t>(dim));

    auto lt = [log_target, dim](
        std::size_t, const double *lx) { return log_target(dim, lx); };

    auto prop = [proposal, rng_ptr, dim](::vsmc::RNG &, std::size_t,
        const double *px,
        double *py) { return proposal(rng_ptr, dim, px, py); };

    return static_cast<int>(rw(::vsmc::cast(rng_ptr), x, ltx, lt, prop));
}

int vsmc_random_walk_g(vsmc_rng *rng_ptr, int dim_x, int dim_g, double *x,
    double *ltx, double *g,
    double (*log_target)(int, int, const double *, double *),
    double (*proposal)(vsmc_rng *, int, const double *, double *))
{
    ::vsmc::RandomWalkG<double, vsmc::Dynamic, ::vsmc::Dynamic> rw(
        static_cast<std::size_t>(dim_x), static_cast<std::size_t>(dim_g));

    auto lt = [log_target, dim_x, dim_g](std::size_t, std::size_t,
        const double *lx,
        double *lg) { return log_target(dim_x, dim_g, lx, lg); };

    auto prop = [proposal, rng_ptr, dim_x](::vsmc::RNG &, std::size_t,
        const double *px,
        double *py) { return proposal(rng_ptr, dim_x, px, py); };

    return static_cast<int>(rw(::vsmc::cast(rng_ptr), x, ltx, g, lt, prop));
}

double vsmc_normal_proposal(vsmc_rng *rng_ptr, int, const double *x, double *y,
    double stddev, double a, double b)
{
    ::vsmc::NormalProposal<double> prop(stddev, a, b);

    return prop(::vsmc::cast(rng_ptr), 1, x, y);
}

double vsmc_normal_mv_proposal(vsmc_rng *rng_ptr, int dim, const double *x,
    double *y, const double *chol, const double *a, const double *b)
{
    ::vsmc::NormalMVProposal<double, vsmc::Dynamic> prop(
        static_cast<std::size_t>(dim), chol, a, b);

    return prop(::vsmc::cast(rng_ptr), static_cast<std::size_t>(dim), x, y);
}

} // extern "C"
