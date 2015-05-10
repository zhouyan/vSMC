//============================================================================
// vSMC/example/pf/include/pf_smp_do.hpp
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

#ifndef VSMC_EXAMPLE_PF_SMP_DO_HPP
#define VSMC_EXAMPLE_PF_SMP_DO_HPP

template <vsmc::MatrixOrder Order>
inline void cv_do(
    vsmc::ResampleScheme res, char **argv, const std::string &name)
{
    typedef cv_state<Order> cv;

    vsmc::Seed::instance().set(101);
    vsmc::Sampler<cv> sampler(ParticleNum, res, 0.5);
    sampler.init(cv_init<Order>())
        .move(cv_move<Order>(), false)
        .monitor("pos", 2, cv_est<Order>());
    sampler.monitor("pos").name(0) = "pos.x";
    sampler.monitor("pos").name(1) = "pos.y";
    sampler.initialize(argv[1]);
    sampler.iterate(DataNum - 1);

    std::string est_file_name(argv[2] + name + ".txt");
    std::ofstream est_file;
    est_file.open(est_file_name.c_str());
    est_file << sampler << std::endl;
    est_file.close();

#if VSMC_HAS_HDF5
    vsmc::hdf5store(sampler, argv[2] + name + ".h5", "Sampler");
#endif
}

#endif // VSMC_EXAMPLE_PF_SMP_DO_HPP
