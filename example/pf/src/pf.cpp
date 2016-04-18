//============================================================================
// vSMC/example/pf/src/pf.cpp
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

#include "pf_@smp@.hpp"

int main()
{
    std::string datafile("pf.data");
    std::string implname("@SMP@");

    std::string pf_time("pf.");
    pf_time += "@SMP@";
    std::ofstream time(pf_time);
    time << std::setw(20) << std::left << "N";
    time << std::setw(20) << std::left << "Implementation";
    time << std::setw(20) << std::left << "ResampleScheme";
    time << std::setw(20) << std::left << "MatrixLayout";
    time << std::setw(20) << std::left << "RNGSet";
    time << std::setw(20) << std::left << "Time";
    time << std::endl;
    pf_run(ParticleNum, datafile, implname, true, time);
    std::size_t N = 1;
    while (N < ParticleNum) {
        N *= 2;
        pf_run(N, datafile, implname, false, time);
    }

    return 0;
}
