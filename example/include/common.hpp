//============================================================================
// vSMC/vSMCExample/include/common.hpp
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

#ifndef VSMC_EXAMPLE_COMMON_HPP
#define VSMC_EXAMPLE_COMMON_HPP

#include <vsmc/core/sampler.hpp>
#include <vsmc/core/state_matrix.hpp>
#include <vsmc/math/constants.hpp>
#include <vsmc/utility/stop_watch.hpp>
#include <vsmc/utility/program_option.hpp>
#include <algorithm>
#include <fstream>
#include <iostream>

static std::string Suffix;
static std::size_t Repeat;
static vsmc::Seed::result_type Seed;
static bool Interactive;
static int ProposalScale;

static vsmc::ProgramOptionMap Config;

template <typename T>
inline void grow (std::size_t num, T &orig)
{
    if (orig.size() < num && num > 0) {
        T temp(num);
        for (std::size_t i = 0; i != orig.size(); ++i)
            temp[i] = orig[i];
        orig.resize(num);
        for (std::size_t i = 0; i != num; ++i)
            orig[i] = temp[i];
    }
}

inline bool is_valid (double val)
{
    const double val_max = std::numeric_limits<double>::max VSMC_MNE ();
    if (val > -val_max && val < val_max)
        return true;
    return false;
}

#endif // VSMC_EXAMPLE_COMMON_HPP
