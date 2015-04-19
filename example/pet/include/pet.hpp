//============================================================================
// vSMC/example/pet/include/pet.hpp
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

#ifndef VSMC_EXAMPLE_PET_HPP
#define VSMC_EXAMPLE_PET_HPP

#define BASE_STATE   vsmc::State@SMP@
#define BASE_INIT    vsmc::Initialize@SMP@
#define BASE_MOVE    vsmc::Move@SMP@
#define BASE_MONITOR vsmc::MonitorEval@SMP@
#define BASE_PATH    vsmc::PathEval@SMP@

#include <vsmc/smp/backend_@smp@.hpp>

static const std::size_t InitCompNum = 1;
static const std::size_t MinCompNum = 1;
static const std::size_t MaxCompNum = 3;

static std::string DataFile;
static std::size_t SM;
static std::size_t CM;

#include "common.hpp"
#include "pet_param.hpp"
#include "pet_state.hpp"
#include "pet_init.hpp"
#include "pet_move.hpp"
#include "pet_monitor.hpp"
#include "pet_proposal.hpp"

template <typename InputStream>
inline void pet_ignore (InputStream &is)
{is.ignore(std::numeric_limits<std::streamsize>::max VSMC_MNE (), '\n');}

#endif // VSMC_EXAMPLE_PET_HPP
