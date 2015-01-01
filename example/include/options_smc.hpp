//============================================================================
// vSMC/example/include/options_smc.hpp
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

Config
.add("threshold",       "Threshold for resampling",     &Threshold,   0.5)
.add("particle_num",   "Particle number",               &ParticleNum, 1000)
.add("proposal_scale", "Proposal: 1: Default; 2: Adaptive", &ProposalScale, 1)
.add("mh_alpha",       "MH schedule (alph == 1)",       &MHAlpha)
.add("mh_iter",        "MH iteration number",           &MHIterNum)
.add("ess",            "Enable ESS adaptive schedule",  &ESSDrop)
.add("cess",           "Enable CESS adaptive schedule", &CESSDrop)
.add("linear",         "Enbale linear schedule",        &LinearIterNum)
.add("prior2",         "Enbale prior2 schedule",        &Prior2IterNum)
.add("prior5",         "Enbale prior5 schedule",        &Prior5IterNum)
.add("posterior2",     "Enbale posterior2 schedule",    &Posterior2IterNum)
.add("posterior5",     "Enbale posterior5 schedule",    &Posterior5IterNum);
