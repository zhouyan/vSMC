//============================================================================
// vSMC/example/rng/src/rng_r123.cpp
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

#include "rng_test.hpp"

#if VSMC_HAS_AES_NI
#define R123_USE_AES_NI 1
#include <Random123/aes.h>
#include <Random123/ars.h>
#endif

#include <Random123/philox.h>
#include <Random123/threefry.h>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4521)
#endif
#include <Random123/conventional/Engine.hpp>
#ifdef _MSC_VER
#pragma warning(pop)
#endif

int main(int argc, char **argv)
{
    VSMC_RNG_TEST_PRE(rng_r123);

#if VSMC_HAS_AES_NI
    VSMC_RNG_TEST(r123::Engine<r123::AESNI4x32>);
    VSMC_RNG_TEST(r123::Engine<r123::ARS4x32_R<7>>);
#endif
    VSMC_RNG_TEST(r123::Engine<r123::Philox2x32>);
    VSMC_RNG_TEST(r123::Engine<r123::Philox4x32>);
    VSMC_RNG_TEST(r123::Engine<r123::Philox2x64>);
    VSMC_RNG_TEST(r123::Engine<r123::Philox4x64>);
    VSMC_RNG_TEST(r123::Engine<r123::Threefry2x32>);
    VSMC_RNG_TEST(r123::Engine<r123::Threefry4x32>);
    VSMC_RNG_TEST(r123::Engine<r123::Threefry2x64>);
    VSMC_RNG_TEST(r123::Engine<r123::Threefry4x64>);

    VSMC_RNG_TEST_POST;

    return 0;
}
