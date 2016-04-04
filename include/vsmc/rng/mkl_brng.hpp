//============================================================================
// vSMC/include/vsmc/rng/mkl_brng.hpp
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

#ifndef VSMC_RNG_MKL_BRNG_HPP
#define VSMC_RNG_MKL_BRNG_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/engine.hpp>
#include <vsmc/rng/mkl.hpp>
#include <vsmc/vsmc.h>

namespace vsmc
{

/// \brief Register a C++11 RNG as MKL BRNG
/// \ingroup MKLRNG
///
/// \details
/// Only engines defined in this library and the standard library are
/// specialized. This function requires the C runtime of the library.
template <typename RNGType>
int mkl_brng();

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name)                            \
    template <>                                                               \
    inline int mkl_brng<RNGType>()                                            \
    {                                                                         \
        static const int brng = ::vsmc_mkl_brng_##name();                     \
                                                                              \
        return brng;                                                          \
    }

#include <vsmc/rng/internal/rng_define_macro.hpp>

} // namespace vsmc

#endif // VSMC_RNG_MKL_BRNG_HPP
