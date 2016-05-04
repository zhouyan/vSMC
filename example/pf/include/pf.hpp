//============================================================================
// vSMC/example/pf/include/pf.hpp
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

#ifndef VSMC_EXAMPLE_PF_HPP
#define VSMC_EXAMPLE_PF_HPP

#include <vsmc/vsmc.hpp>

template <typename>
std::string pf_smp_name();

template <>
std::string pf_smp_name<vsmc::BackendSEQ>()
{
    return "BackendSEQ";
}

template <>
std::string pf_smp_name<vsmc::BackendSTD>()
{
    return "BackendSTD";
}

#if VSMC_HAS_OMP
template <>
std::string pf_smp_name<vsmc::BackendOMP>()
{
    return "BackendOMP";
}
#endif

#if VSMC_HAS_TBB
template <>
std::string pf_smp_name<vsmc::BackendTBB>()
{
    return "BackendTBB";
}
#endif

template <vsmc::ResampleScheme>
std::string pf_res_name();

template <>
std::string pf_res_name<vsmc::Multinomial>()
{
    return "Multinomial";
}

template <>
std::string pf_res_name<vsmc::Residual>()
{
    return "Residual";
}

template <>
std::string pf_res_name<vsmc::ResidualStratified>()
{
    return "ResidualStratified";
}

template <>
std::string pf_res_name<vsmc::ResidualSystematic>()
{
    return "ResidualSystematic";
}

template <>
std::string pf_res_name<vsmc::Stratified>()
{
    return "Stratified";
}

template <>
std::string pf_res_name<vsmc::Systematic>()
{
    return "Systematic";
}

template <vsmc::MatrixLayout>
std::string pf_rc_name();

template <>
std::string pf_rc_name<vsmc::RowMajor>()
{
    return "RowMajor";
}

template <>
std::string pf_rc_name<vsmc::ColMajor>()
{
    return "ColMajor";
}

template <typename>
std::string pf_rs_name();

template <>
std::string pf_rs_name<vsmc::RNGSetVector<>>()
{
    return "RNGSetVector";
}

#if VSMC_HAS_TBB
template <>
std::string pf_rs_name<vsmc::RNGSetTBB<>>()
{
    return "RNGSetTBB";
}
#endif

#endif // VSMC_EXAMPLE_PF_HPP
