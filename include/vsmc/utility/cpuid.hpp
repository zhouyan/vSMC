//============================================================================
// vSMC/include/vsmc/utility/cpuid.hpp
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

#ifndef VSMC_UTILITY_CPUID_HPP
#define VSMC_UTILITY_CPUID_HPP

#include <vsmc/internal/common.hpp>

#ifdef VSMC_MSVC
#include <intrin.h>
#endif

#define VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(feat, a, c, i, b)             \
    template <>                                                              \
    struct CPUIDFeatureInfo<CPUIDFeature##feat> {                            \
        static std::string str() { return std::string(#feat); }              \
        static constexpr const unsigned eax = a##U;                          \
        static constexpr const unsigned ecx = c##U;                          \
        static constexpr const unsigned bit = b##U;                          \
        static constexpr const std::size_t index = i;                        \
    };

#define VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(feat, a, c, i, b)         \
    template <>                                                              \
    struct CPUIDFeatureInfo<CPUIDFeatureExt##feat> {                         \
        static std::string str() { return std::string(#feat); }              \
        static constexpr const unsigned eax = 0x80000000U + a##U;            \
        static constexpr const unsigned ecx = c##U;                          \
        static constexpr const unsigned bit = b##U;                          \
        static constexpr const std::size_t index = i;                        \
    };

namespace vsmc
{

/// \brief Get the CPUID information stored in EAX, EBX, ECX and EDX, given
/// input EAX and ECX values
/// \ingroup CPUID
inline void cpuid(unsigned eax, unsigned ecx, unsigned *reg)
{
#ifdef VSMC_MSVC
    __cpuidex(reinterpret_cast<int *>(reg), static_cast<int>(eax),
        static_cast<int>(ecx));
#else  // VSMC_MSVC
    unsigned ebx = 0x00;
    unsigned edx = 0x00;
    __asm__ volatile("cpuid\n"
                     : "=a"(eax), "=b"(ebx), "=c"(ecx), "=d"(edx)
                     : "a"(eax), "c"(ecx));
    reg[0] = eax;
    reg[1] = ebx;
    reg[2] = ecx;
    reg[3] = edx;
#endif // VSMC_MSVC
}

/// \brief CPU features
/// \ingroup CPUID
enum CPUIDFeature {
    CPUIDFeatureSSE3,         ///< EAX = 0x01, ECX = 0x00; ECX[00]
    CPUIDFeaturePCLMULQDQ,    ///< EAX = 0x01, ECX = 0x00; ECX[01]
    CPUIDFeatureDTES64,       ///< EAX = 0x01, ECX = 0x00; ECX[02]
    CPUIDFeatureMONITOR,      ///< EAX = 0x01, ECX = 0x00; ECX[03]
    CPUIDFeatureDS_CPL,       ///< EAX = 0x01, ECX = 0x00; ECX[04]
    CPUIDFeatureVMX,          ///< EAX = 0x01, ECX = 0x00; ECX[05]
    CPUIDFeatureSMX,          ///< EAX = 0x01, ECX = 0x00; ECX[06]
    CPUIDFeatureEST,          ///< EAX = 0x01, ECX = 0x00; ECX[07]
    CPUIDFeatureTM2,          ///< EAX = 0x01, ECX = 0x00; ECX[08]
    CPUIDFeatureSSSE3,        ///< EAX = 0x01, ECX = 0x00; ECX[09]
    CPUIDFeatureCNXT_ID,      ///< EAX = 0x01, ECX = 0x00; ECX[10]
    CPUIDFeatureFMA,          ///< EAX = 0x01, ECX = 0x00; ECX[12]
    CPUIDFeatureCX16,         ///< EAX = 0x01, ECX = 0x00; ECX[13]
    CPUIDFeatureXTPR,         ///< EAX = 0x01, ECX = 0x00; ECX[14]
    CPUIDFeaturePDCM,         ///< EAX = 0x01, ECX = 0x00; ECX[15]
    CPUIDFeaturePCID,         ///< EAX = 0x01, ECX = 0x00; ECX[17]
    CPUIDFeatureDCA,          ///< EAX = 0x01, ECX = 0x00; ECX[18]
    CPUIDFeatureSSE4_1,       ///< EAX = 0x01, ECX = 0x00; ECX[19]
    CPUIDFeatureSSE4_2,       ///< EAX = 0x01, ECX = 0x00; ECX[20]
    CPUIDFeatureX2APIC,       ///< EAX = 0x01, ECX = 0x00; ECX[21]
    CPUIDFeatureMOVBE,        ///< EAX = 0x01, ECX = 0x00; ECX[22]
    CPUIDFeaturePOPCNT,       ///< EAX = 0x01, ECX = 0x00; ECX[23]
    CPUIDFeatureTSC_DEADLINE, ///< EAX = 0x01, ECX = 0x00; ECX[24]
    CPUIDFeatureAES,          ///< EAX = 0x01, ECX = 0x00; ECX[25]
    CPUIDFeatureXSAVE,        ///< EAX = 0x01, ECX = 0x00; ECX[26]
    CPUIDFeatureOSXSAVE,      ///< EAX = 0x01, ECX = 0x00; ECX[27]
    CPUIDFeatureAVX,          ///< EAX = 0x01, ECX = 0x00; ECX[28]
    CPUIDFeatureF16C,         ///< EAX = 0x01, ECX = 0x00; ECX[29]
    CPUIDFeatureRDRAND,       ///< EAX = 0x01, ECX = 0x00; ECX[30]
    CPUIDFeatureHYPERVISOR,   ///< EAX = 0x01, ECX = 0x00; ECX[31]

    CPUIDFeatureFPU,    ///< EAX = 0x01, ECX = 0x00; EDX[00]
    CPUIDFeatureVME,    ///< EAX = 0x01, ECX = 0x00; EDX[01]
    CPUIDFeatureDE,     ///< EAX = 0x01, ECX = 0x00; EDX[02]
    CPUIDFeaturePSE,    ///< EAX = 0x01, ECX = 0x00; EDX[03]
    CPUIDFeatureTSC,    ///< EAX = 0x01, ECX = 0x00; EDX[04]
    CPUIDFeatureMSR,    ///< EAX = 0x01, ECX = 0x00; EDX[05]
    CPUIDFeaturePAE,    ///< EAX = 0x01, ECX = 0x00; EDX[06]
    CPUIDFeatureMCE,    ///< EAX = 0x01, ECX = 0x00; EDX[07]
    CPUIDFeatureCX8,    ///< EAX = 0x01, ECX = 0x00; EDX[08]
    CPUIDFeatureAPIC,   ///< EAX = 0x01, ECX = 0x00; EDX[09]
    CPUIDFeatureSEP,    ///< EAX = 0x01, ECX = 0x00; EDX[11]
    CPUIDFeatureMTRR,   ///< EAX = 0x01, ECX = 0x00; EDX[12]
    CPUIDFeaturePGE,    ///< EAX = 0x01, ECX = 0x00; EDX[13]
    CPUIDFeatureMCA,    ///< EAX = 0x01, ECX = 0x00; EDX[14]
    CPUIDFeatureCMOV,   ///< EAX = 0x01, ECX = 0x00; EDX[15]
    CPUIDFeaturePAT,    ///< EAX = 0x01, ECX = 0x00; EDX[16]
    CPUIDFeaturePSE_36, ///< EAX = 0x01, ECX = 0x00; EDX[17]
    CPUIDFeaturePSN,    ///< EAX = 0x01, ECX = 0x00; EDX[18]
    CPUIDFeatureCLFSH,  ///< EAX = 0x01, ECX = 0x00; EDX[19]
    CPUIDFeatureDS,     ///< EAX = 0x01, ECX = 0x00; EDX[21]
    CPUIDFeatureACPI,   ///< EAX = 0x01, ECX = 0x00; EDX[22]
    CPUIDFeatureMMX,    ///< EAX = 0x01, ECX = 0x00; EDX[23]
    CPUIDFeatureFXSR,   ///< EAX = 0x01, ECX = 0x00; EDX[24]
    CPUIDFeatureSSE,    ///< EAX = 0x01, ECX = 0x00; EDX[25]
    CPUIDFeatureSSE2,   ///< EAX = 0x01, ECX = 0x00; EDX[26]
    CPUIDFeatureSS,     ///< EAX = 0x01, ECX = 0x00; EDX[27]
    CPUIDFeatureHTT,    ///< EAX = 0x01, ECX = 0x00; EDX[28]
    CPUIDFeatureTM,     ///< EAX = 0x01, ECX = 0x00; EDX[29]
    CPUIDFeatureIA64,   ///< EAX = 0x01, ECX = 0x00; EDX[30]
    CPUIDFeaturePBE,    ///< EAX = 0x01, ECX = 0x00; EDX[31]

    CPUIDFeatureFSGSBASE,     ///< EAX = 0x07, ECX = 0x00; EBX[00]
    CPUIDFeatureBMI1,         ///< EAX = 0x07, ECX = 0x00; EBX[03]
    CPUIDFeatureHLE,          ///< EAX = 0x07, ECX = 0x00; EBX[04]
    CPUIDFeatureAVX2,         ///< EAX = 0x07, ECX = 0x00; EBX[05]
    CPUIDFeatureSMEP,         ///< EAX = 0x07, ECX = 0x00; EBX[07]
    CPUIDFeatureBMI2,         ///< EAX = 0x07, ECX = 0x00; EBX[08]
    CPUIDFeatureERMS,         ///< EAX = 0x07, ECX = 0x00; EBX[09]
    CPUIDFeatureINVPCID,      ///< EAX = 0x07, ECX = 0x00; EBX[10]
    CPUIDFeatureRTM,          ///< EAX = 0x07, ECX = 0x00; EBX[11]
    CPUIDFeatureMPX,          ///< EAX = 0x07, ECX = 0x00; EBX[14]
    CPUIDFeatureAVX512F,      ///< EAX = 0x07, ECX = 0x00; EBX[16]
    CPUIDFeatureAVX512DQ,     ///< EAX = 0x07, ECX = 0x00; EBX[17]
    CPUIDFeatureRDSEED,       ///< EAX = 0x07, ECX = 0x00; EBX[18]
    CPUIDFeatureADX,          ///< EAX = 0x07, ECX = 0x00; EBX[19]
    CPUIDFeatureSMAP,         ///< EAX = 0x07, ECX = 0x00; EBX[20]
    CPUIDFeatureAVX512IFMA52, ///< EAX = 0x07, ECX = 0x00; EBX[21]
    CPUIDFeatureCLFLUSHOPT,   ///< EAX = 0x07, ECX = 0x00; EBX[23]
    CPUIDFeatureINTEL_TRACE,  ///< EAX = 0x07, ECX = 0x00; EBX[25]
    CPUIDFeatureAVX512PF,     ///< EAX = 0x07, ECX = 0x00; EBX[26]
    CPUIDFeatureAVX512ER,     ///< EAX = 0x07, ECX = 0x00; EBX[27]
    CPUIDFeatureAVX512CD,     ///< EAX = 0x07, ECX = 0x00; EBX[28]
    CPUIDFeatureSHA,          ///< EAX = 0x07, ECX = 0x00; EBX[29]
    CPUIDFeatureAVX512BW,     ///< EAX = 0x07, ECX = 0x00; EBX[30]
    CPUIDFeatureAVX512VL,     ///< EAX = 0x07, ECX = 0x00; EBX[31]

    CPUIDFeaturePREFETCHWT1, ///< EAX = 0x07, ECX = 0x00; ECX[00]
    CPUIDFeatureAVX512VBMI,  ///< EAX = 0x07, ECX = 0x00; ECX[01]

    CPUIDFeatureExtLAHF_LM,       ///< EAX = 0x80000001, ECX = 0x00; ECX[00]
    CPUIDFeatureExtCMP_LEGACY,    ///< EAX = 0x80000001, ECX = 0x00; ECX[01]
    CPUIDFeatureExtSVM,           ///< EAX = 0x80000001, ECX = 0x00; ECX[02]
    CPUIDFeatureExtEXTAPIC,       ///< EAX = 0x80000001, ECX = 0x00; ECX[03]
    CPUIDFeatureExtCR8_LEGACY,    ///< EAX = 0x80000001, ECX = 0x00; ECX[04]
    CPUIDFeatureExtABM,           ///< EAX = 0x80000001, ECX = 0x00; ECX[05]
    CPUIDFeatureExtSSE4A,         ///< EAX = 0x80000001, ECX = 0x00; ECX[06]
    CPUIDFeatureExtMISALIGNSSE,   ///< EAX = 0x80000001, ECX = 0x00; ECX[07]
    CPUIDFeatureExt3DNOWPREFETCH, ///< EAX = 0x80000001, ECX = 0x00; ECX[08]
    CPUIDFeatureExtOSVW,          ///< EAX = 0x80000001, ECX = 0x00; ECX[09]
    CPUIDFeatureExtIBS,           ///< EAX = 0x80000001, ECX = 0x00; ECX[10]
    CPUIDFeatureExtXOP,           ///< EAX = 0x80000001, ECX = 0x00; ECX[11]
    CPUIDFeatureExtSKINIT,        ///< EAX = 0x80000001, ECX = 0x00; ECX[12]
    CPUIDFeatureExtWDT,           ///< EAX = 0x80000001, ECX = 0x00; ECX[13]
    CPUIDFeatureExtLWP,           ///< EAX = 0x80000001, ECX = 0x00; ECX[15]
    CPUIDFeatureExtFMA4,          ///< EAX = 0x80000001, ECX = 0x00; ECX[16]
    CPUIDFeatureExtTCE,           ///< EAX = 0x80000001, ECX = 0x00; ECX[17]
    CPUIDFeatureExtNODEID_MSR,    ///< EAX = 0x80000001, ECX = 0x00; ECX[19]
    CPUIDFeatureExtTBM,           ///< EAX = 0x80000001, ECX = 0x00; ECX[21]
    CPUIDFeatureExtTOPOEXT,       ///< EAX = 0x80000001, ECX = 0x00; ECX[22]
    CPUIDFeatureExtPERFCTR_CORE,  ///< EAX = 0x80000001, ECX = 0x00; ECX[23]
    CPUIDFeatureExtPERFCTR_NB,    ///< EAX = 0x80000001, ECX = 0x00; ECX[24]
    CPUIDFeatureExtDBX,           ///< EAX = 0x80000001, ECX = 0x00; ECX[26]
    CPUIDFeatureExtPERFTSC,       ///< EAX = 0x80000001, ECX = 0x00; ECX[27]
    CPUIDFeatureExtPCX_L2I,       ///< EAX = 0x80000001, ECX = 0x00; ECX[28]

    CPUIDFeatureExtFPU,      ///< EAX = 0x80000001, ECX = 0x00; EDX[00]
    CPUIDFeatureExtVME,      ///< EAX = 0x80000001, ECX = 0x00; EDX[01]
    CPUIDFeatureExtDE,       ///< EAX = 0x80000001, ECX = 0x00; EDX[02]
    CPUIDFeatureExtPSE,      ///< EAX = 0x80000001, ECX = 0x00; EDX[03]
    CPUIDFeatureExtTSC,      ///< EAX = 0x80000001, ECX = 0x00; EDX[04]
    CPUIDFeatureExtMSR,      ///< EAX = 0x80000001, ECX = 0x00; EDX[05]
    CPUIDFeatureExtPAE,      ///< EAX = 0x80000001, ECX = 0x00; EDX[06]
    CPUIDFeatureExtMCE,      ///< EAX = 0x80000001, ECX = 0x00; EDX[07]
    CPUIDFeatureExtCX8,      ///< EAX = 0x80000001, ECX = 0x00; EDX[08]
    CPUIDFeatureExtAPIC,     ///< EAX = 0x80000001, ECX = 0x00; EDX[09]
    CPUIDFeatureExtSYSCALL,  ///< EAX = 0x80000001, ECX = 0x00; EDX[11]
    CPUIDFeatureExtMTRR,     ///< EAX = 0x80000001, ECX = 0x00; EDX[12]
    CPUIDFeatureExtPGE,      ///< EAX = 0x80000001, ECX = 0x00; EDX[13]
    CPUIDFeatureExtMCA,      ///< EAX = 0x80000001, ECX = 0x00; EDX[14]
    CPUIDFeatureExtCMOV,     ///< EAX = 0x80000001, ECX = 0x00; EDX[15]
    CPUIDFeatureExtPAT,      ///< EAX = 0x80000001, ECX = 0x00; EDX[16]
    CPUIDFeatureExtPSE36,    ///< EAX = 0x80000001, ECX = 0x00; EDX[17]
    CPUIDFeatureExtMP,       ///< EAX = 0x80000001, ECX = 0x00; EDX[19]
    CPUIDFeatureExtNX,       ///< EAX = 0x80000001, ECX = 0x00; EDX[20]
    CPUIDFeatureExtMMX,      ///< EAX = 0x80000001, ECX = 0x00; EDX[22]
    CPUIDFeatureExtMMXEXT,   ///< EAX = 0x80000001, ECX = 0x00; EDX[23]
    CPUIDFeatureExtFXSR,     ///< EAX = 0x80000001, ECX = 0x00; EDX[24]
    CPUIDFeatureExtFXSR_OPT, ///< EAX = 0x80000001, ECX = 0x00; EDX[25]
    CPUIDFeatureExtGBPAGES,  ///< EAX = 0x80000001, ECX = 0x00; EDX[26]
    CPUIDFeatureExtRDTSCP,   ///< EAX = 0x80000001, ECX = 0x00; EDX[27]
    CPUIDFeatureExtLM,       ///< EAX = 0x80000001, ECX = 0x00; EDX[29]
    CPUIDFeatureExt3DNOWEXT, ///< EAX = 0x80000001, ECX = 0x00; EDX[30]
    CPUIDFeatureExt3DNOW     ///< EAX = 0x80000001, ECX = 0x00; EDX[31]
};                           // enum CPUIDFeature

/// \brief Type of CPU caches
enum CPUIDCacheType {
    CPUIDCacheTypeNull,        ///< No more cache
    CPUIDCacheTypeData,        ///< Data cache
    CPUIDCacheTypeInstruction, ///< Instruction cache
    CPUIDCacheTypeUnified      ///< Unified cache
};                             // enum CPUIDCacheType

/// \brief CPU feature information
/// \ingroup CPUID
///
/// \details
/// This class is specialized for each value of CPUIDFeature
template <CPUIDFeature>
struct CPUIDFeatureInfo {
    /// \brief A short string representing the feature
    static std::string str();

    /// \brief The value of the calling parameter EAX
    static constexpr const unsigned eax = 0x00U;

    /// \brief The value of the calling parameter ECX
    static constexpr const unsigned ecx = 0x00U;

    /// \brief The index of in CPUID::reg_type of the output register
    static constexpr const unsigned index = 0x00U;

    /// \brief The bit number of the feature in the register
    static constexpr const unsigned bit = 0x00U;
}; // struct CPUIDFeatureInfo

VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(SSE3, 0x01, 0x00, 2, 0)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(PCLMULQDQ, 0x01, 0x00, 2, 1)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(DTES64, 0x01, 0x00, 2, 2)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(MONITOR, 0x01, 0x00, 2, 3)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(DS_CPL, 0x01, 0x00, 2, 4)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(VMX, 0x01, 0x00, 2, 5)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(SMX, 0x01, 0x00, 2, 6)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(EST, 0x01, 0x00, 2, 7)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(TM2, 0x01, 0x00, 2, 8)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(SSSE3, 0x01, 0x00, 2, 9)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(CNXT_ID, 0x01, 0x00, 2, 10)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(FMA, 0x01, 0x00, 2, 12)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(CX16, 0x01, 0x00, 2, 13)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(XTPR, 0x01, 0x00, 2, 14)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(PDCM, 0x01, 0x00, 2, 15)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(PCID, 0x01, 0x00, 2, 17)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(DCA, 0x01, 0x00, 2, 18)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(SSE4_1, 0x01, 0x00, 2, 19)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(SSE4_2, 0x01, 0x00, 2, 20)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(X2APIC, 0x01, 0x00, 2, 21)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(MOVBE, 0x01, 0x00, 2, 22)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(POPCNT, 0x01, 0x00, 2, 23)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(TSC_DEADLINE, 0x01, 0x00, 2, 24)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(AES, 0x01, 0x00, 2, 25)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(XSAVE, 0x01, 0x00, 2, 26)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(OSXSAVE, 0x01, 0x00, 2, 27)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(AVX, 0x01, 0x00, 2, 28)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(F16C, 0x01, 0x00, 2, 29)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(RDRAND, 0x01, 0x00, 2, 30)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(HYPERVISOR, 0x01, 0x00, 2, 31)

VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(FPU, 0x01, 0x00, 3, 0)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(VME, 0x01, 0x00, 3, 1)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(DE, 0x01, 0x00, 3, 2)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(PSE, 0x01, 0x00, 3, 3)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(TSC, 0x01, 0x00, 3, 4)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(MSR, 0x01, 0x00, 3, 5)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(PAE, 0x01, 0x00, 3, 6)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(MCE, 0x01, 0x00, 3, 7)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(CX8, 0x01, 0x00, 3, 8)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(APIC, 0x01, 0x00, 3, 9)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(SEP, 0x01, 0x00, 3, 11)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(MTRR, 0x01, 0x00, 3, 12)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(PGE, 0x01, 0x00, 3, 13)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(MCA, 0x01, 0x00, 3, 14)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(CMOV, 0x01, 0x00, 3, 15)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(PAT, 0x01, 0x00, 3, 16)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(PSE_36, 0x01, 0x00, 3, 17)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(PSN, 0x01, 0x00, 3, 18)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(CLFSH, 0x01, 0x00, 3, 19)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(DS, 0x01, 0x00, 3, 21)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(ACPI, 0x01, 0x00, 3, 22)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(MMX, 0x01, 0x00, 3, 23)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(FXSR, 0x01, 0x00, 3, 24)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(SSE, 0x01, 0x00, 3, 25)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(SSE2, 0x01, 0x00, 3, 26)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(SS, 0x01, 0x00, 3, 27)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(HTT, 0x01, 0x00, 3, 28)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(TM, 0x01, 0x00, 3, 29)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(IA64, 0x01, 0x00, 3, 30)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(PBE, 0x01, 0x00, 3, 31)

VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(FSGSBASE, 0x07, 0x00, 1, 0)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(BMI1, 0x07, 0x00, 1, 3)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(HLE, 0x07, 0x00, 1, 4)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(AVX2, 0x07, 0x00, 1, 5)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(SMEP, 0x07, 0x00, 1, 7)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(BMI2, 0x07, 0x00, 1, 8)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(ERMS, 0x07, 0x00, 1, 9)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(INVPCID, 0x07, 0x00, 1, 10)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(RTM, 0x07, 0x00, 1, 11)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(MPX, 0x07, 0x00, 1, 14)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(AVX512F, 0x07, 0x00, 1, 16)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(AVX512DQ, 0x07, 0x00, 1, 17)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(RDSEED, 0x07, 0x00, 1, 18)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(ADX, 0x07, 0x00, 1, 19)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(SMAP, 0x07, 0x00, 1, 20)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(AVX512IFMA52, 0x07, 0x00, 1, 21)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(CLFLUSHOPT, 0x07, 0x00, 1, 23)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(INTEL_TRACE, 0x07, 0x00, 1, 25)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(AVX512PF, 0x07, 0x00, 1, 26)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(AVX512ER, 0x07, 0x00, 1, 27)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(AVX512CD, 0x07, 0x00, 1, 28)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(SHA, 0x07, 0x00, 1, 29)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(AVX512BW, 0x07, 0x00, 1, 30)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(AVX512VL, 0x07, 0x00, 1, 31)

VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(PREFETCHWT1, 0x07, 0x00, 3, 0)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO(AVX512VBMI, 0x07, 0x00, 3, 1)

VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(LAHF_LM, 0x01, 0x00, 2, 0)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(CMP_LEGACY, 0x01, 0x00, 2, 1)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(SVM, 0x01, 0x00, 2, 2)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(EXTAPIC, 0x01, 0x00, 2, 3)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(CR8_LEGACY, 0x01, 0x00, 2, 4)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(ABM, 0x01, 0x00, 2, 5)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(SSE4A, 0x01, 0x00, 2, 6)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(MISALIGNSSE, 0x01, 0x00, 2, 7)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(3DNOWPREFETCH, 0x01, 0x00, 2, 8)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(OSVW, 0x01, 0x00, 2, 9)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(IBS, 0x01, 0x00, 2, 10)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(XOP, 0x01, 0x00, 2, 11)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(SKINIT, 0x01, 0x00, 2, 12)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(WDT, 0x01, 0x00, 2, 13)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(LWP, 0x01, 0x00, 2, 15)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(FMA4, 0x01, 0x00, 2, 16)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(TCE, 0x01, 0x00, 2, 17)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(NODEID_MSR, 0x01, 0x00, 2, 19)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(TBM, 0x01, 0x00, 2, 21)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(TOPOEXT, 0x01, 0x00, 2, 22)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(PERFCTR_CORE, 0x01, 0x00, 2, 23)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(PERFCTR_NB, 0x01, 0x00, 2, 24)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(DBX, 0x01, 0x00, 2, 26)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(PERFTSC, 0x01, 0x00, 2, 27)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(PCX_L2I, 0x01, 0x00, 2, 28)

VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(FPU, 0x01, 0x00, 3, 0)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(VME, 0x01, 0x00, 3, 1)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(DE, 0x01, 0x00, 3, 2)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(PSE, 0x01, 0x00, 3, 3)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(TSC, 0x01, 0x00, 3, 4)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(MSR, 0x01, 0x00, 3, 5)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(PAE, 0x01, 0x00, 3, 6)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(MCE, 0x01, 0x00, 3, 7)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(CX8, 0x01, 0x00, 3, 8)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(APIC, 0x01, 0x00, 3, 9)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(SYSCALL, 0x01, 0x00, 3, 11)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(MTRR, 0x01, 0x00, 3, 12)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(PGE, 0x01, 0x00, 3, 13)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(MCA, 0x01, 0x00, 3, 14)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(CMOV, 0x01, 0x00, 3, 15)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(PAT, 0x01, 0x00, 3, 16)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(PSE36, 0x01, 0x00, 3, 17)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(MP, 0x01, 0x00, 3, 19)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(NX, 0x01, 0x00, 3, 20)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(MMX, 0x01, 0x00, 3, 22)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(MMXEXT, 0x01, 0x00, 3, 23)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(FXSR, 0x01, 0x00, 3, 24)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(FXSR_OPT, 0x01, 0x00, 3, 25)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(GBPAGES, 0x01, 0x00, 3, 26)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(RDTSCP, 0x01, 0x00, 3, 27)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(LM, 0x01, 0x00, 3, 29)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(3DNOWEXT, 0x01, 0x00, 3, 30)
VSMC_DEFINE_UTILITY_CPUID_FEATURE_INFO_EXT(3DNOW, 0x01, 0x00, 3, 31)

/// \brief Query CPUID information
/// \ingroup CPUID
///
/// \note Most member functions are not thread-safe. However, normal program
/// shall not need to call CPUID instruction from multiple threads. Most
/// information shall be obtained once when the program launch.
class CPUID
{
    public:
    /// \brief The array type that holds EAX, EBX, ECX, and EDX, in that order
    typedef std::array<unsigned, 4> reg_type;

    /// \brief Structure of deterministic cache parameter
    struct cache_param_type {
        cache_param_type(const reg_type &reg)
            : level_(0),
              max_proc_sharing_(0),
              max_proc_physical_(0),
              line_size_(0),
              partitions_(0),
              ways_(0),
              sets_(0),
              self_initializing_(false),
              fully_associative_(false),
              wbinvd_(false),
              inclusiveness_(false),
              complex_indexing_(false)
        {
            unsigned t = extract_bits<4, 0>(std::get<0>(reg));
            switch (t) {
                case 1: type_ = CPUIDCacheTypeData; break;
                case 2: type_ = CPUIDCacheTypeInstruction; break;
                case 3: type_ = CPUIDCacheTypeUnified; break;
                default: type_ = CPUIDCacheTypeNull; return;
            }

            level_ = extract_bits<7, 5>(std::get<0>(reg));
            self_initializing_ = test_bit<8>(std::get<0>(reg));
            fully_associative_ = test_bit<9>(std::get<0>(reg));
            max_proc_sharing_ = extract_bits<25, 14>(std::get<0>(reg)) + 1;
            max_proc_physical_ = extract_bits<31, 26>(std::get<0>(reg)) + 1;

            line_size_ = extract_bits<11, 0>(std::get<1>(reg)) + 1;
            partitions_ = extract_bits<21, 12>(std::get<1>(reg)) + 1;
            ways_ = extract_bits<31, 22>(std::get<1>(reg)) + 1;
            sets_ = extract_bits<31, 0>(std::get<2>(reg)) + 1;
            size_ = line_size_ * partitions_ * ways_ * sets_;

            wbinvd_ = test_bit<0>(std::get<3>(reg));
            inclusiveness_ = test_bit<1>(std::get<3>(reg));
            complex_indexing_ = test_bit<2>(std::get<3>(reg));
        }

        /// \brief The type of this cache
        CPUIDCacheType type() const { return type_; }

        /// \brief The level of this cache
        unsigned level() const { return level_; }

        /// \brief Maximum number of addressable logical processors sharing
        /// this cache
        unsigned max_proc_sharing() const { return max_proc_sharing_; }

        /// \brief Maximum number of addressable processor cores in the
        /// physical package
        unsigned max_proc_physical() const { return max_proc_physical_; }

        /// \brief Coherency line size in byte
        unsigned line_size() const { return line_size_; }

        /// \brief Physical line partitions
        unsigned partitions() const { return partitions_; }

        /// \brief Ways of associative
        unsigned ways() const { return ways_; }

        /// \brief Number of sets
        unsigned sets() const { return sets_; }

        /// \brief Cache size in byte
        unsigned size() const { return size_; }

        /// \brief Self initializing cache (does not need SW initialization)
        bool self_initializing() const { return self_initializing_; }

        /// \brief Fully associative cache
        bool fully_associative() const { return fully_associative_; }

        /// \brief Write-back invalidate/invalidate behavior on lower level
        /// caches
        bool wbinvd() const { return wbinvd_; }

        /// \brief Cache inclusiveness
        bool inclusiveness() const { return inclusiveness_; }

        /// \brief Complex cache indexing
        bool complex_indexing() const { return complex_indexing_; }

        private:
        CPUIDCacheType type_;
        unsigned level_;
        unsigned max_proc_sharing_;
        unsigned max_proc_physical_;
        unsigned line_size_;
        unsigned partitions_;
        unsigned ways_;
        unsigned sets_;
        unsigned size_;
        bool self_initializing_;
        bool fully_associative_;
        bool wbinvd_;
        bool inclusiveness_;
        bool complex_indexing_;
    }; // struct cache_param_type

    /// \brief Get CPU feature using CPUID
    template <typename CharT, typename Traits>
    static std::basic_ostream<CharT, Traits> &info(
        std::basic_ostream<CharT, Traits> &os)
    {
        if (!os.good())
            return os;

        print_equal(os);
        os << "Vendor ID                  " << vendor() << '\n';
        if (max_eax_ext() >= ext0_ + 4U)
            os << "Processor brand            " << brand() << '\n';
        if (max_eax() >= 0x16) {
            os << "Base frequency (MHz)       " << base_freq() << '\n';
            os << "Maximum frequency (MHz)    " << max_freq() << '\n';
            os << "Bus frequency (MHz)        " << bus_freq() << '\n';
        }
        if (max_eax() >= 0x04) {
            print_equal(os);
            os << "Deterministic cache parameters\n";
            print_dash(os);
            print_cache(os);
        }
        print_equal(os);
        print_feature(os);
        print_equal(os);

        return os;
    }

    /// \brief Get the CPUID information stored in EAX, EBX, ECX and EDX,
    /// given
    /// input EAX and ECX values.
    ///
    /// \note Unlike the `cpuid(eax, ecx)` call, the results are cached.
    /// Therefore, for subsequent calls of this function with the same value
    /// of
    /// EAX and ECX, the CPUID instruction will not be called.
    template <unsigned EAX, unsigned ECX>
    static const reg_type &info()
    {
        static reg_type reg(info_dispatch<EAX, ECX>(
            std::integral_constant < bool, EAX == 0x00 || EAX == ext0_ > (),
            std::integral_constant < bool, EAX<ext0_>()));

        return reg;
    }

    /// \brief Maximum calling parameter EAX (EAX = 0x00; EAX)
    static unsigned max_eax() { return std::get<0>(info<0x00, 0x00>()); }

    /// \brief Maximum extended calling parameter EAX (EAX = 0x80000000; EAX)
    static unsigned max_eax_ext() { return std::get<0>(info<ext0_, 0x00>()); }

    /// \brief Vendor ID (EAX = 0x00; EBX, EDX, ECX)
    static std::string vendor()
    {
        reg_type reg(info<0x00, 0x00>());
        const unsigned *uptr = reg.data();
        char str[sizeof(unsigned) * 3 + 1] = {'\0'};
        std::memcpy(str + sizeof(unsigned) * 0, uptr + 1, sizeof(unsigned));
        std::memcpy(str + sizeof(unsigned) * 1, uptr + 3, sizeof(unsigned));
        std::memcpy(str + sizeof(unsigned) * 2, uptr + 2, sizeof(unsigned));

        return std::string(str);
    }

    /// \brief Processor brand string (EAX = 0x80000002,0x80000003,0x80000004;
    /// EAX, EBX, ECX, EDX)
    static std::string brand()
    {
        reg_type reg2(info<ext0_ + 2U, 0>());
        reg_type reg3(info<ext0_ + 3U, 0>());
        reg_type reg4(info<ext0_ + 4U, 0>());
        const std::size_t reg_size = sizeof(unsigned) * 4;
        char str[reg_size * 3] = {'\0'};
        std::memcpy(str + reg_size * 0, reg2.data(), reg_size);
        std::memcpy(str + reg_size * 1, reg3.data(), reg_size);
        std::memcpy(str + reg_size * 2, reg4.data(), reg_size);

        return std::string(str);
    }

    /// \brief Get the number of caches
    static unsigned cache_param_num()
    {
        reg_type reg;
        unsigned ecx = 0x00;
        while (true) {
            cpuid(0x04, ecx, reg.data());
            if (extract_bits<4, 0>(std::get<0>(reg)) == 0)
                break;
            ++ecx;
        }

        return ecx;
    }

    /// \brief Get the cache parameters (EAX = 0x04; EAX, EBX, ECX, EDX)
    ///
    /// \note The maximum of the `cache_index` parameter
    /// `cache_param_num() - 1`
    static cache_param_type cache_param(unsigned cache_index)
    {
        reg_type reg;
        cpuid(0x04, cache_index, reg.data());

        return cache_param_type(reg);
    }

    /// \brief Intel Turbo Boost (EAX = 0x06; EAX[1])
    static bool intel_turbo_boost()
    {
        return test_bit<1>(std::get<0>(info<0x06, 0x00>()));
    }

    /// \brief Base frequency in MHz (EAX = 0x16; EAX[15:0])
    static unsigned base_freq()
    {
        return extract_bits<15, 0>(std::get<0>(info<0x16, 0x00>()));
    }

    /// \brief Maximum frequency in MHz (EAX = 0x16; EBX[15:0])
    static unsigned max_freq()
    {
        return extract_bits<15, 0>(std::get<1>(info<0x16, 0x00>()));
    }

    /// \brief Bus (reference) frequency in MHz (EAX = 0x16; ECX[15:0])
    static unsigned bus_freq()
    {
        return extract_bits<15, 0>(std::get<2>(info<0x16, 0x00>()));
    }

    /// \brief CPU feature
    template <CPUIDFeature Feat>
    static bool has_feature()
    {
        return test_bit<CPUIDFeatureInfo<Feat>::bit>(
            std::get<CPUIDFeatureInfo<Feat>::index>(
                info<CPUIDFeatureInfo<Feat>::eax,
                    CPUIDFeatureInfo<Feat>::ecx>()));
    }

    private:
    static constexpr const unsigned ext0_ = 0x80000000U;

    template <unsigned, unsigned>
    static reg_type info_dispatch(std::true_type, std::true_type)
    {
        reg_type reg;
        cpuid(0x00, 0x00, reg.data());

        return reg;
    }

    template <unsigned, unsigned>
    static reg_type info_dispatch(std::true_type, std::false_type)
    {
        reg_type reg;
        cpuid(ext0_, 0x00, reg.data());

        return reg;
    }

    template <unsigned EAX, unsigned ECX, bool Basic>
    static reg_type info_dispatch(
        std::false_type, std::integral_constant<bool, Basic>)
    {
        reg_type reg(info_dispatch<EAX, ECX>(
            std::true_type(), std::integral_constant<bool, Basic>()));

        if (EAX > std::get<0>(reg))
            reg.fill(0);
        else
            cpuid(EAX, ECX, reg.data());

        return reg;
    }

    template <unsigned Hi, unsigned Lo>
    static unsigned extract_bits(unsigned val)
    {
        return (val << (31U - Hi)) >> (31U - Hi + Lo);
    }

    template <unsigned Bit>
    static bool test_bit(unsigned val)
    {
        return (val & (1U << Bit)) != 0;
    }

    template <typename CharT, typename Traits>
    static void print_equal(std::basic_ostream<CharT, Traits> &os)
    {
        os << std::string(90, '=') << '\n';
    }

    template <typename CharT, typename Traits>
    static void print_dash(std::basic_ostream<CharT, Traits> &os)
    {
        os << std::string(90, '-') << '\n';
    }

    template <typename CharT, typename Traits>
    static void print_cache(std::basic_ostream<CharT, Traits> &os)
    {
        std::vector<cache_param_type> caches;
        unsigned max_ecx = cache_param_num();
        for (unsigned ecx = 0x00; ecx != max_ecx; ++ecx)
            caches.push_back(cache_param(ecx));

        const std::size_t fix = 12;
        std::stringstream ss;

        os << "Cache level                ";
        for (std::size_t i = 0; i != caches.size(); ++i)
            os << std::setw(fix) << caches[i].level();
        os << '\n';

        os << "Cache type                 ";
        for (std::size_t i = 0; i != caches.size(); ++i) {
            switch (caches[i].type()) {
                case CPUIDCacheTypeNull: break;
                case CPUIDCacheTypeData:
                    os << std::setw(fix) << "Data";
                    break;
                case CPUIDCacheTypeInstruction:
                    os << std::setw(fix) << "Instruction";
                    break;
                case CPUIDCacheTypeUnified:
                    os << std::setw(fix) << "Unified";
                    break;
            }
        }
        os << '\n';

        os << "Cache size (byte)          ";
        for (std::size_t i = 0; i != caches.size(); ++i) {
            unsigned b = caches[i].size();
            ss.str(std::string());
            if (b < 1024) {
                ss << b;
            } else if ((b /= 1024) < 1024) {
                ss << b << "K";
            } else if ((b /= 1024) < 1024) {
                ss << b << "M";
            } else {
                ss << b / 1024 << "G";
            }
            os << std::setw(fix) << ss.str();
        }
        os << '\n';

        os << "Maximum Proc sharing       ";
        for (std::size_t i = 0; i != caches.size(); ++i)
            os << std::setw(fix) << caches[i].max_proc_sharing();
        os << '\n';

        os << "Maximum Proc physical      ";
        for (std::size_t i = 0; i != caches.size(); ++i)
            os << std::setw(fix) << caches[i].max_proc_physical();
        os << '\n';

        os << "Coherency line size (byte) ";
        for (std::size_t i = 0; i != caches.size(); ++i)
            os << std::setw(fix) << caches[i].line_size();
        os << '\n';

        os << "Physical line partitions   ";
        for (std::size_t i = 0; i != caches.size(); ++i)
            os << std::setw(fix) << caches[i].partitions();
        os << '\n';

        os << "Ways of associative        ";
        for (std::size_t i = 0; i != caches.size(); ++i)
            os << std::setw(fix) << caches[i].ways();
        os << '\n';

        os << "Number of sets             ";
        for (std::size_t i = 0; i != caches.size(); ++i)
            os << std::setw(fix) << caches[i].sets();
        os << '\n';

        os << "Self initializing          ";
        for (std::size_t i = 0; i != caches.size(); ++i) {
            os << std::setw(fix)
               << (caches[i].self_initializing() ? "Yes" : "No");
        }
        os << '\n';

        os << "Fully associative          ";
        for (std::size_t i = 0; i != caches.size(); ++i) {
            os << std::setw(fix)
               << (caches[i].fully_associative() ? "Yes" : "No");
        }
        os << '\n';

        os << "Write-back invalidate      ";
        for (std::size_t i = 0; i != caches.size(); ++i) {
            os << std::setw(fix) << (caches[i].wbinvd() ? "Yes" : "No");
        }
        os << '\n';

        os << "Cache inclusiveness        ";
        for (std::size_t i = 0; i != caches.size(); ++i) {
            os << std::setw(fix)
               << (caches[i].inclusiveness() ? "Yes" : "No");
        }
        os << '\n';

        os << "Complex cache indexing     ";
        for (std::size_t i = 0; i != caches.size(); ++i) {
            os << std::setw(fix)
               << (caches[i].complex_indexing() ? "Yes" : "No");
        }
        os << '\n';
    }

    template <typename CharT, typename Traits>
    static void print_feature(std::basic_ostream<CharT, Traits> &os)
    {
        if (max_eax() >= 0x01) {
            std::vector<std::string> feats;
            feature_str1(feats);
            if (feats.size() != 0) {
                os << "Processor info and features\n";
                print_dash(os);
                print_feature(os, feats);
            }
        }
        if (max_eax() >= 0x07) {
            std::vector<std::string> feats;
            feature_str7(feats);
            if (feats.size() != 0) {
                print_equal(os);
                os << "Extended features\n";
                print_dash(os);
                print_feature(os, feats);
            }
        }
        if (max_eax_ext() >= ext0_ + 1U) {
            std::vector<std::string> feats;
            feature_str_ext1(feats);
            if (feats.size() != 0) {
                print_equal(os);
                os << "Extended processor info and features\n";
                print_dash(os);
                print_feature(os, feats);
            }
        }
    }

    template <typename CharT, typename Traits>
    static void print_feature(std::basic_ostream<CharT, Traits> &os,
        std::vector<std::string> &feats)
    {
        std::sort(feats.begin(), feats.end());
        for (std::size_t i = 0; i != feats.size(); ++i) {
            print_feat(os, feats[i], 15);
            if (i % 6 == 5 || i == feats.size() - 1)
                os << '\n';
        }
    }

    template <typename CharT, typename Traits>
    static void print_feat(std::basic_ostream<CharT, Traits> &os,
        std::string &str, std::size_t fix)
    {
        os << str;
        if (str.size() < fix)
            os << std::string(fix - str.size(), ' ');
    }

    static void feature_str1(std::vector<std::string> &feats)
    {
        feature_str<CPUIDFeatureSSE3>(feats);
        feature_str<CPUIDFeaturePCLMULQDQ>(feats);
        feature_str<CPUIDFeatureDTES64>(feats);
        feature_str<CPUIDFeatureMONITOR>(feats);
        feature_str<CPUIDFeatureDS_CPL>(feats);
        feature_str<CPUIDFeatureVMX>(feats);
        feature_str<CPUIDFeatureSMX>(feats);
        feature_str<CPUIDFeatureEST>(feats);
        feature_str<CPUIDFeatureTM2>(feats);
        feature_str<CPUIDFeatureSSSE3>(feats);
        feature_str<CPUIDFeatureCNXT_ID>(feats);
        feature_str<CPUIDFeatureFMA>(feats);
        feature_str<CPUIDFeatureCX16>(feats);
        feature_str<CPUIDFeatureXTPR>(feats);
        feature_str<CPUIDFeaturePDCM>(feats);
        feature_str<CPUIDFeaturePCID>(feats);
        feature_str<CPUIDFeatureDCA>(feats);
        feature_str<CPUIDFeatureSSE4_1>(feats);
        feature_str<CPUIDFeatureSSE4_2>(feats);
        feature_str<CPUIDFeatureX2APIC>(feats);
        feature_str<CPUIDFeatureMOVBE>(feats);
        feature_str<CPUIDFeaturePOPCNT>(feats);
        feature_str<CPUIDFeatureTSC_DEADLINE>(feats);
        feature_str<CPUIDFeatureAES>(feats);
        feature_str<CPUIDFeatureXSAVE>(feats);
        feature_str<CPUIDFeatureOSXSAVE>(feats);
        feature_str<CPUIDFeatureAVX>(feats);
        feature_str<CPUIDFeatureF16C>(feats);
        feature_str<CPUIDFeatureRDRAND>(feats);
        feature_str<CPUIDFeatureHYPERVISOR>(feats);

        feature_str<CPUIDFeatureFPU>(feats);
        feature_str<CPUIDFeatureVME>(feats);
        feature_str<CPUIDFeatureDE>(feats);
        feature_str<CPUIDFeaturePSE>(feats);
        feature_str<CPUIDFeatureTSC>(feats);
        feature_str<CPUIDFeatureMSR>(feats);
        feature_str<CPUIDFeaturePAE>(feats);
        feature_str<CPUIDFeatureMCE>(feats);
        feature_str<CPUIDFeatureCX8>(feats);
        feature_str<CPUIDFeatureAPIC>(feats);
        feature_str<CPUIDFeatureSEP>(feats);
        feature_str<CPUIDFeatureMTRR>(feats);
        feature_str<CPUIDFeaturePGE>(feats);
        feature_str<CPUIDFeatureMCA>(feats);
        feature_str<CPUIDFeatureCMOV>(feats);
        feature_str<CPUIDFeaturePAT>(feats);
        feature_str<CPUIDFeaturePSE_36>(feats);
        feature_str<CPUIDFeaturePSN>(feats);
        feature_str<CPUIDFeatureCLFSH>(feats);
        feature_str<CPUIDFeatureDS>(feats);
        feature_str<CPUIDFeatureACPI>(feats);
        feature_str<CPUIDFeatureMMX>(feats);
        feature_str<CPUIDFeatureFXSR>(feats);
        feature_str<CPUIDFeatureSSE>(feats);
        feature_str<CPUIDFeatureSSE2>(feats);
        feature_str<CPUIDFeatureSS>(feats);
        feature_str<CPUIDFeatureHTT>(feats);
        feature_str<CPUIDFeatureTM>(feats);
        feature_str<CPUIDFeatureIA64>(feats);
        feature_str<CPUIDFeaturePBE>(feats);
    }

    static void feature_str7(std::vector<std::string> &feats)
    {
        feature_str<CPUIDFeatureFSGSBASE>(feats);
        feature_str<CPUIDFeatureBMI1>(feats);
        feature_str<CPUIDFeatureHLE>(feats);
        feature_str<CPUIDFeatureAVX2>(feats);
        feature_str<CPUIDFeatureSMEP>(feats);
        feature_str<CPUIDFeatureBMI2>(feats);
        feature_str<CPUIDFeatureERMS>(feats);
        feature_str<CPUIDFeatureINVPCID>(feats);
        feature_str<CPUIDFeatureRTM>(feats);
        feature_str<CPUIDFeatureMPX>(feats);
        feature_str<CPUIDFeatureAVX512F>(feats);
        feature_str<CPUIDFeatureAVX512DQ>(feats);
        feature_str<CPUIDFeatureRDSEED>(feats);
        feature_str<CPUIDFeatureADX>(feats);
        feature_str<CPUIDFeatureSMAP>(feats);
        feature_str<CPUIDFeatureAVX512IFMA52>(feats);
        feature_str<CPUIDFeatureCLFLUSHOPT>(feats);
        feature_str<CPUIDFeatureINTEL_TRACE>(feats);
        feature_str<CPUIDFeatureAVX512PF>(feats);
        feature_str<CPUIDFeatureAVX512ER>(feats);
        feature_str<CPUIDFeatureAVX512CD>(feats);
        feature_str<CPUIDFeatureSHA>(feats);
        feature_str<CPUIDFeatureAVX512BW>(feats);
        feature_str<CPUIDFeatureAVX512VL>(feats);

        feature_str<CPUIDFeaturePREFETCHWT1>(feats);
        feature_str<CPUIDFeatureAVX512VBMI>(feats);
    }

    static void feature_str_ext1(std::vector<std::string> &feats)
    {
        feature_str<CPUIDFeatureExtLAHF_LM>(feats);
        feature_str<CPUIDFeatureExtCMP_LEGACY>(feats);
        feature_str<CPUIDFeatureExtSVM>(feats);
        feature_str<CPUIDFeatureExtEXTAPIC>(feats);
        feature_str<CPUIDFeatureExtCR8_LEGACY>(feats);
        feature_str<CPUIDFeatureExtABM>(feats);
        feature_str<CPUIDFeatureExtSSE4A>(feats);
        feature_str<CPUIDFeatureExtMISALIGNSSE>(feats);
        feature_str<CPUIDFeatureExt3DNOWPREFETCH>(feats);
        feature_str<CPUIDFeatureExtOSVW>(feats);
        feature_str<CPUIDFeatureExtIBS>(feats);
        feature_str<CPUIDFeatureExtXOP>(feats);
        feature_str<CPUIDFeatureExtSKINIT>(feats);
        feature_str<CPUIDFeatureExtWDT>(feats);
        feature_str<CPUIDFeatureExtLWP>(feats);
        feature_str<CPUIDFeatureExtFMA4>(feats);
        feature_str<CPUIDFeatureExtTCE>(feats);
        feature_str<CPUIDFeatureExtNODEID_MSR>(feats);
        feature_str<CPUIDFeatureExtTBM>(feats);
        feature_str<CPUIDFeatureExtTOPOEXT>(feats);
        feature_str<CPUIDFeatureExtPERFCTR_CORE>(feats);
        feature_str<CPUIDFeatureExtPERFCTR_NB>(feats);
        feature_str<CPUIDFeatureExtDBX>(feats);
        feature_str<CPUIDFeatureExtPERFTSC>(feats);
        feature_str<CPUIDFeatureExtPCX_L2I>(feats);

        feature_str<CPUIDFeatureExtFPU>(feats);
        feature_str<CPUIDFeatureExtVME>(feats);
        feature_str<CPUIDFeatureExtDE>(feats);
        feature_str<CPUIDFeatureExtPSE>(feats);
        feature_str<CPUIDFeatureExtTSC>(feats);
        feature_str<CPUIDFeatureExtMSR>(feats);
        feature_str<CPUIDFeatureExtPAE>(feats);
        feature_str<CPUIDFeatureExtMCE>(feats);
        feature_str<CPUIDFeatureExtCX8>(feats);
        feature_str<CPUIDFeatureExtAPIC>(feats);
        feature_str<CPUIDFeatureExtSYSCALL>(feats);
        feature_str<CPUIDFeatureExtMTRR>(feats);
        feature_str<CPUIDFeatureExtPGE>(feats);
        feature_str<CPUIDFeatureExtMCA>(feats);
        feature_str<CPUIDFeatureExtCMOV>(feats);
        feature_str<CPUIDFeatureExtPAT>(feats);
        feature_str<CPUIDFeatureExtPSE36>(feats);
        feature_str<CPUIDFeatureExtMP>(feats);
        feature_str<CPUIDFeatureExtNX>(feats);
        feature_str<CPUIDFeatureExtMMX>(feats);
        feature_str<CPUIDFeatureExtMMXEXT>(feats);
        feature_str<CPUIDFeatureExtFXSR>(feats);
        feature_str<CPUIDFeatureExtFXSR_OPT>(feats);
        feature_str<CPUIDFeatureExtGBPAGES>(feats);
        feature_str<CPUIDFeatureExtRDTSCP>(feats);
        feature_str<CPUIDFeatureExtLM>(feats);
        feature_str<CPUIDFeatureExt3DNOWEXT>(feats);
        feature_str<CPUIDFeatureExt3DNOW>(feats);
    }

    template <CPUIDFeature Feat>
    static void feature_str(std::vector<std::string> &feats)
    {
        if (has_feature<Feat>())
            feats.push_back(CPUIDFeatureInfo<Feat>::str());
    }
}; // class CPUID

/// \brief Query CPU information using CPUID
/// \ingroup CPUID
template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os, const CPUID &)
{
    return CPUID::info(os);
}

} // namespace vsmc

#endif // VSMC_UTILITY_CPUID_HPP
