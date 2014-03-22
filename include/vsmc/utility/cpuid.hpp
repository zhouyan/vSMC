#ifndef VSMC_UTILITY_CPUID_HPP
#define VSMC_UTILITY_CPUID_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/utility/array.hpp>
#include <map>

#define VSMC_STATIC_ASSERT_UTILITY_CPUID_EAX(EAX, func) \
    VSMC_STATIC_ASSERT(((EAX >= 0x00U && EAX <= 0x0FU) ||                    \
            (EAX >= 0x80000000U && EAX <= 0x8000000FU)),                     \
            USE_CPUID_##func##_WITH_INVALID_EAX_VALUE)

#define VSMC_RUNTIME_ASSERT_UTILITY_CPUID_EAX(eax, func) \
    VSMC_RUNTIME_ASSERT(((eax >= 0x00U && eax <= 0x0FU) ||                   \
            (eax >= 0x80000000U && eax <= 0x8000000FU)),                     \
            ("USE CPUID::" #func " WITH INVALID INPUT EAX VALUE"))

#define VSMC_DEFINE_CPUID_FEATURE_INFO(feat, eax_val, reg_val, bit) \
template<> struct CPUIDFeatureInfo< CPUIDFeature##feat >                     \
{                                                                            \
    static std::string name () {return std::string(#feat);}                  \
    static VSMC_CONSTEXPR const unsigned eax = eax_val##U;                   \
    static VSMC_CONSTEXPR const unsigned mask = 1 << bit;                    \
    static VSMC_CONSTEXPR const std::size_t reg = reg_val;                   \
};

#define VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(feat, eax_val, reg_val, bit) \
template<> struct CPUIDFeatureInfo< CPUIDFeatureExt##feat >                  \
{                                                                            \
    static std::string name () {return std::string(#feat);}                  \
    static VSMC_CONSTEXPR const unsigned eax = eax_val##U;                   \
    static VSMC_CONSTEXPR const unsigned mask = 1 << bit;                    \
    static VSMC_CONSTEXPR const std::size_t reg = reg_val;                   \
};

namespace vsmc {

/// \brief Basic CPU features
/// \ingroup CPUID
enum CPUIDFeature
{
    CPUIDFeatureFPU,             ///< EAX:0x00000001 EDX:00
    CPUIDFeatureVME,             ///< EAX:0x00000001 EDX:01
    CPUIDFeatureDE,              ///< EAX:0x00000001 EDX:02
    CPUIDFeaturePSE,             ///< EAX:0x00000001 EDX:03
    CPUIDFeatureTSC,             ///< EAX:0x00000001 EDX:04
    CPUIDFeatureMSR,             ///< EAX:0x00000001 EDX:05
    CPUIDFeaturePAE,             ///< EAX:0x00000001 EDX:06
    CPUIDFeatureMCE,             ///< EAX:0x00000001 EDX:07
    CPUIDFeatureCX8,             ///< EAX:0x00000001 EDX:08
    CPUIDFeatureAPIC,            ///< EAX:0x00000001 EDX:09
    CPUIDFeatureSEP,             ///< EAX:0x00000001 EDX:11
    CPUIDFeatureMTRR,            ///< EAX:0x00000001 EDX:12
    CPUIDFeaturePGE,             ///< EAX:0x00000001 EDX:13
    CPUIDFeatureMCA,             ///< EAX:0x00000001 EDX:14
    CPUIDFeatureCMOV,            ///< EAX:0x00000001 EDX:15
    CPUIDFeaturePAT,             ///< EAX:0x00000001 EDX:16
    CPUIDFeaturePSE_36,          ///< EAX:0x00000001 EDX:17
    CPUIDFeaturePSN,             ///< EAX:0x00000001 EDX:18
    CPUIDFeatureCLFSH,           ///< EAX:0x00000001 EDX:19
    CPUIDFeatureDS,              ///< EAX:0x00000001 EDX:21
    CPUIDFeatureACPI,            ///< EAX:0x00000001 EDX:22
    CPUIDFeatureMMX,             ///< EAX:0x00000001 EDX:23
    CPUIDFeatureFXSR,            ///< EAX:0x00000001 EDX:24
    CPUIDFeatureSSE,             ///< EAX:0x00000001 EDX:25
    CPUIDFeatureSSE2,            ///< EAX:0x00000001 EDX:26
    CPUIDFeatureSS,              ///< EAX:0x00000001 EDX:27
    CPUIDFeatureHTT,             ///< EAX:0x00000001 EDX:28
    CPUIDFeatureTM,              ///< EAX:0x00000001 EDX:29
    CPUIDFeatureIA64,            ///< EAX:0x00000001 EDX:30
    CPUIDFeaturePBE,             ///< EAX:0x00000001 EDX:31

    CPUIDFeatureSSE3,            ///< EAX:0x00000001 ECX:00
    CPUIDFeaturePCLMULQDQ,       ///< EAX:0x00000001 ECX:01
    CPUIDFeatureDTES64,          ///< EAX:0x00000001 ECX:02
    CPUIDFeatureMONITOR,         ///< EAX:0x00000001 ECX:03
    CPUIDFeatureDS_CPL,          ///< EAX:0x00000001 ECX:04
    CPUIDFeatureVMX,             ///< EAX:0x00000001 ECX:05
    CPUIDFeatureSMX,             ///< EAX:0x00000001 ECX:06
    CPUIDFeatureEST,             ///< EAX:0x00000001 ECX:07
    CPUIDFeatureTM2,             ///< EAX:0x00000001 ECX:08
    CPUIDFeatureSSSE3,           ///< EAX:0x00000001 ECX:09
    CPUIDFeatureCNXT_ID,         ///< EAX:0x00000001 ECX:10
    CPUIDFeatureFMA,             ///< EAX:0x00000001 ECX:12
    CPUIDFeatureCX16,            ///< EAX:0x00000001 ECX:13
    CPUIDFeatureXTPR,            ///< EAX:0x00000001 ECX:14
    CPUIDFeaturePDCM,            ///< EAX:0x00000001 ECX:15
    CPUIDFeaturePCID,            ///< EAX:0x00000001 ECX:17
    CPUIDFeatureDCA,             ///< EAX:0x00000001 ECX:18
    CPUIDFeatureSSE4_1,          ///< EAX:0x00000001 ECX:19
    CPUIDFeatureSSE4_2,          ///< EAX:0x00000001 ECX:20
    CPUIDFeatureX2APIC,          ///< EAX:0x00000001 ECX:21
    CPUIDFeatureMOVBE,           ///< EAX:0x00000001 ECX:22
    CPUIDFeaturePOPCNT,          ///< EAX:0x00000001 ECX:23
    CPUIDFeatureTSC_DEADLINE,    ///< EAX:0x00000001 ECX:24
    CPUIDFeatureAES,             ///< EAX:0x00000001 ECX:25
    CPUIDFeatureXSAVE,           ///< EAX:0x00000001 ECX:26
    CPUIDFeatureOSXSAVE,         ///< EAX:0x00000001 ECX:27
    CPUIDFeatureAVX,             ///< EAX:0x00000001 ECX:28
    CPUIDFeatureF16C,            ///< EAX:0x00000001 ECX:29
    CPUIDFeatureRDRND,           ///< EAX:0x00000001 ECX:30
    CPUIDFeatureHYPERVISOR,      ///< EAX:0x00000001 ECX:31

    CPUIDFeatureFSGSBASE,        ///< EAX:0x00000007 EBX:00
    CPUIDFeatureBMI1,            ///< EAX:0x00000007 EBX:03
    CPUIDFeatureHLE,             ///< EAX:0x00000007 EBX:04
    CPUIDFeatureAVX2,            ///< EAX:0x00000007 EBX:05
    CPUIDFeatureSMEP,            ///< EAX:0x00000007 EBX:07
    CPUIDFeatureBMI2,            ///< EAX:0x00000007 EBX:08
    CPUIDFeatureERMS,            ///< EAX:0x00000007 EBX:09
    CPUIDFeatureINVPCID,         ///< EAX:0x00000007 EBX:10
    CPUIDFeatureRTM,             ///< EAX:0x00000007 EBX:11
    CPUIDFeatureMPX,             ///< EAX:0x00000007 EBX:14
    CPUIDFeatureAVX512F,         ///< EAX:0x00000007 EBX:16
    CPUIDFeatureRDSEED,          ///< EAX:0x00000007 EBX:18
    CPUIDFeatureADX,             ///< EAX:0x00000007 EBX:19
    CPUIDFeatureSMAP,            ///< EAX:0x00000007 EBX:20
    CPUIDFeatureINTEL_TRACE,     ///< EAX:0x00000007 EBX:25
    CPUIDFeatureAVX512PF,        ///< EAX:0x00000007 EBX:26
    CPUIDFeatureAVX512ER,        ///< EAX:0x00000007 EBX:27
    CPUIDFeatureAVX512CD,        ///< EAX:0x00000007 EBX:28
    CPUIDFeatureSHA,             ///< EAX:0x00000007 EBX:29

    CPUIDFeaturePREFETCHWT1,     ///< EAX:0x00000007 ECX:00

    CPUIDFeatureExtFPU,          ///< EAX:0x80000001 EDX:00
    CPUIDFeatureExtVME,          ///< EAX:0x80000001 EDX:01
    CPUIDFeatureExtDE,           ///< EAX:0x80000001 EDX:02
    CPUIDFeatureExtPSE,          ///< EAX:0x80000001 EDX:03
    CPUIDFeatureExtTSC,          ///< EAX:0x80000001 EDX:04
    CPUIDFeatureExtMSR,          ///< EAX:0x80000001 EDX:05
    CPUIDFeatureExtPAE,          ///< EAX:0x80000001 EDX:06
    CPUIDFeatureExtMCE,          ///< EAX:0x80000001 EDX:07
    CPUIDFeatureExtCX8,          ///< EAX:0x80000001 EDX:08
    CPUIDFeatureExtAPIC,         ///< EAX:0x80000001 EDX:09
    CPUIDFeatureExtSYSCALL,      ///< EAX:0x80000001 EDX:11
    CPUIDFeatureExtMTRR,         ///< EAX:0x80000001 EDX:12
    CPUIDFeatureExtPGE,          ///< EAX:0x80000001 EDX:13
    CPUIDFeatureExtMCA,          ///< EAX:0x80000001 EDX:14
    CPUIDFeatureExtCMOV,         ///< EAX:0x80000001 EDX:15
    CPUIDFeatureExtPAT,          ///< EAX:0x80000001 EDX:16
    CPUIDFeatureExtPSE36,        ///< EAX:0x80000001 EDX:17
    CPUIDFeatureExtMP,           ///< EAX:0x80000001 EDX:19
    CPUIDFeatureExtNX,           ///< EAX:0x80000001 EDX:20
    CPUIDFeatureExtMMXEXT,       ///< EAX:0x80000001 EDX:22
    CPUIDFeatureExtMMX,          ///< EAX:0x80000001 EDX:23
    CPUIDFeatureExtFXSR,         ///< EAX:0x80000001 EDX:24
    CPUIDFeatureExtFXSR_OPT,     ///< EAX:0x80000001 EDX:25
    CPUIDFeatureExtPDPE1GB,      ///< EAX:0x80000001 EDX:26
    CPUIDFeatureExtRDTSCP,       ///< EAX:0x80000001 EDX:27
    CPUIDFeatureExtLM,           ///< EAX:0x80000001 EDX:29
    CPUIDFeatureExt3DNOWEXT,     ///< EAX:0x80000001 EDX:30
    CPUIDFeatureExt3DNOW,        ///< EAX:0x80000001 EDX:31

    CPUIDFeatureExtLAHF_LM,      ///< EAX:0x80000001 ECX:00
    CPUIDFeatureExtCMP_LEGACY,   ///< EAX:0x80000001 ECX:01
    CPUIDFeatureExtSVM,          ///< EAX:0x80000001 ECX:02
    CPUIDFeatureExtEXTAPIC,      ///< EAX:0x80000001 ECX:03
    CPUIDFeatureExtCR8_LEGACY,   ///< EAX:0x80000001 ECX:04
    CPUIDFeatureExtABM,          ///< EAX:0x80000001 ECX:05
    CPUIDFeatureExtSSE4A,        ///< EAX:0x80000001 ECX:06
    CPUIDFeatureExtMISALIGNSSE,  ///< EAX:0x80000001 ECX:07
    CPUIDFeatureExt3DNOWPREFECT, ///< EAX:0x80000001 ECX:08
    CPUIDFeatureExtOSVW,         ///< EAX:0x80000001 ECX:09
    CPUIDFeatureExtIBS,          ///< EAX:0x80000001 ECX:10
    CPUIDFeatureExtXOP,          ///< EAX:0x80000001 ECX:11
    CPUIDFeatureExtSKINIT,       ///< EAX:0x80000001 ECX:12
    CPUIDFeatureExtWDT,          ///< EAX:0x80000001 ECX:13
    CPUIDFeatureExtLWP,          ///< EAX:0x80000001 ECX:15
    CPUIDFeatureExtFMA4,         ///< EAX:0x80000001 ECX:16
    CPUIDFeatureExtTCE,          ///< EAX:0x80000001 ECX:17
    CPUIDFeatureExtNODEID_MSR,   ///< EAX:0x80000001 ECX:19
    CPUIDFeatureExtTBM,          ///< EAX:0x80000001 ECX:21
    CPUIDFeatureExtTOPOEXT,      ///< EAX:0x80000001 ECX:22
    CPUIDFeatureExtPERFCTR_CORE, ///< EAX:0x80000001 ECX:23
    CPUIDFeatureExtPERFCTR_NB,   ///< EAX:0x80000001 ECX:24
    CPUIDFeatureExtDBX,          ///< EAX:0x80000001 ECX:26
    CPUIDFeatureExtPERFTSC,      ///< EAX:0x80000001 ECX:27
    CPUIDFeatureExtPCX_L2I,      ///< EAX:0x80000001 ECX:28
}; // enum CPUIDFeature

namespace internal {

template <CPUIDFeature> struct CPUIDFeatureInfo;

VSMC_DEFINE_CPUID_FEATURE_INFO(FPU,          1, 3,  0)
VSMC_DEFINE_CPUID_FEATURE_INFO(VME,          1, 3,  1)
VSMC_DEFINE_CPUID_FEATURE_INFO(DE,           1, 3,  2)
VSMC_DEFINE_CPUID_FEATURE_INFO(PSE,          1, 3,  3)
VSMC_DEFINE_CPUID_FEATURE_INFO(TSC,          1, 3,  4)
VSMC_DEFINE_CPUID_FEATURE_INFO(MSR,          1, 3,  5)
VSMC_DEFINE_CPUID_FEATURE_INFO(PAE,          1, 3,  6)
VSMC_DEFINE_CPUID_FEATURE_INFO(MCE,          1, 3,  7)
VSMC_DEFINE_CPUID_FEATURE_INFO(CX8,          1, 3,  8)
VSMC_DEFINE_CPUID_FEATURE_INFO(APIC,         1, 3,  9)
VSMC_DEFINE_CPUID_FEATURE_INFO(SEP,          1, 3, 11)
VSMC_DEFINE_CPUID_FEATURE_INFO(MTRR,         1, 3, 12)
VSMC_DEFINE_CPUID_FEATURE_INFO(PGE,          1, 3, 13)
VSMC_DEFINE_CPUID_FEATURE_INFO(MCA,          1, 3, 14)
VSMC_DEFINE_CPUID_FEATURE_INFO(CMOV,         1, 3, 15)
VSMC_DEFINE_CPUID_FEATURE_INFO(PAT,          1, 3, 16)
VSMC_DEFINE_CPUID_FEATURE_INFO(PSE_36,       1, 3, 17)
VSMC_DEFINE_CPUID_FEATURE_INFO(PSN,          1, 3, 18)
VSMC_DEFINE_CPUID_FEATURE_INFO(CLFSH,        1, 3, 19)
VSMC_DEFINE_CPUID_FEATURE_INFO(DS,           1, 3, 21)
VSMC_DEFINE_CPUID_FEATURE_INFO(ACPI,         1, 3, 22)
VSMC_DEFINE_CPUID_FEATURE_INFO(MMX,          1, 3, 23)
VSMC_DEFINE_CPUID_FEATURE_INFO(FXSR,         1, 3, 24)
VSMC_DEFINE_CPUID_FEATURE_INFO(SSE,          1, 3, 25)
VSMC_DEFINE_CPUID_FEATURE_INFO(SSE2,         1, 3, 26)
VSMC_DEFINE_CPUID_FEATURE_INFO(SS,           1, 3, 27)
VSMC_DEFINE_CPUID_FEATURE_INFO(HTT,          1, 3, 28)
VSMC_DEFINE_CPUID_FEATURE_INFO(TM,           1, 3, 29)
VSMC_DEFINE_CPUID_FEATURE_INFO(IA64,         1, 3, 30)
VSMC_DEFINE_CPUID_FEATURE_INFO(PBE,          1, 3, 31)

VSMC_DEFINE_CPUID_FEATURE_INFO(SSE3,         1, 2,  0)
VSMC_DEFINE_CPUID_FEATURE_INFO(PCLMULQDQ,    1, 2,  1)
VSMC_DEFINE_CPUID_FEATURE_INFO(DTES64,       1, 2,  2)
VSMC_DEFINE_CPUID_FEATURE_INFO(MONITOR,      1, 2,  3)
VSMC_DEFINE_CPUID_FEATURE_INFO(DS_CPL,       1, 2,  4)
VSMC_DEFINE_CPUID_FEATURE_INFO(VMX,          1, 2,  5)
VSMC_DEFINE_CPUID_FEATURE_INFO(SMX,          1, 2,  6)
VSMC_DEFINE_CPUID_FEATURE_INFO(EST,          1, 2,  7)
VSMC_DEFINE_CPUID_FEATURE_INFO(TM2,          1, 2,  8)
VSMC_DEFINE_CPUID_FEATURE_INFO(SSSE3,        1, 2,  9)
VSMC_DEFINE_CPUID_FEATURE_INFO(CNXT_ID,      1, 2, 10)
VSMC_DEFINE_CPUID_FEATURE_INFO(FMA,          1, 2, 12)
VSMC_DEFINE_CPUID_FEATURE_INFO(CX16,         1, 2, 13)
VSMC_DEFINE_CPUID_FEATURE_INFO(XTPR,         1, 2, 14)
VSMC_DEFINE_CPUID_FEATURE_INFO(PDCM,         1, 2, 15)
VSMC_DEFINE_CPUID_FEATURE_INFO(PCID,         1, 2, 17)
VSMC_DEFINE_CPUID_FEATURE_INFO(DCA,          1, 2, 18)
VSMC_DEFINE_CPUID_FEATURE_INFO(SSE4_1,       1, 2, 19)
VSMC_DEFINE_CPUID_FEATURE_INFO(SSE4_2,       1, 2, 20)
VSMC_DEFINE_CPUID_FEATURE_INFO(X2APIC,       1, 2, 21)
VSMC_DEFINE_CPUID_FEATURE_INFO(MOVBE,        1, 2, 22)
VSMC_DEFINE_CPUID_FEATURE_INFO(POPCNT,       1, 2, 23)
VSMC_DEFINE_CPUID_FEATURE_INFO(TSC_DEADLINE, 1, 2, 24)
VSMC_DEFINE_CPUID_FEATURE_INFO(AES,          1, 2, 25)
VSMC_DEFINE_CPUID_FEATURE_INFO(XSAVE,        1, 2, 26)
VSMC_DEFINE_CPUID_FEATURE_INFO(OSXSAVE,      1, 2, 27)
VSMC_DEFINE_CPUID_FEATURE_INFO(AVX,          1, 2, 28)
VSMC_DEFINE_CPUID_FEATURE_INFO(F16C,         1, 2, 29)
VSMC_DEFINE_CPUID_FEATURE_INFO(RDRND,        1, 2, 30)
VSMC_DEFINE_CPUID_FEATURE_INFO(HYPERVISOR,   1, 2, 31)

VSMC_DEFINE_CPUID_FEATURE_INFO(FSGSBASE,     7, 1,  0)
VSMC_DEFINE_CPUID_FEATURE_INFO(BMI1,         7, 1,  3)
VSMC_DEFINE_CPUID_FEATURE_INFO(HLE,          7, 1,  4)
VSMC_DEFINE_CPUID_FEATURE_INFO(AVX2,         7, 1,  5)
VSMC_DEFINE_CPUID_FEATURE_INFO(SMEP,         7, 1,  7)
VSMC_DEFINE_CPUID_FEATURE_INFO(BMI2,         7, 1,  8)
VSMC_DEFINE_CPUID_FEATURE_INFO(ERMS,         7, 1,  9)
VSMC_DEFINE_CPUID_FEATURE_INFO(INVPCID,      7, 1, 10)
VSMC_DEFINE_CPUID_FEATURE_INFO(RTM,          7, 1, 11)
VSMC_DEFINE_CPUID_FEATURE_INFO(MPX,          7, 1, 14)
VSMC_DEFINE_CPUID_FEATURE_INFO(AVX512F,      7, 1, 16)
VSMC_DEFINE_CPUID_FEATURE_INFO(RDSEED,       7, 1, 18)
VSMC_DEFINE_CPUID_FEATURE_INFO(ADX,          7, 1, 19)
VSMC_DEFINE_CPUID_FEATURE_INFO(SMAP,         7, 1, 20)
VSMC_DEFINE_CPUID_FEATURE_INFO(INTEL_TRACE,  7, 1, 25)
VSMC_DEFINE_CPUID_FEATURE_INFO(AVX512PF,     7, 1, 26)
VSMC_DEFINE_CPUID_FEATURE_INFO(AVX512ER,     7, 1, 27)
VSMC_DEFINE_CPUID_FEATURE_INFO(AVX512CD,     7, 1, 28)
VSMC_DEFINE_CPUID_FEATURE_INFO(SHA,          7, 1, 29)

VSMC_DEFINE_CPUID_FEATURE_INFO(PREFETCHWT1,  7, 3,  0)

VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(FPU,          0x80000001, 3,  0)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(VME,          0x80000001, 3,  1)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(DE,           0x80000001, 3,  2)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(PSE,          0x80000001, 3,  3)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(TSC,          0x80000001, 3,  4)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(MSR,          0x80000001, 3,  5)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(PAE,          0x80000001, 3,  6)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(MCE,          0x80000001, 3,  7)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(CX8,          0x80000001, 3,  8)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(APIC,         0x80000001, 3,  9)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(SYSCALL,      0x80000001, 3, 11)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(MTRR,         0x80000001, 3, 12)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(PGE,          0x80000001, 3, 13)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(MCA,          0x80000001, 3, 14)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(CMOV,         0x80000001, 3, 15)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(PAT,          0x80000001, 3, 16)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(PSE36,        0x80000001, 3, 17)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(MP,           0x80000001, 3, 19)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(NX,           0x80000001, 3, 20)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(MMXEXT,       0x80000001, 3, 22)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(MMX,          0x80000001, 3, 23)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(FXSR,         0x80000001, 3, 24)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(FXSR_OPT,     0x80000001, 3, 25)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(PDPE1GB,      0x80000001, 3, 26)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(RDTSCP,       0x80000001, 3, 27)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(LM,           0x80000001, 3, 29)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(3DNOWEXT,     0x80000001, 3, 30)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(3DNOW,        0x80000001, 3, 31)

VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(LAHF_LM,      0x80000001, 2,  0)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(CMP_LEGACY,   0x80000001, 2,  1)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(SVM,          0x80000001, 2,  2)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(EXTAPIC,      0x80000001, 2,  3)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(CR8_LEGACY,   0x80000001, 2,  4)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(ABM,          0x80000001, 2,  5)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(SSE4A,        0x80000001, 2,  6)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(MISALIGNSSE,  0x80000001, 2,  7)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(3DNOWPREFECT, 0x80000001, 2,  8)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(OSVW,         0x80000001, 2,  9)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(IBS,          0x80000001, 2, 10)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(XOP,          0x80000001, 2, 11)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(SKINIT,       0x80000001, 2, 12)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(WDT,          0x80000001, 2, 13)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(LWP,          0x80000001, 2, 15)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(FMA4,         0x80000001, 2, 16)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(TCE,          0x80000001, 2, 17)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(NODEID_MSR,   0x80000001, 2, 19)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(TBM,          0x80000001, 2, 21)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(TOPOEXT,      0x80000001, 2, 22)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(PERFCTR_CORE, 0x80000001, 2, 23)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(PERFCTR_NB,   0x80000001, 2, 24)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(DBX,          0x80000001, 2, 26)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(PERFTSC,      0x80000001, 2, 27)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(PCX_L2I,      0x80000001, 2, 28)

} // namespace vsmc::internal

/// \brief Query CPUID informations
/// \ingroup CPUID
class CPUID
{

    public :

    typedef std::map<unsigned, Array<unsigned, 4> > cpuid_map_type;

    /// \brief Initilaization
    ///
    /// \details
    /// Some member functions of CPUID are not thread-safe when it is called
    /// for the first time. After calling this function, all subsequent call to
    /// CPUID member functions will be thread-safe
    ///
    /// \return true if initialization is successful. If false is returned,
    /// then other member functions shall not be used.
    static bool initialize ()
    {
        bool init = false;

        init = cpuid_map_find<0x00U>() || init;
        init = cpuid_map_find<0x01U>() || init;
        init = cpuid_map_find<0x02U>() || init;
        init = cpuid_map_find<0x03U>() || init;
        init = cpuid_map_find<0x04U>() || init;
        init = cpuid_map_find<0x05U>() || init;
        init = cpuid_map_find<0x06U>() || init;
        init = cpuid_map_find<0x07U>() || init;
        init = cpuid_map_find<0x08U>() || init;
        init = cpuid_map_find<0x09U>() || init;
        init = cpuid_map_find<0x0AU>() || init;
        init = cpuid_map_find<0x0BU>() || init;
        init = cpuid_map_find<0x0CU>() || init;
        init = cpuid_map_find<0x0DU>() || init;
        init = cpuid_map_find<0x0EU>() || init;
        init = cpuid_map_find<0x0FU>() || init;

        init = cpuid_map_find<0x80000000U>() || init;
        init = cpuid_map_find<0x80000001U>() || init;
        init = cpuid_map_find<0x80000002U>() || init;
        init = cpuid_map_find<0x80000003U>() || init;
        init = cpuid_map_find<0x80000004U>() || init;
        init = cpuid_map_find<0x80000005U>() || init;
        init = cpuid_map_find<0x80000006U>() || init;
        init = cpuid_map_find<0x80000007U>() || init;
        init = cpuid_map_find<0x80000008U>() || init;
        init = cpuid_map_find<0x80000009U>() || init;
        init = cpuid_map_find<0x8000000AU>() || init;
        init = cpuid_map_find<0x8000000BU>() || init;
        init = cpuid_map_find<0x8000000CU>() || init;
        init = cpuid_map_find<0x8000000DU>() || init;
        init = cpuid_map_find<0x8000000EU>() || init;
        init = cpuid_map_find<0x8000000FU>() || init;

        return init;
    }

    /// \brief Get a map of possible calling parameter EAX and the values
    static const cpuid_map_type &cpuid_map ()
    {
        static cpuid_map_type cmap;
        static bool initialized = false;

        if (initialized)
            return cmap;

        const unsigned eax_max = query(0x00U).at<0>();
        cpuid_map_init<0x00U>(eax_max, cmap);
        cpuid_map_init<0x01U>(eax_max, cmap);
        cpuid_map_init<0x02U>(eax_max, cmap);
        cpuid_map_init<0x03U>(eax_max, cmap);
        cpuid_map_init<0x04U>(eax_max, cmap);
        cpuid_map_init<0x05U>(eax_max, cmap);
        cpuid_map_init<0x06U>(eax_max, cmap);
        cpuid_map_init<0x07U>(eax_max, cmap);
        cpuid_map_init<0x08U>(eax_max, cmap);
        cpuid_map_init<0x09U>(eax_max, cmap);
        cpuid_map_init<0x0AU>(eax_max, cmap);
        cpuid_map_init<0x0BU>(eax_max, cmap);
        cpuid_map_init<0x0CU>(eax_max, cmap);
        cpuid_map_init<0x0DU>(eax_max, cmap);
        cpuid_map_init<0x0EU>(eax_max, cmap);
        cpuid_map_init<0x0FU>(eax_max, cmap);

        const unsigned ext_max = query(0x80000000U).at<0>();
        cpuid_map_init<0x80000000U>(ext_max, cmap);
        cpuid_map_init<0x80000001U>(ext_max, cmap);
        cpuid_map_init<0x80000002U>(ext_max, cmap);
        cpuid_map_init<0x80000003U>(ext_max, cmap);
        cpuid_map_init<0x80000004U>(ext_max, cmap);
        cpuid_map_init<0x80000005U>(ext_max, cmap);
        cpuid_map_init<0x80000006U>(ext_max, cmap);
        cpuid_map_init<0x80000007U>(ext_max, cmap);
        cpuid_map_init<0x80000008U>(ext_max, cmap);
        cpuid_map_init<0x80000009U>(ext_max, cmap);
        cpuid_map_init<0x8000000AU>(ext_max, cmap);
        cpuid_map_init<0x8000000BU>(ext_max, cmap);
        cpuid_map_init<0x8000000CU>(ext_max, cmap);
        cpuid_map_init<0x8000000DU>(ext_max, cmap);
        cpuid_map_init<0x8000000EU>(ext_max, cmap);
        cpuid_map_init<0x8000000FU>(ext_max, cmap);

        initialized = true;

        return cmap;
    }

    /// \brief Query all informations
    template <typename CharT, typename Traits>
    static void info (std::basic_ostream<CharT, Traits> &os)
    {
        print_equal(os);
        os << "Vendor ID:                                  ";
        os << vendor_id() << '\n';
        print_dash(os);
        os << "Highest CPUID calling parameter:            ";
        os << max_eax() << '\n';
        os << "Highest CPUID calling parameter (extended): ";
        os << max_eax_ext() << '\n';

        print_equal(os);
        os << "Basic features" << '\n';
        print_dash(os);
        features(os);

        if (cpuid_map_find<0x80000001U>()) {
            print_equal(os);
            os << "Extended features" << '\n';
            print_dash(os);
            features_ext(os);
        }
        print_dash(os);
    }

    /// \brief Maximum of calling parameter eax
    static unsigned max_eax ()
    {return cpuid_map_citer<0x00U>()->second.at<0>();}

    /// \brief Maximum of extended calling parameter eax
    static unsigned max_eax_ext ()
    {
        if (!cpuid_map_find<0x80000000U>())
            return 0;

        return cpuid_map_citer<0x80000000U>()->second.at<0>();
    }

    /// \brief Test if a given calling parameter eax is supported
    template <unsigned EAX>
    static bool has_eax ()
    {
        VSMC_STATIC_ASSERT_UTILITY_CPUID_EAX(EAX, has_eax);
        return cpuid_map_find<EAX>();
    }

    /// \brief Test if a given clalling parameter eax is supported
    static bool has_eax (unsigned eax)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_CPUID_EAX(eax, has_eax);
        return cpuid_map().find(eax) != cpuid_map().end();
    }

    /// \brief Get a copy of EAX, EBX, ECX, EDX in an Array
    template <unsigned EAX>
    static cpuid_map_type::mapped_type value ()
    {
        VSMC_STATIC_ASSERT_UTILITY_CPUID_EAX(EAX, value);
        if (has_eax<EAX>())
            return cpuid_map_citer<EAX>()->second;
        else
            return cpuid_map_type::mapped_type();
    }

    /// \brief Get a copy of EAX, EBX, ECX, EDX in an Array
    static cpuid_map_type::mapped_type value (unsigned eax)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_CPUID_EAX(eax, value);
        cpuid_map_type::const_iterator citer = cpuid_map().find(eax);
        if (citer != cpuid_map().end())
            return citer->second;
        else
            return cpuid_map_type::mapped_type();
    }

    /// \brief Vendor ID
    static std::string vendor_id ()
    {
        if (!cpuid_map_find<0x00U>())
            return std::string();

        char str[sizeof(unsigned) * 3 + 1] = {'\0'};
        const unsigned *uptr = cpuid_map_citer<0x00U>()->second.data();
        std::memcpy(str + sizeof(unsigned) * 0, uptr + 1, sizeof(unsigned));
        std::memcpy(str + sizeof(unsigned) * 1, uptr + 3, sizeof(unsigned));
        std::memcpy(str + sizeof(unsigned) * 2, uptr + 2, sizeof(unsigned));

        return std::string(str);
    }

    /// \brief CPU feature
    template <CPUIDFeature Feat>
    static bool has_feature ()
    {
        if (!cpuid_map_find<internal::CPUIDFeatureInfo<Feat>::eax>())
            return false;

        return (cpuid_map_citer<internal::CPUIDFeatureInfo<Feat>::eax>()->
                second.template at<internal::CPUIDFeatureInfo<Feat>::reg>() &
                internal::CPUIDFeatureInfo<Feat>::mask) != 0;
    }

    /// \brief Query all basic features
    template <typename CharT, typename Traits>
    static void features (std::basic_ostream<CharT, Traits> &os)
    {
        std::vector<std::string> feats;
        feature_str<CPUIDFeatureACPI>        (feats);
        feature_str<CPUIDFeatureADX>         (feats);
        feature_str<CPUIDFeatureAES>         (feats);
        feature_str<CPUIDFeatureAPIC>        (feats);
        feature_str<CPUIDFeatureAVX2>        (feats);
        feature_str<CPUIDFeatureAVX512CD>    (feats);
        feature_str<CPUIDFeatureAVX512ER>    (feats);
        feature_str<CPUIDFeatureAVX512F>     (feats);
        feature_str<CPUIDFeatureAVX512PF>    (feats);
        feature_str<CPUIDFeatureAVX>         (feats);
        feature_str<CPUIDFeatureBMI1>        (feats);
        feature_str<CPUIDFeatureBMI2>        (feats);
        feature_str<CPUIDFeatureCLFSH>       (feats);
        feature_str<CPUIDFeatureCMOV>        (feats);
        feature_str<CPUIDFeatureCNXT_ID>     (feats);
        feature_str<CPUIDFeatureCX16>        (feats);
        feature_str<CPUIDFeatureCX8>         (feats);
        feature_str<CPUIDFeatureDCA>         (feats);
        feature_str<CPUIDFeatureDE>          (feats);
        feature_str<CPUIDFeatureDS>          (feats);
        feature_str<CPUIDFeatureDS_CPL>      (feats);
        feature_str<CPUIDFeatureDTES64>      (feats);
        feature_str<CPUIDFeatureERMS>        (feats);
        feature_str<CPUIDFeatureEST>         (feats);
        feature_str<CPUIDFeatureF16C>        (feats);
        feature_str<CPUIDFeatureFMA>         (feats);
        feature_str<CPUIDFeatureFPU>         (feats);
        feature_str<CPUIDFeatureFSGSBASE>    (feats);
        feature_str<CPUIDFeatureFXSR>        (feats);
        feature_str<CPUIDFeatureHLE>         (feats);
        feature_str<CPUIDFeatureHTT>         (feats);
        feature_str<CPUIDFeatureHYPERVISOR>  (feats);
        feature_str<CPUIDFeatureIA64>        (feats);
        feature_str<CPUIDFeatureINTEL_TRACE> (feats);
        feature_str<CPUIDFeatureINVPCID>     (feats);
        feature_str<CPUIDFeatureMCA>         (feats);
        feature_str<CPUIDFeatureMCE>         (feats);
        feature_str<CPUIDFeatureMMX>         (feats);
        feature_str<CPUIDFeatureMONITOR>     (feats);
        feature_str<CPUIDFeatureMOVBE>       (feats);
        feature_str<CPUIDFeatureMPX>         (feats);
        feature_str<CPUIDFeatureMSR>         (feats);
        feature_str<CPUIDFeatureMTRR>        (feats);
        feature_str<CPUIDFeatureOSXSAVE>     (feats);
        feature_str<CPUIDFeaturePAE>         (feats);
        feature_str<CPUIDFeaturePAT>         (feats);
        feature_str<CPUIDFeaturePBE>         (feats);
        feature_str<CPUIDFeaturePCID>        (feats);
        feature_str<CPUIDFeaturePCLMULQDQ>   (feats);
        feature_str<CPUIDFeaturePDCM>        (feats);
        feature_str<CPUIDFeaturePGE>         (feats);
        feature_str<CPUIDFeaturePOPCNT>      (feats);
        feature_str<CPUIDFeaturePREFETCHWT1> (feats);
        feature_str<CPUIDFeaturePSE>         (feats);
        feature_str<CPUIDFeaturePSE_36>      (feats);
        feature_str<CPUIDFeaturePSN>         (feats);
        feature_str<CPUIDFeatureRDRND>       (feats);
        feature_str<CPUIDFeatureRDSEED>      (feats);
        feature_str<CPUIDFeatureRTM>         (feats);
        feature_str<CPUIDFeatureSEP>         (feats);
        feature_str<CPUIDFeatureSHA>         (feats);
        feature_str<CPUIDFeatureSMAP>        (feats);
        feature_str<CPUIDFeatureSMEP>        (feats);
        feature_str<CPUIDFeatureSMX>         (feats);
        feature_str<CPUIDFeatureSS>          (feats);
        feature_str<CPUIDFeatureSSE2>        (feats);
        feature_str<CPUIDFeatureSSE3>        (feats);
        feature_str<CPUIDFeatureSSE4_1>      (feats);
        feature_str<CPUIDFeatureSSE4_2>      (feats);
        feature_str<CPUIDFeatureSSE>         (feats);
        feature_str<CPUIDFeatureSSSE3>       (feats);
        feature_str<CPUIDFeatureTM2>         (feats);
        feature_str<CPUIDFeatureTM>          (feats);
        feature_str<CPUIDFeatureTSC>         (feats);
        feature_str<CPUIDFeatureTSC_DEADLINE>(feats);
        feature_str<CPUIDFeatureVME>         (feats);
        feature_str<CPUIDFeatureVMX>         (feats);
        feature_str<CPUIDFeatureX2APIC>      (feats);
        feature_str<CPUIDFeatureXSAVE>       (feats);
        feature_str<CPUIDFeatureXTPR>        (feats);
        print_strvec(os, feats);
        os << std::flush;
    }

    /// \brief Query all extended features
    template <typename CharT, typename Traits>
    static void features_ext (std::basic_ostream<CharT, Traits> &os)
    {
        std::vector<std::string> feats;
        feature_str<CPUIDFeatureExt3DNOW>       (feats);
        feature_str<CPUIDFeatureExt3DNOWEXT>    (feats);
        feature_str<CPUIDFeatureExt3DNOWPREFECT>(feats);
        feature_str<CPUIDFeatureExtABM>         (feats);
        feature_str<CPUIDFeatureExtAPIC>        (feats);
        feature_str<CPUIDFeatureExtCMOV>        (feats);
        feature_str<CPUIDFeatureExtCMP_LEGACY>  (feats);
        feature_str<CPUIDFeatureExtCR8_LEGACY>  (feats);
        feature_str<CPUIDFeatureExtCX8>         (feats);
        feature_str<CPUIDFeatureExtDBX>         (feats);
        feature_str<CPUIDFeatureExtDE>          (feats);
        feature_str<CPUIDFeatureExtEXTAPIC>     (feats);
        feature_str<CPUIDFeatureExtFMA4>        (feats);
        feature_str<CPUIDFeatureExtFPU>         (feats);
        feature_str<CPUIDFeatureExtFXSR>        (feats);
        feature_str<CPUIDFeatureExtFXSR_OPT>    (feats);
        feature_str<CPUIDFeatureExtIBS>         (feats);
        feature_str<CPUIDFeatureExtLAHF_LM>     (feats);
        feature_str<CPUIDFeatureExtLM>          (feats);
        feature_str<CPUIDFeatureExtLWP>         (feats);
        feature_str<CPUIDFeatureExtMCA>         (feats);
        feature_str<CPUIDFeatureExtMCE>         (feats);
        feature_str<CPUIDFeatureExtMISALIGNSSE> (feats);
        feature_str<CPUIDFeatureExtMMX>         (feats);
        feature_str<CPUIDFeatureExtMMXEXT>      (feats);
        feature_str<CPUIDFeatureExtMP>          (feats);
        feature_str<CPUIDFeatureExtMSR>         (feats);
        feature_str<CPUIDFeatureExtMTRR>        (feats);
        feature_str<CPUIDFeatureExtNODEID_MSR>  (feats);
        feature_str<CPUIDFeatureExtNX>          (feats);
        feature_str<CPUIDFeatureExtOSVW>        (feats);
        feature_str<CPUIDFeatureExtPAE>         (feats);
        feature_str<CPUIDFeatureExtPAT>         (feats);
        feature_str<CPUIDFeatureExtPCX_L2I>     (feats);
        feature_str<CPUIDFeatureExtPDPE1GB>     (feats);
        feature_str<CPUIDFeatureExtPERFCTR_CORE>(feats);
        feature_str<CPUIDFeatureExtPERFCTR_NB>  (feats);
        feature_str<CPUIDFeatureExtPERFTSC>     (feats);
        feature_str<CPUIDFeatureExtPGE>         (feats);
        feature_str<CPUIDFeatureExtPSE36>       (feats);
        feature_str<CPUIDFeatureExtPSE>         (feats);
        feature_str<CPUIDFeatureExtRDTSCP>      (feats);
        feature_str<CPUIDFeatureExtSKINIT>      (feats);
        feature_str<CPUIDFeatureExtSSE4A>       (feats);
        feature_str<CPUIDFeatureExtSVM>         (feats);
        feature_str<CPUIDFeatureExtSYSCALL>     (feats);
        feature_str<CPUIDFeatureExtTBM>         (feats);
        feature_str<CPUIDFeatureExtTCE>         (feats);
        feature_str<CPUIDFeatureExtTOPOEXT>     (feats);
        feature_str<CPUIDFeatureExtTSC>         (feats);
        feature_str<CPUIDFeatureExtVME>         (feats);
        feature_str<CPUIDFeatureExtWDT>         (feats);
        feature_str<CPUIDFeatureExtXOP>         (feats);
        print_strvec(os, feats);
        os << std::flush;
    }

    private :

    template <unsigned EAX>
    static bool cpuid_map_init (unsigned eax_max, cpuid_map_type &cmap)
    {
        if (eax_max >= EAX)
            cmap[EAX] = query(EAX);
    }

    template <unsigned EAX>
    static cpuid_map_type::const_iterator cpuid_map_citer ()
    {
        static cpuid_map_type::const_iterator citer = cpuid_map().find(EAX);

        return citer;
    }

    template <unsigned EAX>
    static bool cpuid_map_find ()
    {
        static bool found = cpuid_map_citer<EAX>() != cpuid_map().end();

        return found;
    }

    static Array<unsigned, 4> query (unsigned eax)
    {
        unsigned ebx;
        unsigned ecx;
        unsigned edx;
        __asm__(
            "cpuid;"
            :"=a" (eax), "=b" (ebx), "=c" (ecx), "=d" (edx)
            :"a"  (eax)
            );

        Array<unsigned, 4> reg;
        reg.at<0>() = eax;
        reg.at<1>() = ebx;
        reg.at<2>() = ecx;
        reg.at<3>() = edx;

        return reg;
    }

    template <CPUIDFeature Feat>
    static void feature_str (std::vector<std::string> &feats)
    {
        feats.push_back((has_feature<Feat>() ? "*" : " ") +
                internal::CPUIDFeatureInfo<Feat>::name());
    }

    template<typename CharT, typename Traits>
    static void print_equal (std::basic_ostream<CharT, Traits> &os)
    {os << std::string(80, '=') << '\n';}

    template<typename CharT, typename Traits>
    static void print_dash (std::basic_ostream<CharT, Traits> &os)
    {os << std::string(80, '-') << '\n';}

    template <typename CharT, typename Traits>
    static void print_strvec (std::basic_ostream<CharT, Traits> &os,
            const std::vector<std::string> &strvec, std::size_t fix = 20)
    {
        const std::size_t N = strvec.size();
        std::size_t rows = N / 4;
        if (N % 4 != 0)
            ++rows;
        for (std::size_t r = 0; r != rows; ++r) {
            std::size_t index;
            if ((index = r + rows * 0) < N) print_str(os, strvec[index], fix);
            if ((index = r + rows * 1) < N) print_str(os, strvec[index], fix);
            if ((index = r + rows * 2) < N) print_str(os, strvec[index], fix);
            if ((index = r + rows * 3) < N) print_str(os, strvec[index], fix);
            os << '\n';
        }
    }

    template <typename CharT, typename Traits>
    static void print_str (std::basic_ostream<CharT, Traits> &os,
            const std::string &str, std::size_t fix = 20)
    {
        os << str;
        if (str.size() < fix) {
            for (std::size_t i = 0; i != fix - str.size(); ++i)
                os << ' ';
        }
    }
}; // class CPUID

} // namespace vsmc

#endif // VSMC_UTILITY_CPUID_HPP
