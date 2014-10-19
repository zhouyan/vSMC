//============================================================================
// include/vsmc/utility/cpuid.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_UTILITY_CPUID_HPP
#define VSMC_UTILITY_CPUID_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/utility/array.hpp>
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#ifdef _MSC_VER
#include <intrin.h>
#endif

#define VSMC_DEFINE_CPUID_FEATURE_INFO(feat, eax_val, reg_val, bit) \
template<> struct CPUIDFeatureInfo< CPUIDFeature##feat >                     \
{                                                                            \
    static std::string name () {return std::string(#feat);}                  \
    static VSMC_CONSTEXPR const unsigned eax = eax_val##U;                   \
    static VSMC_CONSTEXPR const unsigned mask = 1U << bit;                   \
    static VSMC_CONSTEXPR const std::size_t reg = reg_val;                   \
};

#define VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(feat, eax_val, reg_val, bit) \
template<> struct CPUIDFeatureInfo< CPUIDFeatureExt##feat >                  \
{                                                                            \
    static std::string name () {return std::string(#feat);}                  \
    static VSMC_CONSTEXPR const unsigned eax = eax_val##U;                   \
    static VSMC_CONSTEXPR const unsigned mask = 1U << bit;                   \
    static VSMC_CONSTEXPR const std::size_t reg = reg_val;                   \
};

namespace vsmc {
/// \brief Basic CPU features
/// \ingroup CPUID
///
/// \details
/// This table is named according to Intel specification. AMD CPUID
/// specification may be slightly different.
enum CPUIDFeature
{
    CPUIDFeatureSSE3,         ///< EAX = 0x01; ECX[00]
    CPUIDFeaturePCLMULQDQ,    ///< EAX = 0x01; ECX[01]
    CPUIDFeatureDTES64,       ///< EAX = 0x01; ECX[02]
    CPUIDFeatureMONITOR,      ///< EAX = 0x01; ECX[03]
    CPUIDFeatureDS_CPL,       ///< EAX = 0x01; ECX[04]
    CPUIDFeatureVMX,          ///< EAX = 0x01; ECX[05]
    CPUIDFeatureSMX,          ///< EAX = 0x01; ECX[06]
    CPUIDFeatureEST,          ///< EAX = 0x01; ECX[07]
    CPUIDFeatureTM2,          ///< EAX = 0x01; ECX[08]
    CPUIDFeatureSSSE3,        ///< EAX = 0x01; ECX[09]
    CPUIDFeatureCNXT_ID,      ///< EAX = 0x01; ECX[10]
    CPUIDFeatureFMA,          ///< EAX = 0x01; ECX[12]
    CPUIDFeatureCX16,         ///< EAX = 0x01; ECX[13]
    CPUIDFeatureXTPR,         ///< EAX = 0x01; ECX[14]
    CPUIDFeaturePDCM,         ///< EAX = 0x01; ECX[15]
    CPUIDFeaturePCID,         ///< EAX = 0x01; ECX[17]
    CPUIDFeatureDCA,          ///< EAX = 0x01; ECX[18]
    CPUIDFeatureSSE4_1,       ///< EAX = 0x01; ECX[19]
    CPUIDFeatureSSE4_2,       ///< EAX = 0x01; ECX[20]
    CPUIDFeatureX2APIC,       ///< EAX = 0x01; ECX[21]
    CPUIDFeatureMOVBE,        ///< EAX = 0x01; ECX[22]
    CPUIDFeaturePOPCNT,       ///< EAX = 0x01; ECX[23]
    CPUIDFeatureTSC_DEADLINE, ///< EAX = 0x01; ECX[24]
    CPUIDFeatureAES,          ///< EAX = 0x01; ECX[25]
    CPUIDFeatureXSAVE,        ///< EAX = 0x01; ECX[26]
    CPUIDFeatureOSXSAVE,      ///< EAX = 0x01; ECX[27]
    CPUIDFeatureAVX,          ///< EAX = 0x01; ECX[28]
    CPUIDFeatureF16C,         ///< EAX = 0x01; ECX[29]
    CPUIDFeatureRDRND,        ///< EAX = 0x01; ECX[30]
    CPUIDFeatureHYPERVISOR,   ///< EAX = 0x01; ECX[31]

    CPUIDFeatureFPU,          ///< EAX = 0x01; EDX[00]
    CPUIDFeatureVME,          ///< EAX = 0x01; EDX[01]
    CPUIDFeatureDE,           ///< EAX = 0x01; EDX[02]
    CPUIDFeaturePSE,          ///< EAX = 0x01; EDX[03]
    CPUIDFeatureTSC,          ///< EAX = 0x01; EDX[04]
    CPUIDFeatureMSR,          ///< EAX = 0x01; EDX[05]
    CPUIDFeaturePAE,          ///< EAX = 0x01; EDX[06]
    CPUIDFeatureMCE,          ///< EAX = 0x01; EDX[07]
    CPUIDFeatureCX8,          ///< EAX = 0x01; EDX[08]
    CPUIDFeatureAPIC,         ///< EAX = 0x01; EDX[09]
    CPUIDFeatureSEP,          ///< EAX = 0x01; EDX[11]
    CPUIDFeatureMTRR,         ///< EAX = 0x01; EDX[12]
    CPUIDFeaturePGE,          ///< EAX = 0x01; EDX[13]
    CPUIDFeatureMCA,          ///< EAX = 0x01; EDX[14]
    CPUIDFeatureCMOV,         ///< EAX = 0x01; EDX[15]
    CPUIDFeaturePAT,          ///< EAX = 0x01; EDX[16]
    CPUIDFeaturePSE_36,       ///< EAX = 0x01; EDX[17]
    CPUIDFeaturePSN,          ///< EAX = 0x01; EDX[18]
    CPUIDFeatureCLFSH,        ///< EAX = 0x01; EDX[19]
    CPUIDFeatureDS,           ///< EAX = 0x01; EDX[21]
    CPUIDFeatureACPI,         ///< EAX = 0x01; EDX[22]
    CPUIDFeatureMMX,          ///< EAX = 0x01; EDX[23]
    CPUIDFeatureFXSR,         ///< EAX = 0x01; EDX[24]
    CPUIDFeatureSSE,          ///< EAX = 0x01; EDX[25]
    CPUIDFeatureSSE2,         ///< EAX = 0x01; EDX[26]
    CPUIDFeatureSS,           ///< EAX = 0x01; EDX[27]
    CPUIDFeatureHTT,          ///< EAX = 0x01; EDX[28]
    CPUIDFeatureTM,           ///< EAX = 0x01; EDX[29]
    CPUIDFeatureIA64,         ///< EAX = 0x01; EDX[30]
    CPUIDFeaturePBE,          ///< EAX = 0x01; EDX[31]

    CPUIDFeatureExtFSGSBASE,     ///< EAX = 0x07; EBX[00]
    CPUIDFeatureExtBMI1,         ///< EAX = 0x07; EBX[03]
    CPUIDFeatureExtHLE,          ///< EAX = 0x07; EBX[04]
    CPUIDFeatureExtAVX2,         ///< EAX = 0x07; EBX[05]
    CPUIDFeatureExtSMEP,         ///< EAX = 0x07; EBX[07]
    CPUIDFeatureExtBMI2,         ///< EAX = 0x07; EBX[08]
    CPUIDFeatureExtERMS,         ///< EAX = 0x07; EBX[09]
    CPUIDFeatureExtINVPCID,      ///< EAX = 0x07; EBX[10]
    CPUIDFeatureExtRTM,          ///< EAX = 0x07; EBX[11]
    CPUIDFeatureExtMPX,          ///< EAX = 0x07; EBX[14]
    CPUIDFeatureExtAVX512F,      ///< EAX = 0x07; EBX[16]
    CPUIDFeatureExtAVX512DQ,     ///< EAX = 0x07; EBX[17]
    CPUIDFeatureExtRDSEED,       ///< EAX = 0x07; EBX[18]
    CPUIDFeatureExtADX,          ///< EAX = 0x07; EBX[19]
    CPUIDFeatureExtSMAP,         ///< EAX = 0x07; EBX[20]
    CPUIDFeatureExtAVX512IFMA52, ///< EAX = 0x07; EBX[21]
    CPUIDFeatureExtCLFLUSHOPT,   ///< EAX = 0x07; EBX[23]
    CPUIDFeatureExtINTEL_TRACE,  ///< EAX = 0x07; EBX[25]
    CPUIDFeatureExtAVX512PF,     ///< EAX = 0x07; EBX[26]
    CPUIDFeatureExtAVX512ER,     ///< EAX = 0x07; EBX[27]
    CPUIDFeatureExtAVX512CD,     ///< EAX = 0x07; EBX[28]
    CPUIDFeatureExtSHA,          ///< EAX = 0x07; EBX[29]
    CPUIDFeatureExtAVX512BW,     ///< EAX = 0x07; EBX[30]
    CPUIDFeatureExtAVX512VL,     ///< EAX = 0x07; EBX[31]

    CPUIDFeatureExtPREFETCHWT1,  ///< EAX = 0x07; ECX[00]
    CPUIDFeatureExtAVX512VBMI,   ///< EAX = 0x07; ECX[01]
}; // enum CPUIDFeature

namespace internal {

template <CPUIDFeature> struct CPUIDFeatureInfo;

VSMC_DEFINE_CPUID_FEATURE_INFO(SSE3,         0x01, 2,  0)
VSMC_DEFINE_CPUID_FEATURE_INFO(PCLMULQDQ,    0x01, 2,  1)
VSMC_DEFINE_CPUID_FEATURE_INFO(DTES64,       0x01, 2,  2)
VSMC_DEFINE_CPUID_FEATURE_INFO(MONITOR,      0x01, 2,  3)
VSMC_DEFINE_CPUID_FEATURE_INFO(DS_CPL,       0x01, 2,  4)
VSMC_DEFINE_CPUID_FEATURE_INFO(VMX,          0x01, 2,  5)
VSMC_DEFINE_CPUID_FEATURE_INFO(SMX,          0x01, 2,  6)
VSMC_DEFINE_CPUID_FEATURE_INFO(EST,          0x01, 2,  7)
VSMC_DEFINE_CPUID_FEATURE_INFO(TM2,          0x01, 2,  8)
VSMC_DEFINE_CPUID_FEATURE_INFO(SSSE3,        0x01, 2,  9)
VSMC_DEFINE_CPUID_FEATURE_INFO(CNXT_ID,      0x01, 2, 10)
VSMC_DEFINE_CPUID_FEATURE_INFO(FMA,          0x01, 2, 12)
VSMC_DEFINE_CPUID_FEATURE_INFO(CX16,         0x01, 2, 13)
VSMC_DEFINE_CPUID_FEATURE_INFO(XTPR,         0x01, 2, 14)
VSMC_DEFINE_CPUID_FEATURE_INFO(PDCM,         0x01, 2, 15)
VSMC_DEFINE_CPUID_FEATURE_INFO(PCID,         0x01, 2, 17)
VSMC_DEFINE_CPUID_FEATURE_INFO(DCA,          0x01, 2, 18)
VSMC_DEFINE_CPUID_FEATURE_INFO(SSE4_1,       0x01, 2, 19)
VSMC_DEFINE_CPUID_FEATURE_INFO(SSE4_2,       0x01, 2, 20)
VSMC_DEFINE_CPUID_FEATURE_INFO(X2APIC,       0x01, 2, 21)
VSMC_DEFINE_CPUID_FEATURE_INFO(MOVBE,        0x01, 2, 22)
VSMC_DEFINE_CPUID_FEATURE_INFO(POPCNT,       0x01, 2, 23)
VSMC_DEFINE_CPUID_FEATURE_INFO(TSC_DEADLINE, 0x01, 2, 24)
VSMC_DEFINE_CPUID_FEATURE_INFO(AES,          0x01, 2, 25)
VSMC_DEFINE_CPUID_FEATURE_INFO(XSAVE,        0x01, 2, 26)
VSMC_DEFINE_CPUID_FEATURE_INFO(OSXSAVE,      0x01, 2, 27)
VSMC_DEFINE_CPUID_FEATURE_INFO(AVX,          0x01, 2, 28)
VSMC_DEFINE_CPUID_FEATURE_INFO(F16C,         0x01, 2, 29)
VSMC_DEFINE_CPUID_FEATURE_INFO(RDRND,        0x01, 2, 30)
VSMC_DEFINE_CPUID_FEATURE_INFO(HYPERVISOR,   0x01, 2, 31)

VSMC_DEFINE_CPUID_FEATURE_INFO(FPU,          0x01, 3,  0)
VSMC_DEFINE_CPUID_FEATURE_INFO(VME,          0x01, 3,  1)
VSMC_DEFINE_CPUID_FEATURE_INFO(DE,           0x01, 3,  2)
VSMC_DEFINE_CPUID_FEATURE_INFO(PSE,          0x01, 3,  3)
VSMC_DEFINE_CPUID_FEATURE_INFO(TSC,          0x01, 3,  4)
VSMC_DEFINE_CPUID_FEATURE_INFO(MSR,          0x01, 3,  5)
VSMC_DEFINE_CPUID_FEATURE_INFO(PAE,          0x01, 3,  6)
VSMC_DEFINE_CPUID_FEATURE_INFO(MCE,          0x01, 3,  7)
VSMC_DEFINE_CPUID_FEATURE_INFO(CX8,          0x01, 3,  8)
VSMC_DEFINE_CPUID_FEATURE_INFO(APIC,         0x01, 3,  9)
VSMC_DEFINE_CPUID_FEATURE_INFO(SEP,          0x01, 3, 11)
VSMC_DEFINE_CPUID_FEATURE_INFO(MTRR,         0x01, 3, 12)
VSMC_DEFINE_CPUID_FEATURE_INFO(PGE,          0x01, 3, 13)
VSMC_DEFINE_CPUID_FEATURE_INFO(MCA,          0x01, 3, 14)
VSMC_DEFINE_CPUID_FEATURE_INFO(CMOV,         0x01, 3, 15)
VSMC_DEFINE_CPUID_FEATURE_INFO(PAT,          0x01, 3, 16)
VSMC_DEFINE_CPUID_FEATURE_INFO(PSE_36,       0x01, 3, 17)
VSMC_DEFINE_CPUID_FEATURE_INFO(PSN,          0x01, 3, 18)
VSMC_DEFINE_CPUID_FEATURE_INFO(CLFSH,        0x01, 3, 19)
VSMC_DEFINE_CPUID_FEATURE_INFO(DS,           0x01, 3, 21)
VSMC_DEFINE_CPUID_FEATURE_INFO(ACPI,         0x01, 3, 22)
VSMC_DEFINE_CPUID_FEATURE_INFO(MMX,          0x01, 3, 23)
VSMC_DEFINE_CPUID_FEATURE_INFO(FXSR,         0x01, 3, 24)
VSMC_DEFINE_CPUID_FEATURE_INFO(SSE,          0x01, 3, 25)
VSMC_DEFINE_CPUID_FEATURE_INFO(SSE2,         0x01, 3, 26)
VSMC_DEFINE_CPUID_FEATURE_INFO(SS,           0x01, 3, 27)
VSMC_DEFINE_CPUID_FEATURE_INFO(HTT,          0x01, 3, 28)
VSMC_DEFINE_CPUID_FEATURE_INFO(TM,           0x01, 3, 29)
VSMC_DEFINE_CPUID_FEATURE_INFO(IA64,         0x01, 3, 30)
VSMC_DEFINE_CPUID_FEATURE_INFO(PBE,          0x01, 3, 31)

VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(FSGSBASE,     0x07, 1,  0)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(BMI1,         0x07, 1,  3)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(HLE,          0x07, 1,  4)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(AVX2,         0x07, 1,  5)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(SMEP,         0x07, 1,  7)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(BMI2,         0x07, 1,  8)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(ERMS,         0x07, 1,  9)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(INVPCID,      0x07, 1, 10)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(RTM,          0x07, 1, 11)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(MPX,          0x07, 1, 14)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(AVX512F,      0x07, 1, 16)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(AVX512DQ,     0x07, 1, 17)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(RDSEED,       0x07, 1, 18)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(ADX,          0x07, 1, 19)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(SMAP,         0x07, 1, 20)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(AVX512IFMA52, 0x07, 1, 21)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(CLFLUSHOPT,   0x07, 1, 23)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(INTEL_TRACE,  0x07, 1, 25)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(AVX512PF,     0x07, 1, 26)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(AVX512ER,     0x07, 1, 27)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(AVX512CD,     0x07, 1, 28)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(SHA,          0x07, 1, 29)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(AVX512BW,     0x07, 1, 30)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(AVX512VL,     0x07, 1, 31)

VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(PREFETCHWT1,  0x07, 3,  0)
VSMC_DEFINE_CPUID_FEATURE_EXT_INFO(AVX512VBMI,   0x07, 3,  1)

} // namespace vsmc::internal

/// \brief Query CPUID informations
/// \ingroup CPUID
class CPUID
{
    public :

    /// \brief The array type that holds EAX, EBX, ECX, and EDX, in that order
    typedef Array<unsigned, 4> reg_type;

    /// \brief Extract bits from an unsigned integer
    ///
    /// \tparam Hi The highest bit (0-31)
    /// \tparam Lo The lowest bit (0-31)
    template <unsigned Hi, unsigned Lo>
    static unsigned extract_bits (unsigned reg)
    {return (reg << (31U - Hi)) >> (31U - Hi + Lo);}

    /// \brief Max calling parameter EAX
    static unsigned max_eax ()
    {return cpuid<0, 0>().at<0>();}

    /// \brief Max extended calling parameter EAX
    static unsigned max_eax_ext ()
    {return cpuid<0x80000000U, 0>().at<0>();}

    /// \brief Get CPU features using CPUID
    template <typename CharT, typename Traits>
    static void info (std::basic_ostream<CharT, Traits> &os)
    {
        print_equal(os);
        os << "Vendor ID:                       " << vendor()      << '\n';
        os << "Cache line size (bytes):         " << cl_size ()    << '\n';
        os << "Maximum processor ID:            " << max_proc_id() << '\n';
        if (max_eax_ext() >= 0x80000004U)
            os << "Processor brand:                 " << brand() << '\n';
        if (max_eax() >= 0x16) {
            os << "Base frequency (MHz):            " << base_freq() << '\n';
            os << "Maximum frequency (MHz):         " << max_freq()  << '\n';
            os << "Bus (reference) frequency (MHz): " << bus_freq()  << '\n';
        }
        print_equal(os);
        features(os);
        print_equal(os);
    }

    /// \brief Vendor ID (EAX = 0x00; EBX, EDX, ECX)
    static std::string vendor ()
    {
        reg_type reg(cpuid<0x00, 0x00>());
        const unsigned *uptr = reg.data();
        char str[sizeof(unsigned) * 3 + 1] = {'\0'};
        std::memcpy(str + sizeof(unsigned) * 0, uptr + 1, sizeof(unsigned));
        std::memcpy(str + sizeof(unsigned) * 1, uptr + 3, sizeof(unsigned));
        std::memcpy(str + sizeof(unsigned) * 2, uptr + 2, sizeof(unsigned));

        return std::string(str);
    }

    /// \brief Processor brand string (EAX = 0x80000002,0x80000003,0x80000004; EAX,
    /// EBX, ECX)
    static std::string brand ()
    {
        reg_type reg2(cpuid<0x80000002U, 0x00>());
        reg_type reg3(cpuid<0x80000003U, 0x00>());
        reg_type reg4(cpuid<0x80000004U, 0x00>());
        const std::size_t reg_size = sizeof(unsigned) * 4;
        char str[reg_size * 3] = {'\0'};
        std::memcpy(str + reg_size * 0, reg2.data(), reg_size);
        std::memcpy(str + reg_size * 1, reg3.data(), reg_size);
        std::memcpy(str + reg_size * 2, reg4.data(), reg_size);

        return std::string(str);
    }

    /// \brief CLFLUSH line size in bytes (EAX = 0x01; EBX[15:8])
    static unsigned cl_size ()
    {return extract_bits<15, 8>(cpuid<0x01, 0x00>().at<1>()) * 8;}

    /// \brief Maxumum number of addressable IDs for logical processor in this
    /// physical pacakge (EAX = 0x01; EBX[23:16])
    static unsigned max_proc_id ()
    {return extract_bits<23, 16>(cpuid<0x01, 0x00>().at<1>());}

    /// \brief Base frequency in MHz (EAX = 0x16; EAX[15:0])
    static unsigned base_freq ()
    {return extract_bits<15, 0>(cpuid<0x16, 0x00>().at<0>());}

    /// \brief Maximum frequency in MHz (EAX = 0x16; EBX[15:0])
    static unsigned max_freq ()
    {return extract_bits<15, 0>(cpuid<0x16, 0x00>().at<1>());}

    /// \brief Bus (reference) frequency in MHz (EAX = 0x16; ECX[15:0])
    static unsigned bus_freq ()
    {return extract_bits<15, 0>(cpuid<0x16, 0x00>().at<2>());}

    /// \brief CPU feature
    template <CPUIDFeature Feat>
    static bool has_feature ()
    {
        return (cpuid<internal::CPUIDFeatureInfo<Feat>::eax, 0x00>().template
                at<internal::CPUIDFeatureInfo<Feat>::reg>() &
                internal::CPUIDFeatureInfo<Feat>::mask) != 0;
    }

    /// \brief Query all basic features
    template <typename CharT, typename Traits>
    static void features (std::basic_ostream<CharT, Traits> &os)
    {
        std::vector<std::string> feats;
        feature_str(feats);
        os << "Basic features\n";
        print_dash(os);
        print_features(os, feats, 20);
        if (max_eax() >= 0x07) {
            print_dash(os);
            std::vector<std::string> feats_ext;
            feature_str_ext(feats_ext);
            os << "Extended features\n";
            print_dash(os);
            print_features(os, feats_ext, 20);
        }
        os << std::flush;
    }

    private :

#ifdef _MSC_VER
    static reg_type cpuid (unsigned eax, unsigned ecx)
    {
        int CPUInfo[4] = {0};
        int InfoType[2] = {0};
        std::memcpy(&InfoType[0], &eax, sizeof(int));
        std::memcpy(&InfoType[1], &ecx, sizeof(int));
        __cpuidex(CPUInfo, InfoType[0], InfoType[1]);
        reg_type reg;
        std::memcpy(reg.data(), CPUInfo, sizeof(int) * 4);

        return reg;
    }
#elif VSMC_HAS_INLINE_ASSEMBLY
    static reg_type cpuid (unsigned eax, unsigned ecx)
    {
        unsigned ebx = 0;
        unsigned edx = 0;
        __asm__(
                "cpuid\n"
                : "=a" (eax), "=b" (ebx), "=c" (ecx), "=d" (edx)
                :  "a" (eax),  "c" (ecx)
               );
        reg_type reg;
        reg.at<0>() = eax;
        reg.at<1>() = ebx;
        reg.at<2>() = ecx;
        reg.at<3>() = edx;

        return reg;
    }
#else // VSMC_HAS_INLINE_ASSEMBLY
#error Compiler not supported
#endif // _MSC_VER

    template <unsigned EAX, unsigned ECX>
    static const reg_type &cpuid ()
    {
        static reg_type reg;
        static bool initialized = false;

        if (initialized)
            return reg;

        if (EAX < 0x80000000U) {
            reg = cpuid(0x00, 0x00);
            if (EAX > reg.at<0>())
                reg.fill(0);
            else
                reg = cpuid(EAX, ECX);
        } else {
            reg = cpuid(0x80000000U, 0x00);
            if (EAX > reg.at<0>())
                reg.fill(0);
            else
                reg = cpuid(EAX, ECX);
        }
        initialized = true;

        return reg;
    }

    static void feature_str (std::vector<std::string> &feats)
    {
        feature_str<CPUIDFeatureSSE3>        (feats);
        feature_str<CPUIDFeaturePCLMULQDQ>   (feats);
        feature_str<CPUIDFeatureDTES64>      (feats);
        feature_str<CPUIDFeatureMONITOR>     (feats);
        feature_str<CPUIDFeatureDS_CPL>      (feats);
        feature_str<CPUIDFeatureVMX>         (feats);
        feature_str<CPUIDFeatureSMX>         (feats);
        feature_str<CPUIDFeatureEST>         (feats);
        feature_str<CPUIDFeatureTM2>         (feats);
        feature_str<CPUIDFeatureSSSE3>       (feats);
        feature_str<CPUIDFeatureCNXT_ID>     (feats);
        feature_str<CPUIDFeatureFMA>         (feats);
        feature_str<CPUIDFeatureCX16>        (feats);
        feature_str<CPUIDFeatureXTPR>        (feats);
        feature_str<CPUIDFeaturePDCM>        (feats);
        feature_str<CPUIDFeaturePCID>        (feats);
        feature_str<CPUIDFeatureDCA>         (feats);
        feature_str<CPUIDFeatureSSE4_1>      (feats);
        feature_str<CPUIDFeatureSSE4_2>      (feats);
        feature_str<CPUIDFeatureX2APIC>      (feats);
        feature_str<CPUIDFeatureMOVBE>       (feats);
        feature_str<CPUIDFeaturePOPCNT>      (feats);
        feature_str<CPUIDFeatureTSC_DEADLINE>(feats);
        feature_str<CPUIDFeatureAES>         (feats);
        feature_str<CPUIDFeatureXSAVE>       (feats);
        feature_str<CPUIDFeatureOSXSAVE>     (feats);
        feature_str<CPUIDFeatureAVX>         (feats);
        feature_str<CPUIDFeatureF16C>        (feats);
        feature_str<CPUIDFeatureRDRND>       (feats);
        feature_str<CPUIDFeatureHYPERVISOR>  (feats);

        feature_str<CPUIDFeatureFPU>         (feats);
        feature_str<CPUIDFeatureVME>         (feats);
        feature_str<CPUIDFeatureDE>          (feats);
        feature_str<CPUIDFeaturePSE>         (feats);
        feature_str<CPUIDFeatureTSC>         (feats);
        feature_str<CPUIDFeatureMSR>         (feats);
        feature_str<CPUIDFeaturePAE>         (feats);
        feature_str<CPUIDFeatureMCE>         (feats);
        feature_str<CPUIDFeatureCX8>         (feats);
        feature_str<CPUIDFeatureAPIC>        (feats);
        feature_str<CPUIDFeatureSEP>         (feats);
        feature_str<CPUIDFeatureMTRR>        (feats);
        feature_str<CPUIDFeaturePGE>         (feats);
        feature_str<CPUIDFeatureMCA>         (feats);
        feature_str<CPUIDFeatureCMOV>        (feats);
        feature_str<CPUIDFeaturePAT>         (feats);
        feature_str<CPUIDFeaturePSE_36>      (feats);
        feature_str<CPUIDFeaturePSN>         (feats);
        feature_str<CPUIDFeatureCLFSH>       (feats);
        feature_str<CPUIDFeatureDS>          (feats);
        feature_str<CPUIDFeatureACPI>        (feats);
        feature_str<CPUIDFeatureMMX>         (feats);
        feature_str<CPUIDFeatureFXSR>        (feats);
        feature_str<CPUIDFeatureSSE>         (feats);
        feature_str<CPUIDFeatureSSE2>        (feats);
        feature_str<CPUIDFeatureSS>          (feats);
        feature_str<CPUIDFeatureHTT>         (feats);
        feature_str<CPUIDFeatureTM>          (feats);
        feature_str<CPUIDFeatureIA64>        (feats);
        feature_str<CPUIDFeaturePBE>         (feats);
    }

    static void feature_str_ext (std::vector<std::string> &feats_ext)
    {
        feature_str<CPUIDFeatureExtFSGSBASE>    (feats_ext);
        feature_str<CPUIDFeatureExtBMI1>        (feats_ext);
        feature_str<CPUIDFeatureExtHLE>         (feats_ext);
        feature_str<CPUIDFeatureExtAVX2>        (feats_ext);
        feature_str<CPUIDFeatureExtSMEP>        (feats_ext);
        feature_str<CPUIDFeatureExtBMI2>        (feats_ext);
        feature_str<CPUIDFeatureExtERMS>        (feats_ext);
        feature_str<CPUIDFeatureExtINVPCID>     (feats_ext);
        feature_str<CPUIDFeatureExtRTM>         (feats_ext);
        feature_str<CPUIDFeatureExtMPX>         (feats_ext);
        feature_str<CPUIDFeatureExtAVX512F>     (feats_ext);
        feature_str<CPUIDFeatureExtAVX512DQ>    (feats_ext);
        feature_str<CPUIDFeatureExtRDSEED>      (feats_ext);
        feature_str<CPUIDFeatureExtADX>         (feats_ext);
        feature_str<CPUIDFeatureExtSMAP>        (feats_ext);
        feature_str<CPUIDFeatureExtAVX512IFMA52>(feats_ext);
        feature_str<CPUIDFeatureExtCLFLUSHOPT>  (feats_ext);
        feature_str<CPUIDFeatureExtINTEL_TRACE> (feats_ext);
        feature_str<CPUIDFeatureExtAVX512PF>    (feats_ext);
        feature_str<CPUIDFeatureExtAVX512ER>    (feats_ext);
        feature_str<CPUIDFeatureExtAVX512CD>    (feats_ext);
        feature_str<CPUIDFeatureExtSHA>         (feats_ext);
        feature_str<CPUIDFeatureExtAVX512BW>    (feats_ext);
        feature_str<CPUIDFeatureExtAVX512VL>    (feats_ext);

        feature_str<CPUIDFeatureExtPREFETCHWT1> (feats_ext);
        feature_str<CPUIDFeatureExtAVX512VBMI>  (feats_ext);
    }

    template <CPUIDFeature Feat>
    static void feature_str (std::vector<std::string> &feats)
    {
        feats.push_back(" " + internal::CPUIDFeatureInfo<Feat>::name() +
                (has_feature<Feat>() ? "*" : " "));
    }

    template<typename CharT, typename Traits>
    static void print_equal (std::basic_ostream<CharT, Traits> &os)
    {os << std::string(80, '=') << '\n';}

    template<typename CharT, typename Traits>
    static void print_dash (std::basic_ostream<CharT, Traits> &os)
    {os << std::string(80, '-') << '\n';}

    template <typename CharT, typename Traits>
    static void print_features (std::basic_ostream<CharT, Traits> &os,
            std::vector<std::string> &feats, std::size_t fix)
    {
        std::sort(feats.begin(), feats.end());
        const std::size_t N = feats.size();
        std::size_t rows = N / 4;
        if (N % 4 != 0)
            ++rows;
        for (std::size_t r = 0; r != rows; ++r) {
            std::size_t index;
            if ((index = r + rows * 0) < N) print_str(os, feats[index], fix);
            if ((index = r + rows * 1) < N) print_str(os, feats[index], fix);
            if ((index = r + rows * 2) < N) print_str(os, feats[index], fix);
            if ((index = r + rows * 3) < N) print_str(os, feats[index], fix);
            os << '\n';
        }
    }

    template <typename CharT, typename Traits>
    static void print_str (std::basic_ostream<CharT, Traits> &os,
            std::string &str, std::size_t fix)
    {
        if (str[str.size() - 1] == '*') {
            str[str.size() - 1] = ' ';
            str[0] = '*';
        }
        os << str;
        if (str.size() < fix)
            os << std::string(fix - str.size(), ' ');
    }
}; // class CPUID

/// \brief Query CPU features using CPUID
/// \ingroup CPUID
template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &operator<< (
        std::basic_ostream<CharT, Traits> &os, const CPUID &)
{CPUID::info(os); return os;}

} // namespace vsmc

#endif // VSMC_UTILITY_CPUID_HPP
