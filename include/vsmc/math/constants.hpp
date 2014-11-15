//============================================================================
// vSMC/include/vsmc/math/constants.hpp
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

#ifndef VSMC_MATH_CONSTANTS_HPP
#define VSMC_MATH_CONSTANTS_HPP

#define VSMC_DEFINE_MATH_CONSTANTS(name, val) \
template <typename T> inline T name () {return static_cast<T>(val##l);}      \
template <> inline float       name <float>       () {return val##f;}        \
template <> inline double      name <double>      () {return val;}           \
template <> inline long double name <long double> () {return val##l;}

namespace vsmc {

namespace math {

/// \brief \f$\pi\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(pi,
        3.1415926535897932384626433832795028842)

/// \brief \f$2\pi\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(pi2,
        6.2831853071795864769252867665590057684)

/// \brief \f$1/\pi\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(pi_inv,
        0.31830988618379067153776752674502872407)

/// \brief \f$\pi/2\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(pi_by2,
        1.5707963267948966192313216916397514421)

/// \brief \f$\pi/3\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(pi_by3,
        1.0471975511965977461542144610931676281)

/// \brief \f$\pi/4\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(pi_by4,
        0.78539816339744830961566084581987572105)

/// \brief \f$\pi/6\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(pi_by6,
        0.52359877559829887307710723054658381403)

/// \brief \f$(2/3)\pi\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(pi_2by3,
        2.0943951023931954923084289221863352561)

/// \brief \f$(3/4)\pi\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(pi_3by4,
        2.3561944901923449288469825374596271631)

/// \brief \f$(4/3)\pi\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(pi_4by3,
        4.1887902047863909846168578443726705123)

/// \brief \f$\sqrt{\pi}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(sqrt_pi,
        1.7724538509055160272981674833411451828)

/// \brief \f$\sqrt{2\pi}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(sqrt_2pi,
        2.5066282746310005024157652848110452530)

/// \brief \f$\sqrt{1/\pi}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(sqrt_pi_inv,
        0.56418958354775628694807945156077258584)

/// \brief \f$\sqrt{\pi/2}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(sqrt_pi_by2,
        1.2533141373155002512078826424055226265)

/// \brief \f$\sqrt{\pi/3}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(sqrt_pi_by3,
        1.0233267079464884884795516248892648607)

/// \brief \f$\sqrt{\pi/4}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(sqrt_pi_by4,
        0.88622692545275801364908374167057259140)

/// \brief \f$\sqrt{\pi/6}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(sqrt_pi_by6,
        0.88622692545275801364908374167057259140)

/// \brief \f$\sqrt{(2/3)\pi}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(sqrt_pi_2by3,
        1.4472025091165353187260292545815915357)

/// \brief \f$\sqrt{(3/4)\pi/}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(sqrt_pi_3by4,
        1.5349900619197327327193274373338972911)

/// \brief \f$\sqrt{(4/3)\pi}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(sqrt_pi_4by3,
        2.0466534158929769769591032497785297214)

/// \brief \f$\ln(\pi)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(ln_pi,
        1.1447298858494001741434273513530587116)

/// \brief \f$\ln(2\pi)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(ln_2pi,
        1.8378770664093454835606594728112352797)

/// \brief \f$\ln(\pi/2)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(ln_pi_by2,
        0.45158270528945486472619522989488214357)

/// \brief \f$\ln(\pi/3)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(ln_pi_by3,
        0.046117597181290482748182114430533007003)

/// \brief \f$\ln(\pi/4)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(ln_pi_by4,
        -0.24156447527049044469103689156329442450)

/// \brief \f$\ln(\pi/6)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(ln_pi_by6,
        -0.64702958337865482666905000702764356107)

/// \brief \f$\ln((2/3)\pi)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(ln_pi_2by3,
        0.73926477774123579216541423588870957508)

/// \brief \f$\ln((3/4)\pi)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(ln_pi_3by4,
        0.85704781339761924670420834535923128014)

/// \brief \f$\ln((4/3)\pi)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(ln_pi_4by3,
        1.4324119583011811015826463573468861432)

/// \brief \f$e\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(e,
        2.7182818284590452353602874713526624978)

/// \brief \f$1/e\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(e_inv,
        0.36787944117144232159552377016146086745)

/// \brief \f$\sqrt{e}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(sqrt_e,
        1.6487212707001281468486507878141635717)

/// \brief \f$\sqrt{1/e}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(sqrt_e_inv,
        0.60653065971263342360379953499118045344)

/// \brief \f$\sqrt{2}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(sqrt_2,
        1.4142135623730950488016887242096980786)

/// \brief \f$\sqrt{3}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(sqrt_3,
        1.7320508075688772935274463415058723669)

/// \brief \f$\sqrt{5}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(sqrt_5,
        2.2360679774997896964091736687312762354)

/// \brief \f$\sqrt{10}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(sqrt_10,
        3.1622776601683793319988935444327185337)

/// \brief \f$\sqrt{1/2}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(sqrt_1by2,
        0.70710678118654752440084436210484903928)

/// \brief \f$\sqrt{1/3}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(sqrt_1by3,
        0.57735026918962576450914878050195745565)

/// \brief \f$\sqrt{1/5}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(sqrt_1by5,
        0.44721359549995793928183473374625524709)

/// \brief \f$\sqrt{1/10}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(sqrt_1by10,
        0.31622776601683793319988935444327185337)

/// \brief \f$\ln(2)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(ln_2,
        0.69314718055994530941723212145817656807)

/// \brief \f$\ln(3)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(ln_3,
        1.0986122886681096913952452369225257046)

/// \brief \f$\ln(5)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(ln_5,
        1.6094379124341003746007593332261876395)

/// \brief \f$\ln(10)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(ln_10,
        2.3025850929940456840179914546843642076)

/// \brief \f$1/\ln(2)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(ln_inv_2,
        1.4426950408889634073599246810018921374)

/// \brief \f$1/\ln(3)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(ln_inv_3,
        0.91023922662683739361424016573610700061)

/// \brief \f$1/\ln(5)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(ln_inv_5,
        0.62133493455961181070719938818057258412)

/// \brief \f$1/\ln(10)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(ln_inv_10,
        0.43429448190325182765112891891660508229)

/// \brief \f$\ln(\ln(2))\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(ln_ln_2,
        -0.36651292058166432701243915823266946946)

} // namespace vsmc::math

} // namespace vsmc

#endif // VSMC_MATH_CONSTANTS_HPP
