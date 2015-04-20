//============================================================================
// vSMC/include/vsmc/math/constants.hpp
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

#ifndef VSMC_MATH_CONSTANTS_HPP
#define VSMC_MATH_CONSTANTS_HPP

#define VSMC_DEFINE_MATH_CONSTANTS(name, val)                                \
    template <typename T>                                                    \
    inline T name()                                                          \
    {                                                                        \
        return static_cast<T>(val##l);                                       \
    }                                                                        \
    template <>                                                              \
    inline float name<float>()                                               \
    {                                                                        \
        return val##f;                                                       \
    }                                                                        \
    template <>                                                              \
    inline double name<double>()                                             \
    {                                                                        \
        return val;                                                          \
    }                                                                        \
    template <>                                                              \
    inline long double name<long double>()                                   \
    {                                                                        \
        return val##l;                                                       \
    }

namespace vsmc
{

namespace math
{

/// \brief \f$\pi\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(pi, 3.141592653589793238462643383279502884197)

/// \brief \f$2\pi\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(pi_2, 6.283185307179586476925286766559005768394)

/// \brief \f$1/\pi\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(pi_inv, 0.3183098861837906715377675267450287240689)

/// \brief \f$\pi^2\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(pi_sqr, 9.869604401089358618834490999876151135314)

/// \brief \f$\pi/2\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(pi_by2, 1.570796326794896619231321691639751442099)

/// \brief \f$\pi/3\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(pi_by3, 1.047197551196597746154214461093167628066)

/// \brief \f$\pi/4\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(pi_by4, 0.7853981633974483096156608458198757210493)

/// \brief \f$\pi/6\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(pi_by6, 0.5235987755982988730771072305465838140329)

/// \brief \f$(2/3)\pi\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(pi_2by3, 2.094395102393195492308428922186335256131)

/// \brief \f$(3/4)\pi\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(pi_3by4, 2.356194490192344928846982537459627163148)

/// \brief \f$(4/3)\pi\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(pi_4by3, 4.188790204786390984616857844372670512263)

/// \brief \f$\sqrt{\pi}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(sqrt_pi, 1.772453850905516027298167483341145182798)

/// \brief \f$\sqrt{2\pi}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(
    sqrt_pi_2, 2.506628274631000502415765284811045253007)

/// \brief \f$\sqrt{1/\pi}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(
    sqrt_pi_inv, 0.5641895835477562869480794515607725858441)

/// \brief \f$\sqrt{\pi/2}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(
    sqrt_pi_by2, 1.253314137315500251207882642405522626503)

/// \brief \f$\sqrt{\pi/3}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(
    sqrt_pi_by3, 1.023326707946488488479551624889264860707)

/// \brief \f$\sqrt{\pi/4}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(
    sqrt_pi_by4, 0.8862269254527580136490837416705725913988)

/// \brief \f$\sqrt{\pi/6}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(
    sqrt_pi_by6, 0.7236012545582676593630146272907957678721)

/// \brief \f$\sqrt{(2/3)\pi}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(
    sqrt_pi_2by3, 1.447202509116535318726029254581591535744)

/// \brief \f$\sqrt{(3/4)\pi/}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(
    sqrt_pi_3by4, 1.534990061919732732719327437333897291061)

/// \brief \f$\sqrt{(4/3)\pi}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(
    sqrt_pi_4by3, 2.046653415892976976959103249778529721415)

/// \brief \f$\ln(\pi)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(ln_pi, 1.144729885849400174143427351353058711647)

/// \brief \f$\ln(2\pi)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(ln_pi_2, 1.837877066409345483560659472811235279723)

/// \brief \f$\ln(\pi/2)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(
    ln_pi_by2, 0.4515827052894548647261952298948821435718)

/// \brief \f$\ln(\pi/3)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(
    ln_pi_by3, 0.04611759718129048274818211443053300699980)

/// \brief \f$\ln(\pi/4)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(
    ln_pi_by4, -0.2415644752704904446910368915632944245037)

/// \brief \f$\ln(\pi/6)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(
    ln_pi_by6, -0.6470295833786548266690500070276435610757)

/// \brief \f$\ln((2/3)\pi)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(
    ln_pi_2by3, 0.7392647777412357921654142358887095750753)

/// \brief \f$\ln((3/4)\pi)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(
    ln_pi_3by4, 0.8570478133976192467042083453592312801438)

/// \brief \f$\ln((4/3)\pi)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(
    ln_pi_4by3, 1.432411958301181101582646357346886143151)

/// \brief \f$e\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(e, 2.718281828459045235360287471352662497757)

/// \brief \f$1/e\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(e_inv, 0.3678794411714423215955237701614608674458)

/// \brief \f$\sqrt{e}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(sqrt_e, 1.648721270700128146848650787814163571654)

/// \brief \f$\sqrt{1/e}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(
    sqrt_e_inv, 0.6065306597126334236037995349911804534419)

/// \brief \f$\sqrt{2}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(sqrt_2, 1.414213562373095048801688724209698078570)

/// \brief \f$\sqrt{3}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(sqrt_3, 1.732050807568877293527446341505872366943)

/// \brief \f$\sqrt{5}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(sqrt_5, 2.236067977499789696409173668731276235441)

/// \brief \f$\sqrt{10}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(sqrt_10, 3.162277660168379331998893544432718533720)

/// \brief \f$\sqrt{1/2}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(
    sqrt_1by2, 0.7071067811865475244008443621048490392848)

/// \brief \f$\sqrt{1/3}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(
    sqrt_1by3, 0.5773502691896257645091487805019574556476)

/// \brief \f$\sqrt{1/5}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(
    sqrt_1by5, 0.4472135954999579392818347337462552470881)

/// \brief \f$\sqrt{1/10}\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(
    sqrt_1by10, 0.3162277660168379331998893544432718533720)

/// \brief \f$\ln(2)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(ln_2, 0.6931471805599453094172321214581765680755)

/// \brief \f$\ln(3)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(ln_3, 1.098612288668109691395245236922525704647)

/// \brief \f$\ln(5)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(ln_5, 1.609437912434100374600759333226187639526)

/// \brief \f$\ln(10)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(ln_10, 2.302585092994045684017991454684364207601)

/// \brief \f$1/\ln(2)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(
    ln_inv_2, 1.442695040888963407359924681001892137427)

/// \brief \f$1/\ln(3)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(
    ln_inv_3, 0.9102392266268373936142401657361070006126)

/// \brief \f$1/\ln(5)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(
    ln_inv_5, 0.6213349345596118107071993881805725841234)

/// \brief \f$1/\ln(10)\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(
    ln_inv_10, 0.4342944819032518276511289189166050822944)

/// \brief \f$\ln(\ln(2))\f$
/// \ingroup Constants
VSMC_DEFINE_MATH_CONSTANTS(
    ln_ln_2, -0.3665129205816643270124391582326694694543)

} // namespace vsmc::math

} // namespace vsmc

#endif // VSMC_MATH_CONSTANTS_HPP
