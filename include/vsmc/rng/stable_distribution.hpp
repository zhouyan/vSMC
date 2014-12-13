//============================================================================
// vSMC/include/vsmc/rng/stable_distribution.hpp
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

#ifndef VSMC_RNG_STABLE_DISTRIBUTION_HPP
#define VSMC_RNG_STABLE_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>

#define VSMC_RUNTIME_ASSERT_RNG_STABLE_DISTRIBUTION_PARAM_CHECK_STABILITY(a) \
    VSMC_RUNTIME_ASSERT((a > 0 && a <= 2),                                   \
            ("**StableDistribution** CONSTRUCTED WITH INVALID "              \
             "STABILITY PARAMETER VALUE"))

#define VSMC_RUNTIME_ASSERT_RNG_STABLE_DISTRIBUTION_PARAM_CHECK_SKEWNESS(a) \
    VSMC_RUNTIME_ASSERT((a >= -1 && a <= 1),                                 \
            ("**StableDistribution** CONSTRUCTED WITH INVALID "              \
             "SKEWNESS PARAMETER VALUE"))

#define VSMC_RUNTIME_ASSERT_RNG_STABLE_DISTRIBUTION_PARAM_CHECK_SCALE(a) \
    VSMC_RUNTIME_ASSERT((a > 0),                                             \
            ("**StableDistribution** CONSTRUCTED WITH INVALID "              \
             "SCALE PARAMETER VALUE"))

namespace vsmc {

/// \brief Stable distribution
/// \ingroup Distribution
template <typename FPType>
class StableDistribution
{
    private :

    public :

    typedef FPType result_type;

    struct param_type
    {
        typedef FPType result_type;

        typedef StableDistribution<FPType> distribution_type;

        explicit param_type (
                result_type stability = 1, result_type skewness = 0,
                result_type location = 0, result_type scale = 1) :
            stability_(stability), skewness_(skewness),
            location_(location), scale_(scale) {}

        result_type stability () const {return stability_;}
        result_type skewness  () const {return skewness_;}
        result_type location  () const {return location_;}
        result_type scale     () const {return scale_;}

        friend inline bool operator== (
                const param_type &param1, const param_type &param2)
        {
            if (param1.stability_ < param2.stability_ ||
                    param1.stability_ > param2.stability_)
                return false;
            if (param1.skewness_ < param2.skewness_ ||
                    param1.skewness_ > param2.skewness_)
                return false;
            if (param1.location_ < param2.location_ ||
                    param1.location_ > param2.location_)
                return false;
            if (param1.scale_ < param2.scale_ ||
                    param1.scale_ > param2.scale_)
                return false;
            return true;
        }

        friend inline bool operator!= (
                const param_type param1, const param_type param2)
        {return !(param1 == param2);}

        template <typename CharT, typename Traits>
        friend inline std::basic_ostream<CharT, Traits> &operator<< (
                std::basic_ostream<CharT, Traits> &os, const param_type &param)
        {
            if (!os.good())
                return os;

            os << param.stability_ << ' ' << param.skewness_ << ' ';
            os << param.location_ << ' ' << param.scale_ << ' ';

            return os;
        }

        template <typename CharT, typename Traits>
        friend inline std::basic_istream<CharT, Traits> &operator>> (
                std::basic_istream<CharT, Traits> &is, param_type &param)
        {
            if (!is.good())
                return is;

            result_type stability = 0;
            result_type skewness = 0;
            result_type location = 0;
            result_type scale = 0;
            is >> std::ws >> stability;
            is >> std::ws >> skewness;
            is >> std::ws >> location;
            is >> std::ws >> scale;

            if (is.good()) {
                if (stability > 0 && stability <= 2 &&
                        skewness >= -1 && skewness <= 1 && scale > 0) {
                    param.stability_ = stability;
                    param.skewness_  = skewness;
                    param.location_  = location;
                    param.scale_     = scale;
                } else {
                    is.setstate(std::ios_base::failbit);
                }
            }

            return is;
        }

        private :

        result_type stability_;
        result_type skewness_;
        result_type location_;
        result_type scale_;
    }; // class param_type

    explicit StableDistribution (
            result_type stability = 1, result_type skewness = 0,
            result_type location = 0, result_type scale = 1) :
        stability_(stability), skewness_(skewness),
        location_(location), scale_(scale),
        zeta_(0), xi_(0), stability_1_(false)
    {
        VSMC_RUNTIME_ASSERT_RNG_STABLE_DISTRIBUTION_PARAM_CHECK_STABILITY(
                stability_);
        VSMC_RUNTIME_ASSERT_RNG_STABLE_DISTRIBUTION_PARAM_CHECK_SKEWNESS(
                skewness_);
        VSMC_RUNTIME_ASSERT_RNG_STABLE_DISTRIBUTION_PARAM_CHECK_SCALE(
                scale_);
        invariant();
    }

    StableDistribution (const param_type &param) :
        stability_(param.stability()), skewness_(param.skewness()),
        location_(param.location()), scale_(param.scale()),
        zeta_(0), xi_(0), stability_1_(false)
    {
        VSMC_RUNTIME_ASSERT_RNG_STABLE_DISTRIBUTION_PARAM_CHECK_STABILITY(
                stability_);
        VSMC_RUNTIME_ASSERT_RNG_STABLE_DISTRIBUTION_PARAM_CHECK_SKEWNESS(
                skewness_);
        VSMC_RUNTIME_ASSERT_RNG_STABLE_DISTRIBUTION_PARAM_CHECK_SCALE(
                scale_);
        invariant();
    }

    param_type param () const
    {return param_type(stability_, skewness_, location_, scale_);}

    void param (const param_type &param)
    {
        stability_ = param.stability();
        skewness_  = param.skewness();
        location_  = param.location();
        scale_     = param.scale();
        invariant();
    }

    void reset () const {}

    result_type stability () const {return stability_;}
    result_type skewness  () const {return skewness_;}
    result_type location  () const {return location_;}
    result_type scale     () const {return scale_;}
    result_type min VSMC_MNE () const
    {return -std::numeric_limits<result_type>::infinity();}
    result_type max VSMC_MNE () const
    {return std::numeric_limits<result_type>::infinity();}

    template <typename Eng>
    result_type operator() (Eng &eng) const
    {
        if (stability_1_)
            return trans_1(standard_1(eng));
        else
            return trans_a(standard_a(eng));
    }

    friend inline bool operator== (
            const StableDistribution<FPType> &rstable1,
            const StableDistribution<FPType> &rstable2)
    {
        if (rstable1.stability_ < rstable2.stability_ ||
                rstable1.stability_ > rstable2.stability_)
            return false;
        if (rstable1.skewness_ < rstable2.skewness_ ||
                rstable1.skewness_ > rstable2.skewness_)
            return false;
        if (rstable1.location_ < rstable2.location_ ||
                rstable1.location_ > rstable2.location_)
            return false;
        if (rstable1.scale_ < rstable2.scale_ ||
                rstable1.scale_ > rstable2.scale_)
            return false;
        return true;
    }

    friend inline bool operator!= (
            const StableDistribution<FPType> &rstable1,
            const StableDistribution<FPType> &rstable2)
    {return !(rstable1 == rstable2);}

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &operator<< (
            std::basic_ostream<CharT, Traits> &os,
            const StableDistribution<FPType> &rstable)
    {
        if (!os.good())
            return os;

        os << rstable.stability_ << ' ' << rstable.skewness_ << ' ';
        os << rstable.location_ << ' ' << rstable.scale_ << ' ';

        return os;
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &operator>> (
            std::basic_istream<CharT, Traits> &is,
            StableDistribution<FPType> &rstable)
    {
        if (!is.good())
            return is;

        result_type stability = 0;
        result_type skewness = 0;
        result_type location = 0;
        result_type scale = 0;
        is >> std::ws >> stability;
        is >> std::ws >> skewness;
        is >> std::ws >> location;
        is >> std::ws >> scale;

        if (is.good()) {
            if (stability > 0 && stability <= 2 &&
                    skewness >= -1 && skewness <= 1 && scale > 0) {
                rstable.stability_ = stability;
                rstable.skewness_  = skewness;
                rstable.location_  = location;
                rstable.scale_     = scale;
            } else {
                is.setstate(std::ios_base::failbit);
            }
        }

        return is;
    }

    private :

    result_type stability_;
    result_type skewness_;
    result_type location_;
    result_type scale_;
    result_type zeta_;
    result_type xi_;
    bool stability_1_;

    template <typename Eng>
    result_type standard_1 (Eng &eng) const
    {
        using std::cos;
        using std::log;
        using std::tan;

        cxx11::uniform_real_distribution<result_type> runif(0, 1);
        result_type w = -log(runif(eng));
        result_type u = (runif(eng) - 0.5) * math::pi<result_type>();
        result_type a = (math::pi_by2<result_type>() + skewness_ * u) * tan(u);
        result_type b = log(math::pi_by2<result_type>() * w * cos(u));
        result_type c = log(math::pi_by2<result_type>() + skewness_ * u);
        result_type x = (a - skewness_ * (b - c)) / xi_;

        return x;
    }

    template <typename Eng>
    result_type standard_a (Eng &eng) const
    {
        using std::cos;
        using std::exp;
        using std::log;
        using std::sin;

        cxx11::uniform_real_distribution<result_type> runif(0, 1);
        result_type w = -log(runif(eng));
        result_type u = (runif(eng) - 0.5) * math::pi<result_type>();
        result_type a = 0.5 * log(1 + zeta_ * zeta_) / stability_;
        result_type b = sin(stability_ * (u + xi_));
        result_type c = log(cos(u)) / stability_;
        result_type d = (1 - stability_) / stability_ * log(
                cos(u - stability_ * (u + xi_)) / w);
        result_type x = b * std::exp(a - c + d);

        return x;
    }

    result_type trans_1 (result_type x) const
    {
        using std::log;

        return scale_ * x + location_ +
            2 * math::pi_inv<result_type>() * skewness_ * scale_ * log(scale_);
    }

    result_type trans_a (result_type x) const
    {return scale_ * x + location_;}

    void invariant ()
    {
        using std::atan;
        using std::tan;

        if (stability_ < 1 || stability_ > 1) {
            stability_1_ = false;
            zeta_ = -skewness_ * tan(math::pi_by2<result_type>() * stability_);
            xi_ = 1 / stability_ * atan(-zeta_);
        } else {
            stability_1_ = true;
            zeta_ = -std::numeric_limits<result_type>::infinity();
            xi_ = math::pi_by2<result_type>();
        }
    }
}; // class StableDistributionBase

} // namespace vsmc

#endif // VSMC_RNG_STABLE_DISTRIBUTION_HPP
