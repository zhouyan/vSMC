//============================================================================
// vSMC/include/vsmc/rng/rng_set.hpp
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

#ifndef VSMC_RNG_RNG_SET_HPP
#define VSMC_RNG_RNG_SET_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/seed.hpp>
#include <vsmc/rng/engine.hpp>
#if VSMC_HAS_TBB
#include <tbb/tbb.h>
#endif

/// \brief Default RNG set type
/// \ingroup Config
#ifndef VSMC_RNG_SET_TYPE
#define VSMC_RNG_SET_TYPE ::vsmc::RNGSetVector<::vsmc::RNG>
#endif

namespace vsmc
{

/// \brief Scalar RNG set
/// \ingroup RNG
template <typename RNGType>
class RNGSetScalar
{
    public:
    using rng_type = RNGType;
    using size_type = std::size_t;

    explicit RNGSetScalar(size_type N = 0) : size_(N) { seed(); }

    size_type size() const { return size_; }

    void resize(std::size_t) {}

    void seed() { Seed::instance().seed_rng(rng_); }

    rng_type &operator[](size_type) { return rng_; }

    private:
    std::size_t size_;
    rng_type rng_;
}; // class RNGSetScalar

/// \brief Vector RNG set
/// \ingroup RNG
template <typename RNGType>
class RNGSetVector
{
    public:
    using rng_type = RNGType;
    using size_type = typename AlignedVector<rng_type>::size_type;

    explicit RNGSetVector(size_type N = 0) : rng_(N, rng_type()) { seed(); }

    size_type size() const { return rng_.size(); }

    void resize(std::size_t n)
    {
        if (n == rng_.size())
            return;

        if (n < rng_.size())
            rng_.resize(n);

        rng_.reserve(n);
        rng_type rng;
        for (std::size_t i = rng_.size(); i != n; ++i) {
            Seed::instance().seed_rng(rng);
            rng_.push_back(rng);
        }
    }

    void seed()
    {
        for (auto &rng : rng_)
            Seed::instance().seed_rng(rng);
    }

    rng_type &operator[](size_type id) { return rng_[id]; }

    private:
    AlignedVector<rng_type> rng_;
}; // class RNGSetVector

using RNGSet = VSMC_RNG_SET_TYPE;

/// \brief Particle::rng_set_type trait
/// \ingroup Traits
VSMC_DEFINE_TYPE_DISPATCH_TRAIT(RNGSetType, rng_set_type, RNGSet)

} // namespace vsmc

#endif // VSMC_RNG_RNG_SET_HPP
