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

#if VSMC_HAS_AES_NI
#include <vsmc/rng/aes.hpp>
#include <vsmc/rng/ars.hpp>
#endif

#include <vsmc/rng/philox.hpp>
#include <vsmc/rng/threefry.hpp>

#if VSMC_HAS_TBB
#include <tbb/tbb.h>
#if VSMC_HAS_MKL
#include <vsmc/rng/mkl.hpp>
#endif
#endif

/// \brief Default RNG set type
/// \ingroup Config
#ifndef VSMC_RNG_SET_TYPE
#if VSMC_USE_TBB
#define VSMC_RNG_SET_TYPE ::vsmc::RngSetTBB<::vsmc::Threefry4x32>
#else
#define VSMC_RNG_SET_TYPE ::vsmc::RngSetVector<::vsmc::Threefry4x32>
#endif
#endif

namespace vsmc
{

/// \brief Scalar RNG set
/// \ingroup RNG
template <typename RngType>
class RngSetScalar
{
    public:
    typedef RngType rng_type;
    typedef std::size_t size_type;

    explicit RngSetScalar(size_type N = 0) : size_(N) { seed(); }

    size_type size() const { return size_; }

    void resize(std::size_t) {}

    void seed() { Seed::instance().seed_rng(rng_); }

    rng_type &operator[](size_type) { return rng_; }

    private:
    std::size_t size_;
    rng_type rng_;
}; // class RngSetScalar

/// \brief Vector RNG set
/// \ingroup RNG
template <typename RngType>
class RngSetVector
{
    public:
    typedef RngType rng_type;
    typedef typename std::vector<rng_type>::size_type size_type;

    explicit RngSetVector(size_type N = 0) : rng_(N, rng_type()) { seed(); }

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
    std::vector<rng_type> rng_;
}; // class RngSetVector

#if VSMC_HAS_TBB

/// \brief Thread local RNG set
/// \ingroup RNG
template <typename RngType>
class RngSetTBB
{
    public:
    typedef RngType rng_type;
    typedef std::size_t size_type;

    explicit RngSetTBB(size_type N = 0) : size_(N), rng_(rng_init)
    {
        rng_init();
    }

    size_type size() const { return size_; }

    void resize(std::size_t) {}

    void seed() { rng_.clear(); }

    rng_type &operator[](size_type) { return rng_.local(); }

    private:
    std::size_t size_;
    ::tbb::combinable<rng_type> rng_;

    static rng_type rng_init()
    {
        static ::tbb::mutex mtx;

        ::tbb::mutex::scoped_lock lock(mtx);
        rng_type rng;
        Seed::instance().seed_rng(rng);

        return rng;
    }
}; // class RngSetTBB

#if VSMC_HAS_MKL

template <typename>
class RngSetMKL;

/// \brief Thread local RNG set using MKL RNG and TBB
/// \ingroup RNG
template <MKL_INT BRNG, typename ResultType>
class RngSetMKL<MKLEngine<BRNG, ResultType>>
{
    public:
    typedef MKLEngine<BRNG, ResultType> rng_type;
    typedef std::size_t size_type;

    explicit RngSetMKL(size_type N = 0) : size_(N), rng_(rng_init)
    {
        rng_init();
    }

    size_type size() const { return size_; }

    void resize(std::size_t) {}

    void seed() { rng_.clear(); }

    rng_type &operator[](size_type) { return rng_.local(); }

    private:
    std::size_t size_;
    ::tbb::combinable<rng_type> rng_;

    static rng_type rng_init()
    {
        static ::tbb::mutex mtx;

        ::tbb::mutex::scoped_lock lock(mtx);
        MKL_UINT seed = static_cast<MKL_UINT>(Seed::instance().get_scalar());
        MKL_INT offset_max = internal::MKLOffset<BRNG>::type::max VSMC_MNE();
        MKL_INT offset = 0;
        if (offset_max != 0) {
            offset = SeedGenerator<rng_type, MKL_INT>::instance().get() %
                offset_max;
        }
        rng_type rng(seed, offset);

        return rng;
    }
}; // class RngSetMKL

#endif // VSMC_HAS_MKL

#endif // VSMC_HAS_TBB

namespace traits
{

/// \brief Particle::rng_set_type trait
/// \ingroup Traits
VSMC_DEFINE_TYPE_DISPATCH_TRAIT(RngSetType, rng_set_type, VSMC_RNG_SET_TYPE)

} // namespace vsmc::traits

} // namespace vsmc

#endif // VSMC_RNG_RNG_SET_HPP
