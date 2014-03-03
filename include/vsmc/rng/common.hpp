#ifndef VSMC_RNG_COMMON_HPP
#define VSMC_RNG_COMMON_HPP

#include <vsmc/internal/assert.hpp>
#include <vsmc/internal/defines.hpp>
#include <vsmc/internal/forward.hpp>
#include <vsmc/cxx11/random.hpp>
#include <vsmc/cxx11/type_traits.hpp>
#include <iostream>
#include <stdint.h>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4351)
#endif

namespace vsmc {

namespace internal {

template <typename ResultType>
inline void rng_array_shift (ResultType *state, Position<2>)
{
    state[0] = state[1];
}

template <typename ResultType, std::size_t K>
inline void rng_array_shift (ResultType *state, Position<K>)
{
    state[0] = state[1];
    rng_array_shift(state + 1, Position<K - 1>());
}

template <typename ResultType, std::size_t K, bool B>
class RngStorage
{
    public :

    RngStorage () : state_() {}

    RngStorage (const RngStorage<ResultType, K, B> &other) :
        state_()
    {
        for (std::size_t i = 0; i != K; ++i)
            state_[i] = other.state_[i];
    }

    RngStorage<ResultType, K, B> &operator= (
            const RngStorage<ResultType, K, B> &other)
    {
        for (std::size_t i = 0; i != K; ++i)
            state_[i] = other.state_[i];
    }

    ResultType &operator[] (std::size_t i) {return state_[i];}
    const ResultType &operator[] (std::size_t i) const {return state_[i];}

    operator ResultType * () {return state_;}
    operator const ResultType * () const {return state_;}

    ResultType *data () {return state_;}
    const ResultType *data () const {return state_;}

    private :

    ResultType state_[K];
}; // class RngStorage

template <typename ResultType, std::size_t K>
class RngStorage<ResultType, K, false>
{
    public :

    RngStorage () : state_(new ResultType[K]) {}

    RngStorage (const RngStorage<ResultType, K, false> &other) :
        state_(new ResultType[K])
    {
        for (std::size_t i = 0; i != K; ++i)
            state_[i] = other.state_[i];
    }

    RngStorage<ResultType, K, false> &operator= (
            const RngStorage<ResultType, K, false> &other)
    {
        if (this != &other) {
            for (std::size_t i = 0; i != K; ++i)
                state_[i] = other.state_[i];
        }

        return *this;
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    RngStorage (RngStorage<ResultType, K, false> &&other) :
        state_(other.state_)
    {other.state_ = VSMC_NULLPTR;}

    RngStorage<ResultType, K, false> &operator= (
            RngStorage<ResultType, K, false> &&other)
    {
        if (this != &other) {
            if (state_ != VSMC_NULLPTR)
                delete [] state_;
            state_ = other.state_;
            other.state_ = VSMC_NULLPTR;
        }

        return *this;
    }
#endif

    ~RngStorage ()
    {
        if (state_ != VSMC_NULLPTR)
            delete [] state_;
    }

    ResultType &operator[] (std::size_t i) {return state_[i];}
    const ResultType &operator[] (std::size_t i) const {return state_[i];}

    operator ResultType * () {return state_;}
    operator const ResultType * () const {return state_;}

    ResultType *data () {return state_;}
    const ResultType *data () const {return state_;}

    private :

    ResultType *state_;
}; // class RngStorage

} // namespace vsmc::internal

} // namespace vsmc

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif // VSMC_RNG_COMMON_HPP
