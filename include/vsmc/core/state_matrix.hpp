//============================================================================
// vSMC/include/vsmc/core/state_matrix.hpp
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

#ifndef VSMC_CORE_STATE_MATRIX_HPP
#define VSMC_CORE_STATE_MATRIX_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/single_particle.hpp>

#define VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_RESIZE_DIM                      \
    VSMC_RUNTIME_ASSERT((dim == this->dim() || Dim == Dynamic),               \
        "**StateMatrix::resize** CHANGE DIMENSION OF A FIXED DIMENSION "      \
        "OBJECT")

#define VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_UNPACK_SIZE(psize, dim)         \
    VSMC_RUNTIME_ASSERT((psize >= dim),                                       \
        "**StateMatrix::state_unpack** INPUT PACK SIZE TOO SMALL")

namespace vsmc
{

namespace internal
{

template <std::size_t Dim>
class StateMatrixDim
{
    public:
    static constexpr std::size_t dim() { return Dim; }

    protected:
    void swap(StateMatrixDim<Dim> &) noexcept {}

    static void set_dim(std::size_t) {}
}; // class StateMatrixDim

template <>
class StateMatrixDim<Dynamic>
{
    public:
    StateMatrixDim() : dim_(0) {}

    std::size_t dim() const { return dim_; }

    protected:
    void swap(StateMatrixDim<Dynamic> &other) noexcept
    {
        std::swap(dim_, other.dim_);
    }

    void set_dim(std::size_t dim) { dim_ = dim; }

    private:
    std::size_t dim_;
}; // class StateMatrixDim

} // namespace vsmc::internal

/// \brief Base type of StateMatrix
/// \ingroup Core
template <MatrixLayout Layout, std::size_t Dim, typename T>
class StateMatrixBase : public internal::StateMatrixDim<Dim>
{
    public:
    using size_type = std::size_t;
    using state_type = T;
    using state_pack_type = Vector<T>;

    template <typename S>
    class single_particle_type : public SingleParticleBase<S>
    {
        public:
        single_particle_type(
            typename Particle<S>::size_type id, Particle<S> *pptr)
            : SingleParticleBase<S>(id, pptr)
        {
        }

        std::size_t dim() const { return this->particle().value().dim(); }

        state_type &state(std::size_t pos) const
        {
            return this->particle().value().state(
                static_cast<size_type>(this->id()), pos);
        }
    }; // class single_particle_type

    size_type size() const { return size_; }

    void resize(size_type N)
    {
        static_cast<StateMatrix<Layout, Dim, T> *>(this)->resize(
            N, this->dim());
    }

    void resize_dim(std::size_t dim)
    {
        static_cast<StateMatrix<Layout, Dim, T> *>(this)->resize(
            this->size(), dim);
    }

    state_type *data() { return data_.data(); }

    const state_type *data() const { return data_.data(); }

    void swap(StateMatrixBase<Layout, Dim, T> &other) noexcept
    {
        internal::StateMatrixDim<Dim>::swap(other);
        std::swap(size_, other.size_);
        data_.swap(other.data_);
    }

    template <typename OutputIter>
    OutputIter read_state(std::size_t pos, OutputIter first) const
    {
        auto sptr = static_cast<const StateMatrix<Layout, Dim, T> *>(this);
        for (size_type i = 0; i != size_; ++i, ++first)
            *first = sptr->state(i, pos);

        return first;
    }

    template <typename OutputIterIter>
    OutputIterIter read_state_list(OutputIterIter first) const
    {
        for (std::size_t d = 0; d != this->dim(); ++d, ++first)
            read_state(d, *first);

        return first;
    }

    template <typename OutputIter>
    OutputIter read_state_matrix(MatrixLayout layout, OutputIter first) const
    {
        if (layout == Layout)
            return std::copy(data_.begin(), data_.end(), first);

        auto sptr = static_cast<const StateMatrix<Layout, Dim, T> *>(this);

        if (layout == RowMajor) {
            for (size_type i = 0; i != size_; ++i)
                for (std::size_t d = 0; d != this->dim(); ++d)
                    *first++ = sptr->state(i, d);
        }

        if (layout == ColMajor) {
            for (std::size_t d = 0; d != this->dim(); ++d)
                for (size_type i = 0; i != size_; ++i)
                    *first++ = sptr->state(i, d);
        }

        return first;
    }

    template <typename CharT, typename Traits>
    std::basic_ostream<CharT, Traits> &print(
        std::basic_ostream<CharT, Traits> &os, char sepchar = '\t') const
    {
        if (this->dim() == 0 || size_ == 0 || !os.good())
            return os;

        auto sptr = static_cast<const StateMatrix<Layout, Dim, T> *>(this);
        for (size_type i = 0; i != size_; ++i) {
            for (std::size_t d = 0; d != this->dim() - 1; ++d)
                os << sptr->state(i, d) << sepchar;
            os << sptr->state(i, this->dim() - 1) << '\n';
        }

        return os;
    }

    protected:
    explicit StateMatrixBase(size_type N = 0) : size_(N), data_(N * Dim) {}

    StateMatrixBase(const StateMatrixBase<Layout, Dim, T> &) = default;

    StateMatrixBase<Layout, Dim, T> &operator=(
        const StateMatrixBase<Layout, Dim, T> &) = default;

    StateMatrixBase(StateMatrixBase<Layout, Dim, T> &&) = default;

    StateMatrixBase<Layout, Dim, T> &operator=(
        StateMatrixBase<Layout, Dim, T> &&other) noexcept
    {
        swap(other);

        return *this;
    }

    void resize_data(size_type N, std::size_t dim)
    {
        size_ = N;
        this->set_dim(dim);
        data_.resize(N * dim);
    }

    private:
    size_type size_;
    Vector<T> data_;
}; // class StateMatrixBase

/// \brief Particle::value_type subtype
/// \ingroup Core
template <std::size_t Dim, typename T>
class StateMatrix<RowMajor, Dim, T> : public StateMatrixBase<RowMajor, Dim, T>
{
    public:
    using state_matrix_base_type = StateMatrixBase<RowMajor, Dim, T>;
    using size_type = typename state_matrix_base_type::size_type;
    using state_type = typename state_matrix_base_type::state_type;
    using state_pack_type = typename state_matrix_base_type::state_pack_type;

    explicit StateMatrix(size_type N = 0) : state_matrix_base_type(N) {}

    using state_matrix_base_type::resize;

    void resize(size_type N, std::size_t dim)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_RESIZE_DIM;

        if (N == this->size() && dim == this->dim())
            return;

        if (dim == this->dim()) {
            this->resize_data(N, dim);
            return;
        }

        StateMatrix<RowMajor, Dim, T> tmp(0);
        tmp.resize_data(N, dim);
        const size_type K = std::min(N, this->size());
        const std::size_t D = std::min(dim, this->dim());
        if (D > 0)
            for (size_type k = 0; k != K; ++k)
                std::copy_n(row_data(k), D, tmp.row_data(k));
        *this = std::move(tmp);
    }

    T &state(size_type id, std::size_t pos)
    {
        return this->data()[id * this->dim() + pos];
    }

    const T &state(size_type id, std::size_t pos) const
    {
        return this->data()[id * this->dim() + pos];
    }

    state_type *row_data(size_type id)
    {
        return this->data() + id * this->dim();
    }

    const state_type *row_data(size_type id) const
    {
        return this->data() + id * this->dim();
    }

    template <typename InputIter>
    InputIter copy(size_type N, InputIter index)
    {
        if (N > this->size())
            this->resize(N);

        for (size_type dst = 0; dst != N; ++dst, ++index)
            copy_particle(static_cast<size_type>(*index), dst);

        if (N < this->size())
            this->resize(N);

        return index;
    }

    void copy_particle(size_type src, size_type dst)
    {
        if (src == dst)
            return;

        copy_particle_dispatch(src, dst, std::integral_constant < bool,
            Dim == Dynamic || 8 < Dim > ());
    }

    state_pack_type state_pack(size_type id) const
    {
        state_pack_type pack(this->dim());
        std::copy(row_data(id), row_data(id) + this->dim(), pack.data());

        return pack;
    }

    void state_unpack(size_type id, const state_pack_type &pack)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_UNPACK_SIZE(
            pack.size(), this->dim());

        const state_type *ptr = pack.data();
        std::copy(ptr, ptr + this->dim(), row_data(id));
    }

    void state_unpack(size_type id, state_pack_type &&pack)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_UNPACK_SIZE(
            pack.size(), this->dim());

        const state_type *ptr = pack.data();
        std::move(ptr, ptr + this->dim(), row_data(id));
    }

    private:
    void copy_particle_dispatch(size_type src, size_type dst, std::true_type)
    {
        std::copy(row_data(src), row_data(src) + this->dim(), row_data(dst));
    }

    void copy_particle_dispatch(size_type src, size_type dst, std::false_type)
    {
        copy_particle_pos<0>(row_data(src), row_data(dst),
            std::integral_constant<bool, 0 < Dim>());
    }

    template <std::size_t D>
    void copy_particle_pos(const state_type *, state_type *, std::false_type)
    {
    }

    template <std::size_t D>
    void copy_particle_pos(
        const state_type *src, state_type *dst, std::true_type)
    {
        dst[D] = src[D];
        copy_particle_pos<D + 1>(
            src, dst, std::integral_constant<bool, D + 1 < Dim>());
    }
}; // class StateMatrix

/// \brief Particle::value_type subtype
/// \ingroup Core
template <std::size_t Dim, typename T>
class StateMatrix<ColMajor, Dim, T> : public StateMatrixBase<ColMajor, Dim, T>
{
    public:
    using state_matrix_base_type = StateMatrixBase<ColMajor, Dim, T>;
    using size_type = typename state_matrix_base_type::size_type;
    using state_type = typename state_matrix_base_type::state_type;
    using state_pack_type = typename state_matrix_base_type::state_pack_type;

    explicit StateMatrix(size_type N = 0) : state_matrix_base_type(N) {}

    using state_matrix_base_type::resize;

    void resize(size_type N, std::size_t dim)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_RESIZE_DIM;

        if (N == this->size() && dim == this->dim())
            return;

        if (N == this->size()) {
            this->resize_data(N, dim);
            return;
        }

        StateMatrix<ColMajor, Dim, T> tmp(0);
        tmp.resize_data(N, dim);
        const size_type K = std::min(N, this->size());
        const std::size_t D = std::min(dim, this->dim());
        if (K > 0)
            for (std::size_t d = 0; d != D; ++d)
                std::copy_n(col_data(d), K, tmp.col_data(d));
        *this = std::move(tmp);
    }

    T &state(size_type id, std::size_t pos)
    {
        return this->data()[pos * this->size() + id];
    }

    const T &state(size_type id, std::size_t pos) const
    {
        return this->data()[pos * this->size() + id];
    }

    state_type *col_data(std::size_t pos)
    {
        return this->data() + pos * this->size();
    }

    const state_type *col_data(std::size_t pos) const
    {
        return this->data() + pos * this->size();
    }

    template <typename InputIter>
    InputIter copy(size_type N, InputIter index)
    {
        InputIter idx = index;

        if (N == this->size()) {
            for (std::size_t d = 0; d != this->dim(); ++d) {
                idx = index;
                for (size_type dst = 0; dst != N; ++dst, ++idx)
                    state(dst, d) = state(static_cast<size_type>(*idx), d);
            }
        } else {
            StateMatrix<ColMajor, Dim, T> tmp;
            tmp.resize(N, this->dim());
            for (std::size_t d = 0; d != this->dim(); ++d) {
                idx = index;
                for (size_type dst = 0; dst != N; ++dst, ++idx)
                    tmp.state(dst, d) = state(static_cast<size_type>(*idx), d);
            }
            *this = std::move(tmp);
        }

        return idx;
    }

    void copy_particle(size_type src, size_type dst)
    {
        if (src == dst)
            return;

        copy_particle_dispatch(src, dst, std::integral_constant < bool,
            Dim == Dynamic || 8 < Dim > ());
    }

    state_pack_type state_pack(size_type id) const
    {
        state_pack_type pack(this->dim());
        for (std::size_t d = 0; d != this->dim(); ++d)
            pack[d] = state(id, d);

        return pack;
    }

    void state_unpack(size_type id, const state_pack_type &pack)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_UNPACK_SIZE(
            pack.size(), this->dim());

        for (std::size_t d = 0; d != this->dim(); ++d)
            state(id, d) = pack[d];
    }

    void state_unpack(size_type id, state_pack_type &&pack)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_UNPACK_SIZE(
            pack.size(), this->dim());

        for (std::size_t d = 0; d != this->dim(); ++d)
            state(id, d) = std::move(pack[d]);
    }

    private:
    void copy_particle_dispatch(size_type src, size_type dst, std::true_type)
    {
        for (std::size_t d = 0; d != this->dim(); ++d)
            state(dst, d) = state(src, d);
    }

    void copy_particle_dispatch(size_type src, size_type dst, std::false_type)
    {
        copy_particle_pos<0>(this->data() + src, this->data() + dst,
            std::integral_constant<bool, 0 < Dim>());
    }

    template <std::size_t D>
    void copy_particle_pos(const state_type *, state_type *, std::false_type)
    {
    }

    template <std::size_t D>
    void copy_particle_pos(
        const state_type *src, state_type *dst, std::true_type)
    {
        dst[D * this->size()] = src[D * this->size()];
        copy_particle_pos<D + 1>(
            src, dst, std::integral_constant<bool, D + 1 < Dim>());
    }
}; // class StateMatrix

template <typename CharT, typename Traits, MatrixLayout Layout,
    std::size_t Dim, typename T>
inline std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os,
    const StateMatrix<Layout, Dim, T> &state)
{
    return state.print(os);
}

template <MatrixLayout Layout, std::size_t Dim, typename T>
inline bool operator==(const StateMatrix<Layout, Dim, T> &state1,
    const StateMatrix<Layout, Dim, T> &state2)
{
    if (state1.size() != state2.size())
        return false;
    if (state1.dim() != state2.dim())
        return false;
    std::size_t N = state1.size() * state1.dim();
    const T *s1 = state1.data();
    const T *s2 = state2.data();
    for (std::size_t i = 0; i != N; ++i)
        if (!internal::is_equal(s1[i] != s2[i]))
            return false;
    return true;
}

template <MatrixLayout Layout, std::size_t Dim, typename T>
inline bool operator!=(const StateMatrix<Layout, Dim, T> &state1,
    const StateMatrix<Layout, Dim, T> &state2)
{
    return !(state1 == state2);
}

template <MatrixLayout Layout, std::size_t Dim, typename T>
inline void swap(StateMatrix<Layout, Dim, T> &state1,
    StateMatrix<Layout, Dim, T> &state2) noexcept
{
    state1.swap(state2);
}

} // namespace vsmc

#endif // VSMC_CORE_STATE_MATRIX_HPP
