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

#define VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_UNPACK_SIZE(psize, dim)         \
    VSMC_RUNTIME_ASSERT((psize >= dim),                                       \
        "**StateMatrix::state_unpack** INPUT PACK SIZE TOO SMALL")

#define VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_INDEX(flag)                     \
    VSMC_RUNTIME_ASSERT(flag, "**StateMatrix** INDEX OUT OF RANGE");

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
    using value_type = T;
    using pack_type = Vector<T>;

    template <typename S>
    class particle_index_type : public ParticleIndexBase<S>
    {
        public:
        particle_index_type(
            typename Particle<S>::size_type id, Particle<S> *pptr)
            : ParticleIndexBase<S>(id, pptr)
        {
        }

        std::size_t dim() const { return this->particle().state().dim(); }

        value_type &operator()(std::size_t pos) const
        {
            return this->particle().state()(
                static_cast<size_type>(this->id()), pos);
        }

        value_type &at(std::size_t pos) const
        {
            VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_INDEX(
                (!internal::is_negative(pos) && pos < this->dim()));

            return operator()(pos);
        }
    }; // class particle_index_type

    /// \brief The numbrer of particles
    size_type size() const { return size_; }

    /// \brief Reserve space for specified number of particles
    void reserve(size_type N) { data_.reserve(N * this->dim()); }

    /// \brief Reserve space for specified number of particles and dimensions
    ///
    /// \details
    /// `dim` is ignored if `Dim > 0`.
    void reserve(size_type N, std::size_t dim)
    {
        data_.reserve(N * (Dim == Dynamic ? dim : this->dim()));
    }

    void shrink_to_fit() { data_.shrink_to_fit(); }

    value_type *data() { return data_.data(); }

    const value_type *data() const { return data_.data(); }

    void swap(StateMatrixBase<Layout, Dim, T> &other) noexcept
    {
        internal::StateMatrixDim<Dim>::swap(other);
        std::swap(size_, other.size_);
        data_.swap(other.data_);
    }

    protected:
    explicit StateMatrixBase(size_type N) : size_(N), data_(N * Dim) {}

    StateMatrixBase(size_type N, std::size_t dim) : size_(N)
    {
        static_assert(Dim == Dynamic, "**StateMatrix::StateMatrix** USED WITH "
                                      "AN OBJECT WITH FIXED DIMENSION");
        resize_data(N, dim);
    }

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

    std::size_t data_size() const { return data_.size(); }

    friend bool operator==(const StateMatrixBase<Layout, Dim, T> &state1,
        const StateMatrixBase<Layout, Dim, T> &state2)
    {
        if (state1.dim() != state2.dim())
            return false;
        if (state1.size_ != state2.size_)
            return false;
        if (state1.data_ != state2.data_)
            return false;
        return true;
    }

    friend bool operator!=(const StateMatrixBase<Layout, Dim, T> &state1,
        const StateMatrixBase<Layout, Dim, T> &state2)
    {
        return !(state1 == state2);
    }

    private:
    size_type size_;
    Vector<T> data_;
}; // class StateMatrixBase

/// \brief Swap two StateMatrixBase objects
/// \ingroup Core
template <MatrixLayout Layout, std::size_t Dim, typename T>
inline void swap(StateMatrixBase<Layout, Dim, T> &state1,
    StateMatrixBase<Layout, Dim, T> &state2) noexcept
{
    state1.swap(state2);
}

/// \brief Particle::value_type subtype
/// \ingroup Core
template <std::size_t Dim, typename T>
class StateMatrix<RowMajor, Dim, T> : public StateMatrixBase<RowMajor, Dim, T>
{
    public:
    using state_matrix_base_type = StateMatrixBase<RowMajor, Dim, T>;
    using size_type = typename state_matrix_base_type::size_type;
    using value_type = typename state_matrix_base_type::value_type;
    using pack_type = typename state_matrix_base_type::pack_type;

    explicit StateMatrix(size_type N = 0) : state_matrix_base_type(N) {}

    StateMatrix(size_type N, std::size_t dim) : state_matrix_base_type(N, dim)
    {
    }

    void resize(size_type N) { resize_both(N, this->dim()); }

    void resize(size_type N, std::size_t dim)
    {
        static_assert(Dim == Dynamic, "**StateMatrix::resize** USED WITH AN "
                                      "OBJECT WITH FIXED DIMENSION");
        resize_both(N, dim);
    }

    void resize_dim(std::size_t dim)
    {
        static_assert(Dim == Dynamic, "**StateMatrix::resize_dim** USED WITH "
                                      "AN OBJECT WITH FIXED DIMENSION");
        resize_both(this->size(), dim);
    }

    T &operator()(size_type id, std::size_t pos)
    {
        return this->data()[id * this->dim() + pos];
    }

    const T &operator()(size_type id, std::size_t pos) const
    {
        return this->data()[id * this->dim() + pos];
    }

    T &at(size_type id, std::size_t pos)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_INDEX(
            (!internal::is_negative(id) && id < this->size() &&
                !internal::is_negative(pos) && pos < this->dim()));

        return operator()(id, pos);
    }

    const T &at(size_type id, std::size_t pos) const
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_INDEX(
            (!internal::is_negative(id) && id < this->size() &&
                !internal::is_negative(pos) && pos < this->dim()));

        return operator()(id, pos);
    }

    template <typename OutputIter>
    OutputIter read_state(std::size_t pos, OutputIter first) const
    {
        for (size_type i = 0; i != this->size(); ++i, ++first)
            *first = operator()(i, pos);

        return first;
    }

    template <typename OutputIter>
    OutputIter read_state_matrix(MatrixLayout layout, OutputIter first) const
    {
        if (layout == RowMajor)
            first = std::copy_n(this->data(), this->data_size(), first);

        if (layout == ColMajor)
            for (std::size_t d = 0; d != this->dim(); ++d)
                for (size_type i = 0; i != this->size(); ++i, ++first)
                    *first = operator()(i, d);

        return first;
    }

    value_type *row_data(size_type id)
    {
        return this->data() + id * this->dim();
    }

    const value_type *row_data(size_type id) const
    {
        return this->data() + id * this->dim();
    }

    /// \brief Copy particles
    ///
    /// \param N The new sample size
    /// \param index N-vector of parent index
    ///
    /// \details
    /// Let \f$a_i\f$ denote the value of `index[i]`, and
    /// \f$r_i = \sum_{j=1}^N \mathbb{I}_{\{i\}}(a_j)\f$. Then it is required
    /// that \f$a_i = i\f$ for all \f$r_i > 0\f$. This condition is always
    /// satisfied if `index` comes from `resamle_trans_rep_index`.
    template <typename IntType, typename InputIter>
    void select(IntType N, InputIter index)
    {
        size_type n = static_cast<size_type>(N);
        if (this->size() == 0 || internal::is_nullptr(index)) {
            this->resize(n);
            return;
        }

        if (n > this->size())
            this->resize(n);
        for (size_type dst = 0; dst != n; ++dst, ++index)
            duplicate(static_cast<size_type>(*index), dst);
        if (n < this->size())
            this->resize(n);

        return;
    }

    void duplicate(size_type src, size_type dst)
    {
        if (src == dst)
            return;

        duplicate_dispatch(src, dst, std::integral_constant < bool,
            Dim == Dynamic || 8 < Dim > ());
    }

    pack_type state_pack(size_type id) const
    {
        pack_type pack(this->dim());
        std::copy(row_data(id), row_data(id) + this->dim(), pack.data());

        return pack;
    }

    void state_unpack(size_type id, const pack_type &pack)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_UNPACK_SIZE(
            pack.size(), this->dim());

        const value_type *ptr = pack.data();
        std::copy(ptr, ptr + this->dim(), row_data(id));
    }

    void state_unpack(size_type id, pack_type &&pack)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_UNPACK_SIZE(
            pack.size(), this->dim());

        const value_type *ptr = pack.data();
        std::move(ptr, ptr + this->dim(), row_data(id));
    }

    private:
    void resize_both(size_type N, std::size_t dim)
    {
        if (N == this->size() && dim == this->dim())
            return;

        if (dim == this->dim()) {
            this->resize_data(N, dim);
            return;
        }

        StateMatrix<RowMajor, Dim, T> tmp;
        tmp.resize_data(N, dim);
        const size_type K = std::min(N, this->size());
        const std::size_t D = std::min(dim, this->dim());
        if (D > 0)
            for (size_type k = 0; k != K; ++k)
                std::copy_n(row_data(k), D, tmp.row_data(k));
        *this = std::move(tmp);
    }

    void duplicate_dispatch(size_type src, size_type dst, std::true_type)
    {
        std::copy(row_data(src), row_data(src) + this->dim(), row_data(dst));
    }

    void duplicate_dispatch(size_type src, size_type dst, std::false_type)
    {
        duplicate_pos<0>(row_data(src), row_data(dst),
            std::integral_constant<bool, 0 < Dim>());
    }

    template <std::size_t D>
    void duplicate_pos(const value_type *, value_type *, std::false_type)
    {
    }

    template <std::size_t D>
    void duplicate_pos(const value_type *src, value_type *dst, std::true_type)
    {
        dst[D] = src[D];
        duplicate_pos<D + 1>(
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
    using value_type = typename state_matrix_base_type::value_type;
    using pack_type = typename state_matrix_base_type::pack_type;

    explicit StateMatrix(size_type N = 0) : state_matrix_base_type(N) {}

    StateMatrix(size_type N, std::size_t dim) : state_matrix_base_type(N, dim)
    {
    }

    void resize(size_type N) { resize_both(N, this->dim()); }

    void resize(size_type N, std::size_t dim)
    {
        static_assert(Dim == Dynamic, "**StateMatrix::resize** USED WITH AN "
                                      "OBJECT WITH FIXED DIMENSION");
        resize_both(N, dim);
    }

    void resize_dim(std::size_t dim)
    {
        static_assert(Dim == Dynamic, "**StateMatrix::resize_dim** USED WITH "
                                      "AN OBJECT WITH FIXED DIMENSION");
        resize_both(this->size(), dim);
    }

    T &operator()(size_type id, std::size_t pos)
    {
        return this->data()[pos * this->size() + id];
    }

    const T &operator()(size_type id, std::size_t pos) const
    {
        return this->data()[pos * this->size() + id];
    }

    template <typename OutputIter>
    OutputIter read_state(std::size_t pos, OutputIter first) const
    {
        return std::copy_n(col_data(pos), this->size(), first);
    }

    template <typename OutputIter>
    OutputIter read_state_matrix(MatrixLayout layout, OutputIter first) const
    {
        if (layout == RowMajor)
            for (size_type i = 0; i != this->size(); ++i)
                for (std::size_t d = 0; d != this->size(); ++d, ++first)
                    *first = operator()(i, d);

        if (layout == ColMajor)
            first = std::copy_n(this->data(), this->data_size(), first);

        return first;
    }

    value_type *col_data(std::size_t pos)
    {
        return this->data() + pos * this->size();
    }

    const value_type *col_data(std::size_t pos) const
    {
        return this->data() + pos * this->size();
    }

    /// \brief Copy particles
    ///
    /// \param N The new sample size
    /// \param index N-vector of parent index
    ///
    /// \details
    /// Let \f$a_i\f$ denote the value of `index[i]`, and
    /// \f$r_i = \sum_{j=1}^N \mathbb{I}_{a_j = i}\f$. Then it is required that
    /// \f$a_i = i\f$ for all \f$r_i > 0\f$. This condition is always satisfied
    /// if `index` comes from `resamle_trans_rep_index`.
    template <typename InputIter>
    void select(size_type N, InputIter index)
    {
        size_type n = static_cast<size_type>(N);
        if (this->size() == 0 || internal::is_nullptr(index)) {
            this->resize(N);
            return;
        }

        InputIter idx = index;
        if (n == this->size()) {
            for (std::size_t d = 0; d != this->dim(); ++d) {
                idx = index;
                for (size_type dst = 0; dst != n; ++dst, ++idx) {
                    operator()(dst, d) = operator()(
                        static_cast<size_type>(*idx), d);
                }
            }
        } else {
            StateMatrix<ColMajor, Dim, T> tmp;
            tmp.resize_data(N, this->dim());
            for (std::size_t d = 0; d != this->dim(); ++d) {
                idx = index;
                for (size_type dst = 0; dst != n; ++dst, ++idx)
                    tmp(dst, d) = operator()(static_cast<size_type>(*idx), d);
            }
            *this = std::move(tmp);
        }

        return;
    }

    void duplicate(size_type src, size_type dst)
    {
        if (src == dst)
            return;

        duplicate_dispatch(src, dst, std::integral_constant < bool,
            Dim == Dynamic || 8 < Dim > ());
    }

    pack_type state_pack(size_type id) const
    {
        pack_type pack(this->dim());
        for (std::size_t d = 0; d != this->dim(); ++d)
            pack[d] = operator()(id, d);

        return pack;
    }

    void state_unpack(size_type id, const pack_type &pack)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_UNPACK_SIZE(
            pack.size(), this->dim());

        for (std::size_t d = 0; d != this->dim(); ++d)
            operator()(id, d) = pack[d];
    }

    void state_unpack(size_type id, pack_type &&pack)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_UNPACK_SIZE(
            pack.size(), this->dim());

        for (std::size_t d = 0; d != this->dim(); ++d)
            operator()(id, d) = std::move(pack[d]);
    }

    private:
    void resize_both(size_type N, std::size_t dim)
    {
        if (N == this->size() && dim == this->dim())
            return;

        if (N == this->size()) {
            this->resize_data(N, dim);
            return;
        }

        StateMatrix<ColMajor, Dim, T> tmp;
        tmp.resize_data(N, dim);
        const size_type K = std::min(N, this->size());
        const std::size_t D = std::min(dim, this->dim());
        if (K > 0)
            for (std::size_t d = 0; d != D; ++d)
                std::copy_n(col_data(d), K, tmp.col_data(d));
        *this = std::move(tmp);
    }

    void duplicate_dispatch(size_type src, size_type dst, std::true_type)
    {
        for (std::size_t d = 0; d != this->dim(); ++d)
            operator()(dst, d) = operator()(src, d);
    }

    void duplicate_dispatch(size_type src, size_type dst, std::false_type)
    {
        duplicate_pos<0>(this->data() + src, this->data() + dst,
            std::integral_constant<bool, 0 < Dim>());
    }

    template <std::size_t D>
    void duplicate_pos(const value_type *, value_type *, std::false_type)
    {
    }

    template <std::size_t D>
    void duplicate_pos(const value_type *src, value_type *dst, std::true_type)
    {
        dst[D * this->size()] = src[D * this->size()];
        duplicate_pos<D + 1>(
            src, dst, std::integral_constant<bool, D + 1 < Dim>());
    }
}; // class StateMatrix

} // namespace vsmc

#endif // VSMC_CORE_STATE_MATRIX_HPP
