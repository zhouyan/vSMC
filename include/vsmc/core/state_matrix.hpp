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

#define VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_COPY_SIZE_MISMATCH              \
    VSMC_RUNTIME_ASSERT((N == static_cast<size_type>(this->size())),          \
        "**StateMatrix::copy** SIZE MISMATCH")

#define VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_DIM_SIZE(dim)                   \
    VSMC_RUNTIME_ASSERT((dim >= 1), "**StateMatrix** DIMENSION IS LESS THAN " \
                                    "1")

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
    void swap(StateMatrixDim<Dim> &) {}
}; // class StateMatrixDim

template <>
class StateMatrixDim<Dynamic>
{
    public:
    StateMatrixDim() : dim_(1) {}

    std::size_t dim() const { return dim_; }

    protected:
    void swap(StateMatrixDim<Dynamic> &other) { std::swap(dim_, other.dim_); }

    void resize_dim(std::size_t dim) { dim_ = dim; }

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
            return this->particle().value().state(this->id(), pos);
        }
    }; // class single_particle_type

    void resize_dim(std::size_t dim)
    {
        static_assert(Dim == Dynamic,
            "**StateMatrix** OBJECT DECLARED WITH A FIXED DIMENSION");
        VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_DIM_SIZE(dim);

        internal::StateMatrixDim<Dim>::resize_dim(dim);
        data_.resize(size_ * dim);
    }

    size_type size() const { return size_; }

    state_type *data() { return data_.data(); }

    const state_type *data() const { return data_.data(); }

    void swap(StateMatrixBase<Layout, Dim, T> &other)
    {
        internal::StateMatrixDim<Dim>::swap(other);
        std::swap(size_, other.size_);
        data_.swap(other.data_);
    }

    template <typename OutputIter>
    void read_state(std::size_t pos, OutputIter first) const
    {
        const StateMatrix<Layout, Dim, T> *sptr =
            static_cast<const StateMatrix<Layout, Dim, T> *>(this);
        for (size_type i = 0; i != size_; ++i, ++first)
            *first = sptr->state(i, pos);
    }

    template <typename OutputIterIter>
    void read_state_list(OutputIterIter first) const
    {
        for (std::size_t d = 0; d != this->dim(); ++d, ++first)
            read_state(d, *first);
    }

    template <typename OutputIter>
    void read_state_matrix(MatrixLayout layout, OutputIter first) const
    {
        if (layout == Layout) {
            std::copy(data_.begin(), data_.end(), first);
        } else {
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
        }
    }

    template <typename CharT, typename Traits>
    std::basic_ostream<CharT, Traits> &print(
        std::basic_ostream<CharT, Traits> &os, char sepchar = '\t') const
    {
        if (this->dim() == 0 || size_ == 0 || !os.good())
            return os;

        const StateMatrix<Layout, Dim, T> *sptr =
            static_cast<const StateMatrix<Layout, Dim, T> *>(this);
        for (size_type i = 0; i != size_; ++i) {
            for (std::size_t d = 0; d != this->dim() - 1; ++d)
                os << sptr->state(i, d) << sepchar;
            os << sptr->state(i, this->dim() - 1) << '\n';
        }

        return os;
    }

    protected:
    explicit StateMatrixBase(size_type N) : size_(N), data_(N * Dim) {}

    private:
    size_type size_;
    Vector<T> data_;
}; // class StateMatrixBase

template <typename CharT, typename Traits, MatrixLayout Layout,
    std::size_t Dim, typename T>
inline std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os,
    const StateMatrixBase<Layout, Dim, T> &smatrix)
{
    return smatrix.print(os);
}

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

    explicit StateMatrix(size_type N) : state_matrix_base_type(N) {}

    T &state(size_type id, std::size_t pos)
    {
        return this->data()[id * this->dim() + pos];
    }

    const T &state(size_type id, std::size_t pos) const
    {
        return this->data()[id * this->dim() + pos];
    }

    using state_matrix_base_type::data;

    state_type *data(size_type id) { return row_data(id); }

    const state_type *data(size_type id) const { return row_data(id); }

    state_type *row_data(size_type id)
    {
        return this->data() + id * this->dim();
    }

    const state_type *row_data(size_type id) const
    {
        return this->data() + id * this->dim();
    }

    template <typename IntType>
    void copy(size_type N, const IntType *index)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_COPY_SIZE_MISMATCH;

        for (size_type dst = 0; dst != N; ++dst)
            copy_particle(static_cast<size_type>(index[dst]), dst);
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

    explicit StateMatrix(size_type N) : state_matrix_base_type(N) {}

    T &state(size_type id, std::size_t pos)
    {
        return this->data()[pos * this->size() + id];
    }

    const T &state(size_type id, std::size_t pos) const
    {
        return this->data()[pos * this->size() + id];
    }

    using state_matrix_base_type::data;

    state_type *data(size_type pos) { return col_data(pos); }

    const state_type *data(size_type pos) const { return col_data(pos); }

    state_type *col_data(std::size_t pos)
    {
        return this->data() + pos * this->size();
    }

    const state_type *col_data(std::size_t pos) const
    {
        return this->data() + pos * this->size();
    }

    template <typename IntType>
    void copy(size_type N, const IntType *index)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_COPY_SIZE_MISMATCH;

        for (std::size_t d = 0; d != this->dim(); ++d)
            for (size_type dst = 0; dst != N; ++dst)
                state(dst, d) = state(static_cast<size_type>(index[dst]), d);
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

} // namespace vsmc

#endif // VSMC_CORE_STATE_MATRIX_HPP
