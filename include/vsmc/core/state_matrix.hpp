//============================================================================
// vSMC/include/vsmc/core/state_matrix.hpp
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

#ifndef VSMC_CORE_STATE_MATRIX_HPP
#define VSMC_CORE_STATE_MATRIX_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/single_particle.hpp>

#define VSMC_STATIC_ASSERT_CORE_STATE_MATRIX_DYNAMIC_DIM_RESIZE(Dim)          \
    VSMC_STATIC_ASSERT((Dim == Dynamic),                                      \
        "**StateMatrix::resize_dim** USED WITH A FIXED DIMENSION OBJECT")

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
template <MatrixOrder Order, std::size_t Dim, typename T>
class StateMatrixBase : public internal::StateMatrixDim<Dim>
{
    public:
    typedef std::size_t size_type;
    typedef T state_type;
    typedef typename std::conditional<Dim == Dynamic, std::vector<T>,
        std::array<T, Dim>>::type state_pack_type;

    template <typename S>
    struct single_particle_type : public SingleParticleBase<S> {
        single_particle_type(
            typename Particle<S>::size_type id, Particle<S> *particle_ptr)
            : SingleParticleBase<S>(id, particle_ptr)
        {
        }

        std::size_t dim() const { return this->particle_ptr()->value().dim(); }

        state_type &state(std::size_t pos) const
        {
            return this->mutable_particle_ptr()->value().state(
                this->id(), pos);
        }
    }; // struct single_particle_type

    template <typename S>
    struct const_single_particle_type : public ConstSingleParticleBase<S> {
        const_single_particle_type(typename Particle<S>::size_type id,
            const Particle<S> *particle_ptr)
            : ConstSingleParticleBase<S>(id, particle_ptr)
        {
        }

        std::size_t dim() const { return this->particle_ptr()->value().dim(); }

        const state_type &state(std::size_t pos) const
        {
            return this->particle_ptr()->value().state(this->id(), pos);
        }
    }; // struct const_single_particle_type

    void resize_dim(std::size_t dim)
    {
        VSMC_STATIC_ASSERT_CORE_STATE_MATRIX_DYNAMIC_DIM_RESIZE(Dim);
        VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_DIM_SIZE(dim);

        internal::StateMatrixDim<Dim>::resize_dim(dim);
        data_.resize(size_ * dim);
    }

    size_type size() const { return size_; }

    state_type &operator()(std::size_t i, std::size_t pos)
    {
        return static_cast<StateMatrix<Order, Dim, T> *>(this)->state(i, pos);
    }

    const state_type &operator()(std::size_t i, std::size_t pos) const
    {
        return static_cast<const StateMatrix<Order, Dim, T> *>(this)->state(
            i, pos);
    }

    T *data() { return data_.data(); }

    const T *data() const { return data_.data(); }

    void swap(StateMatrixBase<Order, Dim, T> &other)
    {
        internal::StateMatrixDim<Dim>::swap(other);
        std::swap(size_, other.size_);
        data_.swap(other.data_);
    }

    template <typename OutputIter>
    void read_state(std::size_t pos, OutputIter first) const
    {
        const StateMatrix<Order, Dim, T> *sptr =
            static_cast<const StateMatrix<Order, Dim, T> *>(this);
        for (size_type i = 0; i != size_; ++i, ++first)
            *first = sptr->state(i, pos);
    }

    template <typename OutputIterIter>
    void read_state_matrix(OutputIterIter first) const
    {
        for (std::size_t d = 0; d != this->dim(); ++d, ++first)
            read_state(d, *first);
    }

    template <MatrixOrder ROrder, typename OutputIter>
    void read_state_matrix(OutputIter first) const
    {
        if (ROrder == Order) {
            std::copy(data_.begin(), data_.end(), first);
        } else {
            const StateMatrix<Order, Dim, T> *sptr =
                static_cast<const StateMatrix<Order, Dim, T> *>(this);
            if (ROrder == RowMajor) {
                for (size_type i = 0; i != size_; ++i) {
                    for (std::size_t d = 0; d != this->dim(); ++d) {
                        *first = sptr->state(i, d);
                        ++first;
                    }
                }
            } else if (ROrder == ColMajor) {
                for (std::size_t d = 0; d != this->dim(); ++d) {
                    for (size_type i = 0; i != size_; ++i) {
                        *first = sptr->state(i, d);
                        ++first;
                    }
                }
            }
        }
    }

    template <typename CharT, typename Traits>
    std::basic_ostream<CharT, Traits> &print(
        std::basic_ostream<CharT, Traits> &os, char sepchar = '\t') const
    {
        if (this->dim() == 0 || size_ == 0 || !os.good())
            return os;

        const StateMatrix<Order, Dim, T> *sptr =
            static_cast<const StateMatrix<Order, Dim, T> *>(this);
        for (size_type i = 0; i != size_; ++i) {
            for (std::size_t d = 0; d != this->dim() - 1; ++d)
                os << sptr->state(i, d) << sepchar;
            os << sptr->state(i, this->dim() - 1) << '\n';
        }

        return os;
    }

    protected:
    explicit StateMatrixBase(size_type N) : size_(N), data_(N * Dim) {}

    state_pack_type create_pack() const
    {
        return create_pack_dispatch(
            std::integral_constant<bool, Dim == Dynamic>());
    }

    private:
    size_type size_;
    Vector<T> data_;

    std::vector<T> create_pack_dispatch(std::true_type) const
    {
        return std::vector<T>(this->dim());
    }

    std::array<T, Dim> create_pack_dispatch(std::false_type) const
    {
        return std::array<T, Dim>();
    }
}; // class StateMatrixBase

template <typename CharT, typename Traits, MatrixOrder Order, std::size_t Dim,
    typename T>
inline std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os,
    const StateMatrixBase<Order, Dim, T> &smatrix)
{
    return smatrix.print(os);
}

/// \brief Particle::value_type subtype
/// \ingroup Core
template <std::size_t Dim, typename T>
class StateMatrix<RowMajor, Dim, T> : public StateMatrixBase<RowMajor, Dim, T>
{
    public:
    typedef StateMatrixBase<RowMajor, Dim, T> state_matrix_base_type;
    typedef typename state_matrix_base_type::size_type size_type;
    typedef typename state_matrix_base_type::state_pack_type state_pack_type;

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

    T *data(size_type id) { return row_data(id); }

    const T *data(size_type id) const { return row_data(id); }

    T *row_data(size_type id) { return this->data() + id * this->dim(); }

    const T *row_data(size_type id) const
    {
        return this->data() + id * this->dim();
    }

    template <typename IntType>
    void copy(size_type N, const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_COPY_SIZE_MISMATCH;

        for (size_type to = 0; to != N; ++to)
            copy_particle(copy_from[to], to);
    }

    void copy_particle(size_type from, size_type to)
    {
        if (from == to)
            return;

        copy_particle_dispatch(from, to, std::integral_constant < bool,
            Dim == Dynamic || 8 < Dim > ());
    }

    state_pack_type state_pack(size_type id) const
    {
        state_pack_type pack(this->create_pack());
        std::copy(row_data(id), row_data(id) + this->dim(), pack.data());

        return pack;
    }

    void state_unpack(size_type id, const state_pack_type &pack)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_UNPACK_SIZE(
            pack.size(), this->dim());

        const T *ptr = pack.data();
        std::copy(ptr, ptr + this->dim(), row_data(id));
    }

    void state_unpack(size_type id, state_pack_type &&pack)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_UNPACK_SIZE(
            pack.size(), this->dim());

        const T *ptr = pack.data();
        std::move(ptr, ptr + this->dim(), row_data(id));
    }

    private:
    void copy_particle_dispatch(size_type from, size_type to, std::true_type)
    {
        std::copy(row_data(from), row_data(from) + this->dim(), row_data(to));
    }

    void copy_particle_dispatch(size_type from, size_type to, std::false_type)
    {
        copy_particle_pos<0>(row_data(from), row_data(to),
            std::integral_constant<bool, 0 < Dim>());
    }

    template <std::size_t D>
    void copy_particle_pos(const T *, T *, std::false_type)
    {
    }

    template <std::size_t D>
    void copy_particle_pos(const T *from, T *to, std::true_type)
    {
        to[D] = from[D];
        copy_particle_pos<D + 1>(
            from, to, std::integral_constant<bool, D + 1 < Dim>());
    }
}; // class StateMatrix

/// \brief Particle::value_type subtype
/// \ingroup Core
template <std::size_t Dim, typename T>
class StateMatrix<ColMajor, Dim, T> : public StateMatrixBase<ColMajor, Dim, T>
{
    public:
    typedef StateMatrixBase<ColMajor, Dim, T> state_matrix_base_type;
    typedef typename state_matrix_base_type::size_type size_type;
    typedef typename state_matrix_base_type::state_pack_type state_pack_type;

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

    T *data(size_type pos) { return col_data(pos); }

    const T *data(size_type pos) const { return col_data(pos); }

    T *col_data(std::size_t pos) { return this->data() + pos * this->size(); }

    const T *col_data(std::size_t pos) const
    {
        return this->data() + pos * this->size();
    }

    template <typename IntType>
    void copy(size_type N, const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_COPY_SIZE_MISMATCH;

        for (size_type to = 0; to != N; ++to)
            copy_particle(static_cast<size_type>(copy_from[to]), to);
    }

    void copy_particle(size_type from, size_type to)
    {
        if (from == to)
            return;

        copy_particle_dispatch(from, to, std::integral_constant < bool,
            Dim == Dynamic || 8 < Dim > ());
    }

    state_pack_type state_pack(size_type id) const
    {
        state_pack_type pack(this->create_pack());
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
    void copy_particle_dispatch(size_type from, size_type to, std::true_type)
    {
        for (std::size_t d = 0; d != this->dim(); ++d)
            state(to, d) = state(from, d);
    }

    void copy_particle_dispatch(size_type from, size_type to, std::false_type)
    {
        copy_particle_pos<0>(this->data() + from, this->data() + to,
            std::integral_constant<bool, 0 < Dim>());
    }

    template <std::size_t D>
    void copy_particle_pos(const T *, T *, std::false_type)
    {
    }

    template <std::size_t D>
    void copy_particle_pos(const T *from, T *to, std::true_type)
    {
        to[D * this->size()] = from[D * this->size()];
        copy_particle_pos<D + 1>(
            from, to, std::integral_constant<bool, D + 1 < Dim>());
    }
}; // class StateMatrix

} // namespace vsmc

#endif // VSMC_CORE_STATE_MATRIX_HPP
