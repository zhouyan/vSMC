//============================================================================
// vSMC/include/vsmc/core/state_matrix.hpp
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

#ifndef VSMC_CORE_STATE_MATRIX_HPP
#define VSMC_CORE_STATE_MATRIX_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/single_particle.hpp>
#include <vsmc/utility/aligned_memory.hpp>

#define VSMC_STATIC_ASSERT_CORE_STATE_MATRIX_DYNAMIC_DIM_RESIZE(Dim) \
    VSMC_STATIC_ASSERT((Dim == Dynamic),                                     \
            USE_METHOD_resize_dim_WITH_A_FIXED_SIZE_StateMatrix_OBJECT)

#define VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_COPY_SIZE_MISMATCH \
    VSMC_RUNTIME_ASSERT((N == static_cast<size_type>(this->size())),         \
            ("**StateMatrix::copy** SIZE MISMATCH"))

#define VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_DIM_SIZE(dim) \
    VSMC_RUNTIME_ASSERT((dim >= 1),                                          \
            ("**StateMatrix** DIMENSION IS LESS THAN 1"))

#define VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_UNPACK_SIZE(psize, dim) \
    VSMC_RUNTIME_ASSERT((psize >= dim),                                      \
            ("**StateMatrix::state_unpack** INPUT PACK SIZE TOO SMALL"))

namespace vsmc {

/// \brief Base type of StateTuple
/// \ingroup Core
template <MatrixOrder Order, std::size_t Dim, typename T>
class StateMatrixBase : public traits::DimTrait<Dim>
{
    public :

    typedef std::size_t size_type;
    typedef std::vector<T> state_pack_type;
    typedef T state_type;

    template <typename S>
    struct single_particle_type : public SingleParticleBase<S>
    {
        single_particle_type (typename Particle<S>::size_type id,
                Particle<S> *particle_ptr) :
            SingleParticleBase<S>(id, particle_ptr) {}

        std::size_t dim () const {return this->particle_ptr()->value().dim();}

        state_type &state (std::size_t pos) const
        {return this->mutable_particle_ptr()->value().state(this->id(), pos);}

        template <std::size_t Pos>
        state_type &state (Position<Pos>) const
        {return this->state(Pos);}

        template <std::size_t Pos>
        state_type &state () const
        {return this->state(Pos);}
    }; // struct single_particle_type

    template <typename S>
    struct const_single_particle_type : public ConstSingleParticleBase<S>
    {
        const_single_particle_type (typename Particle<S>::size_type id,
                const Particle<S> *particle_ptr) :
            ConstSingleParticleBase<S>(id, particle_ptr) {}

        std::size_t dim () const {return this->particle_ptr()->value().dim();}

        const state_type &state (std::size_t pos) const
        {return this->particle_ptr()->value().state(this->id(), pos);}

        template <std::size_t Pos>
        const state_type &state (Position<Pos>) const
        {return this->state(Pos);}

        template <std::size_t Pos>
        const state_type &state () const
        {return this->state(Pos);}
    }; // struct const_single_particle_type

    void resize_dim (std::size_t dim)
    {
        VSMC_STATIC_ASSERT_CORE_STATE_MATRIX_DYNAMIC_DIM_RESIZE(Dim);
        VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_DIM_SIZE(dim);

        traits::DimTrait<Dim>::resize_dim(dim);
        data_.resize(size_ * dim);
    }

    size_type size () const {return size_;}

    state_type &operator() (std::size_t i, std::size_t pos)
    {
        return static_cast<StateMatrix<Order, Dim, T> *>(this)->
            state(i, pos);
    }

    const state_type &operator() (std::size_t i, std::size_t pos) const
    {
        return static_cast<const StateMatrix<Order, Dim, T> *>(this)->
            state(i, pos);
    }

    T *data () {return &data_[0];}

    const T *data () const {return &data_[0];}

    template <typename OutputIter>
    OutputIter read_state (std::size_t pos, OutputIter first) const
    {
        const StateMatrix<Order, Dim, T> *sptr =
            static_cast<const StateMatrix<Order, Dim, T> *>(this);
        for (size_type i = 0; i != size_; ++i, ++first)
                *first = sptr->state(i, pos);

        return first;
    }

    template <std::size_t Pos, typename OutputIter>
    OutputIter read_state (Position<Pos>, OutputIter first) const
    {return read_state(Pos, first);}

    template <std::size_t Pos, typename OutputIter>
    OutputIter read_state (OutputIter first) const
    {return read_state(Pos, first);}

    template <typename OutputIterIter>
    void read_state_matrix (OutputIterIter first) const
    {
        for (std::size_t d = 0; d != this->dim(); ++d, ++first)
            read_state(d, *first);
    }

    template <MatrixOrder ROrder, typename OutputIter>
    OutputIter read_state_matrix (OutputIter first) const
    {
        if (ROrder == Order) {
            first = std::copy(data_.begin(), data_.end(), first);
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

        return first;
    }

    template <typename CharT, typename Traits>
    std::basic_ostream<CharT, Traits> &print (
            std::basic_ostream<CharT, Traits> &os, std::size_t iter = 0,
            char sepchar = ' ', char eolchar = '\n') const
    {
        const StateMatrix<Order, Dim, T> *sptr =
            static_cast<const StateMatrix<Order, Dim, T> *>(this);
        for (size_type i = 0; i != size_; ++i) {
            os << iter << sepchar;
            for (std::size_t d = 0; d != this->dim() - 1; ++d)
                os << sptr->state(i, d) << sepchar;
            os << sptr->state(i, this->dim() - 1) << eolchar;
        }

        return os;
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &operator<< (
            std::basic_ostream<CharT, Traits> &os,
            const StateMatrixBase<Order, Dim, T> &smatrix)
    {
        if (os.good())
            smatrix.print(os);

        return os;
    }

    protected :

    explicit StateMatrixBase (size_type N) : size_(N), data_(N * Dim) {}

    private :

    size_type size_;
    typename cxx11::conditional<cxx11::is_arithmetic<T>::value,
             std::vector<T, AlignedAllocator<T> >,
             std::vector<T> >::type data_;
}; // class StateMatrixBase

/// \brief Particle::value_type subtype
/// \ingroup Core
template <std::size_t Dim, typename T>
class StateMatrix<RowMajor, Dim, T> : public StateMatrixBase<RowMajor, Dim, T>
{
    public :

    typedef StateMatrixBase<RowMajor, Dim, T> state_matrix_base_type;
    typedef typename state_matrix_base_type::size_type size_type;
    typedef typename state_matrix_base_type::state_pack_type state_pack_type;

    explicit StateMatrix (size_type N) : state_matrix_base_type(N) {}

    T &state (size_type id, std::size_t pos)
    {return this->data()[id * this->dim() + pos];}

    const T &state (size_type id, std::size_t pos) const
    {return this->data()[id * this->dim() + pos];}

    template <std::size_t Pos>
    T &state (size_type id, Position<Pos>)
    {return state(id, Pos);}

    template <std::size_t Pos>
    const T &state (size_type id, Position<Pos>) const
    {return state(id, Pos);}

    template <std::size_t Pos>
    T &state (size_type id)
    {return state(id, Pos);}

    template <std::size_t Pos>
    const T &state (size_type id) const
    {return state(id, Pos);}

    T *row_data (size_type id)
    {return this->data() + id * this->dim();}

    const T *row_data (size_type id) const
    {return this->data() + id * this->dim();}

    template <typename IntType>
    void copy (size_type N, const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_COPY_SIZE_MISMATCH;

        for (size_type to = 0; to != N; ++to)
            copy_particle(copy_from[to], to);
    }

    state_pack_type state_pack (size_type id) const
    {return state_pack_type(row_data(id), row_data(id +1));}

    void state_unpack (size_type id, const state_pack_type &pack)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_UNPACK_SIZE(
                pack.size(), this->dim());

        const T *ptr = &pack[0];
        std::copy(ptr, ptr + this->dim(), row_data(id));
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES && VSMC_HAS_CXX11LIB_ALGORITHM
    void state_unpack (size_type id, state_pack_type &&pack)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_UNPACK_SIZE(
                pack.size(), this->dim());

        const T *ptr = &pack[0];
        std::move(ptr, ptr + this->dim(), row_data(id));
    }
#endif

    protected :

    void copy_particle (size_type from, size_type to)
    {
        if (from == to)
            return;

        std::copy(row_data(from), row_data(from + 1), row_data(to));
    }
}; // class StateMatrix

/// \brief Particle::value_type subtype
/// \ingroup Core
template <std::size_t Dim, typename T>
class StateMatrix<ColMajor, Dim, T> : public StateMatrixBase<ColMajor, Dim, T>
{
    public :

    typedef StateMatrixBase<ColMajor, Dim, T> state_matrix_base_type;
    typedef typename state_matrix_base_type::size_type size_type;
    typedef typename state_matrix_base_type::state_pack_type state_pack_type;

    explicit StateMatrix (size_type N) : state_matrix_base_type(N) {}

    T &state (size_type id, std::size_t pos)
    {return this->data()[pos * this->size() + id];}

    const T &state (size_type id, std::size_t pos) const
    {return this->data()[pos * this->size() + id];}

    template <std::size_t Pos>
    T &state (size_type id, Position<Pos>)
    {return state(id, Pos);}

    template <std::size_t Pos>
    const T &state (size_type id, Position<Pos>) const
    {return state(id, Pos);}

    template <std::size_t Pos>
    T &state (size_type id)
    {return state(id, Pos);}

    template <std::size_t Pos>
    const T &state (size_type id) const
    {return state(id, Pos);}

    T *col_data (std::size_t pos)
    {return this->data() + pos * this->size();}

    const T *col_data (std::size_t pos) const
    {return this->data() + pos * this->size();}

    template <typename IntType>
    void copy (size_type N, const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_COPY_SIZE_MISMATCH;

        for (size_type to = 0; to != N; ++to)
            copy_particle(copy_from[to], to);
    }

    state_pack_type state_pack (size_type id) const
    {
        state_pack_type pack;
        pack.reserve(this->dim());
        for (std::size_t d = 0; d != this->dim(); ++d)
            pack.push_back(state(id, d));

        return pack;
    }

    void state_unpack (size_type id, const state_pack_type &pack)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_UNPACK_SIZE(
                pack.size(), this->dim());

        for (std::size_t d = 0; d != this->dim(); ++d)
            state(id, d) = pack[d];
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    void state_unpack (size_type id, state_pack_type &&pack)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_MATRIX_UNPACK_SIZE(
                pack.size(), this->dim());

        for (std::size_t d = 0; d != this->dim(); ++d)
            state(id, d) = cxx11::move(pack[d]);
    }
#endif

    protected :

    void copy_particle (size_type from, size_type to)
    {
        if (from == to)
            return;

        for (std::size_t d = 0; d != this->dim(); ++d)
            state(to, d) = state(from, d);
    }
}; // class StateMatrix

} // namespace vsmc

#endif // VSMC_CORE_STATE_MATRIX_HPP
