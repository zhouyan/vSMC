#ifndef VSMC_SMP_STATE_MATRIX_HPP
#define VSMC_SMP_STATE_MATRIX_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/single_particle.hpp>

namespace vsmc {

/// \brief Base type of StateTuple
/// \ingroup SMP
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
    };

    template <typename S>
    struct const_single_particle_type : public ConstSingleParticleBase<S>
    {
        const_single_particle_type (typename Particle<S>::size_type id,
                const Particle<S> *particle_ptr) :
            ConstSingleParticleBase<S>(id, particle_ptr) {}

        std::size_t dim () const {return this->particle_ptr()->value().dim();}

        const state_type &state (std::size_t pos) const
        {return this->particle_ptr()->value().state(this->id(), pos);}
    };

    void resize_dim (std::size_t dim)
    {
        VSMC_STATIC_ASSERT_DYNAMIC_DIM_RESIZE;

        traits::DimTrait<Dim>::resize_dim(dim);
        state_.resize(dim * size_);
    }

    size_type size () const {return size_;}

    T *data () {return &state_[0];}

    const T *data () const {return &state_[0];}

    state_pack_type state_pack (size_type id) const
    {
        const StateMatrix<Order, Dim, T> *sptr =
            static_cast<const StateMatrix<Order, Dim, T> *>(this);
        state_pack_type pack(this->dim());
        for (std::size_t d = 0; d != this->dim(); ++d)
            pack[d] = sptr->state(id, d);

        return pack;
    }

    void state_unpack (size_type id, const state_pack_type &pack)
    {
        VSMC_RUNTIME_ASSERT_STATE_UNPACK_SIZE(
                pack.size(), this->dim(), Matrix);

        StateMatrix<Order, Dim, T> *sptr =
            static_cast<StateMatrix<Order, Dim, T> *>(this);
        for (std::size_t d = 0; d != this->dim(); ++d)
            sptr->state(id, d) = pack[d];
    }

    template <typename IntType>
    void copy (size_type N, const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_STATE_COPY_SIZE_MISMATCH(Matrix);

        for (size_type to = 0; to != N; ++to)
            copy_particle(copy_from[to], to);
    }

    template <typename OutputIter>
    OutputIter read_state (std::size_t pos, OutputIter first) const
    {
        const StateMatrix<Order, Dim, T> *sptr =
            static_cast<const StateMatrix<Order, Dim, T> *>(this);
        for (size_type i = 0; i != size_; ++i, ++first)
                *first = sptr->state(i, pos);

        return first;
    }

    template <typename OutputIter>
    void read_state_matrix (OutputIter *first) const
    {
        for (std::size_t d = 0; d != this->dim(); ++d)
            read_state(d, first[d]);
    }

    template <typename OutputIter>
    OutputIter read_state_matrix (MatrixOrder order, OutputIter first) const
    {
        VSMC_RUNTIME_ASSERT_MATRIX_ORDER(
                order, StateMatrix::read_state_matrix);

        if (order == Order) {
            for (std::size_t i = 0; i != state_.size(); ++i, ++first)
                *first = state_[i];
        } else {
            const StateMatrix<Order, Dim, T> *sptr =
                static_cast<const StateMatrix<Order, Dim, T> *>(this);
            for (size_type i = 0; i != size_; ++i) {
                for (std::size_t d = 0; d != this->dim(); ++d) {
                    *first = sptr->state(i, d);
                    ++first;
                }
            }
        }

        return first;
    }

    template <typename OutputStream>
    OutputStream &print (OutputStream &os, std::size_t iter = 0,
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

    protected :

    explicit StateMatrixBase (size_type N) : size_(N), state_(N * Dim) {}

    void copy_particle (size_type from, size_type to)
    {
        StateMatrix<Order, Dim, T> *sptr =
            static_cast<StateMatrix<Order, Dim, T> *>(this);
        if (from != to)
            for (std::size_t d = 0; d != this->dim(); ++d)
                sptr->state(to, d) = sptr->state(from, d);
    }

    std::vector<T> &state_matrix () {return state_;}

    const std::vector<T> &state_matrix () const {return state_;}

    private :

    size_type size_;
    std::vector<T> state_;
}; // class StateMatrixBase

/// \brief Particle::value_type subtype
/// \ingroup SMP
template <std::size_t Dim, typename T>
class StateMatrix<RowMajor, Dim, T> : public StateMatrixBase<RowMajor, Dim, T>
{
    public :

    typedef StateMatrixBase<RowMajor, Dim, T> state_matrix_base_type;
    typedef typename state_matrix_base_type::size_type size_type;

    explicit StateMatrix (size_type N) : state_matrix_base_type(N) {}

    T &state (size_type id, std::size_t pos)
    {return this->state_matrix()[id * this->dim() + pos];}

    const T &state (size_type id, std::size_t pos) const
    {return this->state_matrix()[id * this->dim() + pos];}

    T *row_ptr (size_type id)
    {return this->data() + id * this->dim();}

    const T *row_ptr (size_type id) const
    {return this->data() + id * this->dim();}
}; // class StateMatrix

/// \brief Particle::value_type subtype
/// \ingroup SMP
template <std::size_t Dim, typename T>
class StateMatrix<ColMajor, Dim, T> : public StateMatrixBase<ColMajor, Dim, T>
{
    public :

    typedef StateMatrixBase<ColMajor, Dim, T> state_matrix_base_type;
    typedef typename state_matrix_base_type::size_type size_type;

    explicit StateMatrix (size_type N) : state_matrix_base_type(N) {}

    T &state (size_type id, std::size_t pos)
    {return this->state_matrix()[pos * this->size() + id];}

    const T &state (size_type id, std::size_t pos) const
    {return this->state_matrix()[pos * this->size() + id];}

    T *col_ptr (std::size_t pos)
    {return this->data() + pos * this->size();}

    const T *col_ptr (std::size_t pos) const
    {return this->data() + pos * this->size();}
}; // class StateMatrix

} // namespace vsmc

#endif // VSMC_SMP_STATE_MATRIX_HPP
