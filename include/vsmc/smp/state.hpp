#ifndef VSMC_SMP_STATE_HPP
#define VSMC_SMP_STATE_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/single_particle.hpp>

namespace vsmc {

/// \brief Particle::value_type subtype
/// \ingroup SMP
template <std::size_t Dim, typename T, MatrixOrder Order>
class StateMatrixBase : public traits::DimTrait<Dim>
{
    public :

    typedef std::size_t size_type;
    typedef T state_type;

    explicit StateMatrixBase (size_type N) : size_(N), state_(N * Dim) {}

    template <typename S>
    struct single_particle_type : public SingleParticleBase<S>
    {
        single_particle_type (
                typename SingleParticleBase<S>::size_type id,
                typename SingleParticleBase<S>::particle_ptr_type ptr) :
            SingleParticleBase<S>(id, ptr) {}

        std::size_t dim () const
        {return this->particle_ptr()->value().dim();}

        state_type &state (std::size_t pos) const
        {return this->mutable_particle_ptr()->value().state(this->id(), pos);}
    };

    template <typename S>
    struct const_single_particle_type : public ConstSingleParticleBase<S>
    {
        const_single_particle_type (
                typename ConstSingleParticleBase<S>::size_type id,
                typename ConstSingleParticleBase<S>::particle_ptr_type ptr) :
            ConstSingleParticleBase<S>(id, ptr) {}

        std::size_t dim () const
        {return this->particle_ptr()->value().dim();}

        const state_type &state (std::size_t pos) const
        {return this->particle_ptr()->value().state(this->id(), pos);}
    };

    template <typename IntType>
    void copy (size_type N, const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_STATE_COPY_SIZE_MISMATCH(Base);

        for (size_type to = 0; to != N; ++to)
            copy_particle(copy_from[to], to);
    }

    void resize_dim (std::size_t dim)
    {
        VSMC_STATIC_ASSERT_DYNAMIC_DIM_RESIZE;

        traits::DimTrait<Dim>::resize_dim(dim);
        state_.resize(dim * size_);
    }

    size_type size () const
    {
        return size_;
    }

    template <typename OutputIter>
    OutputIter read_state (std::size_t pos, OutputIter first) const
    {
        const StateMatrix<Dim, T, Order> *sptr =
            static_cast<const StateMatrix<Dim, T, Order> *>(this);
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

        const StateMatrix<Dim, T, Order> *sptr =
            static_cast<const StateMatrix<Dim, T, Order> *>(this);
        if (order == Order) {
            for (std::size_t i = 0; i != state_.size(); ++i, ++first)
                *first = state_[i];
        } else {
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
        const StateMatrix<Dim, T, Order> *sptr =
            static_cast<const StateMatrix<Dim, T, Order> *>(this);
        for (size_type i = 0; i != size_; ++i) {
            os << iter << sepchar;
            for (std::size_t d = 0; d != this->dim() - 1; ++d)
                os << sptr->state(i, d) << sepchar;
            os << sptr->state(i, this->dim() - 1) << eolchar;
        }

        return os;
    }

    protected :

    void copy_particle (size_type from, size_type to)
    {
        StateMatrix<Dim, T, Order> *sptr =
            static_cast<StateMatrix<Dim, T, Order> *>(this);
        if (from != to)
            for (std::size_t d = 0; d != this->dim(); ++d)
                sptr->state(to, d) = sptr->state(from, d);
    }

    std::vector<T> &state_matrix ()
    {
        return state_;
    }

    const std::vector<T> &state_matrix () const
    {
        return state_;
    }

    private :

    size_type size_;
    std::vector<T> state_;
}; // class StateMatrixBase

/// \brief Particle::value_type subtype
/// \ingroup SMP
template <std::size_t Dim, typename T>
class StateMatrix<Dim, T, RowMajor> : public StateMatrixBase<Dim, T, RowMajor>
{
    public :

    typedef typename StateMatrixBase<Dim, T, RowMajor>::size_type size_type;
    typedef T state_type;

    explicit StateMatrix (size_type N) :
        StateMatrixBase<Dim, T, RowMajor>(N) {}

    state_type &state (size_type id, std::size_t pos)
    {
        return this->state_matrix()[id * this->dim() + pos];
    }

    const state_type &state (size_type id, std::size_t pos) const
    {
        return this->state_matrix()[id * this->dim() + pos];
    }
}; // class StateMatrix

/// \brief Particle::value_type subtype
/// \ingroup SMP
template <std::size_t Dim, typename T>
class StateMatrix<Dim, T, ColMajor> : public StateMatrixBase<Dim, T, ColMajor>
{
    public :

    typedef typename StateMatrixBase<Dim, T, ColMajor>::size_type size_type;
    typedef T state_type;

    explicit StateMatrix (size_type N) :
        StateMatrixBase<Dim, T, ColMajor>(N) {}

    state_type &state (size_type id, std::size_t pos)
    {
        return this->state_matrix()[pos * this->size() + id];
    }

    const state_type &state (size_type id, std::size_t pos) const
    {
        return this->state_matrix()[pos * this->size() + id];
    }
}; // class StateMatrix

} // namespace vsmc

#endif
