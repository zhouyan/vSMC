#ifndef VSMC_SMP_STATE_MATRIX_HPP
#define VSMC_SMP_STATE_MATRIX_HPP

#include <vsmc/core/single_particle.hpp>
#include <vsmc/smp/iterator.hpp>

#define VSMC_RUNTIME_ASSERT_SMP_STATE_MATRIX_COPY_SIZE_MISMATCH \
    VSMC_RUNTIME_ASSERT((N == static_cast<size_type>(this->size())),         \
            ("**StateMatrix::copy** SIZE MISMATCH"))

#define VSMC_RUNTIME_ASSERT_SMP_STATE_MATRIX_DIM_SIZE(dim) \
    VSMC_RUNTIME_ASSERT((dim >= 1),                                          \
            ("**StateMatrix** DIMENSION IS LESS THAN 1"))

#define VSMC_RUNTIME_ASSERT_SMP_STATE_MATRIX_UNPACK_SIZE(psize, dim, name) \
    VSMC_RUNTIME_ASSERT((psize >= dim),                                      \
            ("**State"#name"::state_unpack** INPUT PACK SIZE TOO SMALL"))

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
    typedef typename std::vector<T>::iterator iterator;
    typedef typename std::vector<T>::const_iterator const_iterator;
    typedef typename std::vector<T>::reverse_iterator reverse_iterator;
    typedef typename std::vector<T>::const_reverse_iterator
        const_reverse_iterator;

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

        template <std::size_t Pos>
        const state_type &state (Position<Pos>) const
        {return this->state(Pos);}

        template <std::size_t Pos>
        const state_type &state () const
        {return this->state(Pos);}
    };

    void resize_dim (std::size_t dim)
    {
        VSMC_STATIC_ASSERT_DYNAMIC_DIM_RESIZE(Dim);
        VSMC_RUNTIME_ASSERT_SMP_STATE_MATRIX_DIM_SIZE(dim);

        traits::DimTrait<Dim>::resize_dim(dim);
        state_.resize(StateMatrix<Order, Dim, T>::storage_size(size_, dim));
    }

    size_type size () const {return size_;}

    T *data () {return &state_[0];}

    const T *data () const {return &state_[0];}

    iterator begin() {return state_.begin();}

    iterator end () {return state_.begin() + this->dim() * size_;}

    const_iterator begin () const {return state_.begin();}

    const_iterator end () const {return state_.begin() + this->dim() * size_;}

    const_iterator cbegin () const {return state_.begin();}

    const_iterator cend () const {return state_.begin() + this->dim() * size_;}

    reverse_iterator rbegin () {return reverse_iterator(end());}

    reverse_iterator rend () {return reverse_iterator(begin());}

    const_reverse_iterator rbegin () const
    {return const_reverse_iterator(end());}

    const_reverse_iterator rend () const
    {return const_reverse_iterator(begin());}

    const_reverse_iterator crbegin () const
    {return const_reverse_iterator(cend());}

    const_reverse_iterator crend () const
    {return const_reverse_iterator(cbegin());}

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
        VSMC_RUNTIME_ASSERT_SMP_STATE_MATRIX_UNPACK_SIZE(
                pack.size(), this->dim(), Matrix);

        StateMatrix<Order, Dim, T> *sptr =
            static_cast<StateMatrix<Order, Dim, T> *>(this);
        for (std::size_t d = 0; d != this->dim(); ++d)
            sptr->state(id, d) = pack[d];
    }

    template <typename IntType>
    void copy (size_type N, const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_SMP_STATE_MATRIX_COPY_SIZE_MISMATCH;

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
            for (std::size_t i = 0; i != this->dim() * size_; ++i, ++first)
                *first = state_[i];
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

    explicit StateMatrixBase (size_type N) :
        size_(N), state_(StateMatrix<Order, Dim, T>::storage_size(size_, Dim))
    {}

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
    typedef typename std::vector<T>::iterator
        row_iterator;
    typedef typename std::vector<T>::const_iterator
        row_const_iterator;
    typedef typename std::vector<T>::reverse_iterator
        row_reverse_iterator;
    typedef typename std::vector<T>::const_reverse_iterator
        row_const_reverse_iterator;
    typedef StepRandomIterator<typename std::vector<T>::iterator>
        col_iterator;
    typedef StepRandomIterator<typename std::vector<T>::const_iterator>
        col_const_iterator;
    typedef std::reverse_iterator<col_iterator>
        col_reverse_iterator;
    typedef std::reverse_iterator<col_const_iterator>
        col_const_reverse_iterator;

    explicit StateMatrix (size_type N) : state_matrix_base_type(N) {}

    static std::size_t storage_size (std::size_t N, std::size_t dim)
    {return (N + 1) * dim;}

    T &state (size_type id, std::size_t pos)
    {return this->state_matrix()[id * this->dim() + pos];}

    const T &state (size_type id, std::size_t pos) const
    {return this->state_matrix()[id * this->dim() + pos];}

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

    row_iterator row_begin (std::size_t i)
    {return this->state_matrix().begin() + i * this->dim();}

    row_iterator row_end (std::size_t i)
    {return row_begin(i) + this->dim();}

    row_const_iterator row_begin (std::size_t i) const
    {return this->state_matrix().begin() + i * this->dim();}

    row_const_iterator row_end (std::size_t i) const
    {return row_begin(i) + this->dim();}

    row_const_iterator row_cbegin (std::size_t i) const
    {return this->state_matrix().begin() + i * this->dim();}

    row_const_iterator row_cend (std::size_t i) const
    {return row_cbegin(i) + this->dim();}

    row_reverse_iterator row_rbegin (std::size_t i)
    {return row_reverse_iterator(row_end(i));}

    row_reverse_iterator row_rend (std::size_t i)
    {return row_reverse_iterator(row_begin(i));}

    row_const_reverse_iterator row_rbegin (std::size_t i) const
    {return row_const_reverse_iterator(row_end(i));}

    row_const_reverse_iterator row_rend (std::size_t i) const
    {return row_const_reverse_iterator(row_begin(i));}

    row_const_reverse_iterator row_crbegin (std::size_t i) const
    {return row_const_reverse_iterator(row_cend(i));}

    row_const_reverse_iterator row_crend (std::size_t i) const
    {return row_const_reverse_iterator(row_cbegin(i));}

    col_iterator col_begin (std::size_t i)
    {return col_iterator(this->state_matrix().begin() + i, this->dim());}

    col_iterator col_end (std::size_t i)
    {return col_begin(i) + this->size();}

    col_const_iterator col_begin (std::size_t i) const
    {
        return col_const_iterator(
                this->state_matrix().begin() + i, this->dim());
    }

    col_const_iterator col_end (std::size_t i) const
    {return col_begin(i) + this->size();}

    col_const_iterator col_cbegin (std::size_t i) const
    {
        return col_const_iterator(
                this->state_matrix().begin() + i, this->dim());
    }

    col_const_iterator col_cend (std::size_t i) const
    {return col_cbegin(i) + this->size();}

    col_reverse_iterator col_rbegin (std::size_t i)
    {return col_reverse_iterator(col_end(i));}

    col_reverse_iterator col_rend (std::size_t i)
    {return col_reverse_iterator(col_begin(i));}

    col_const_reverse_iterator col_rbegin (std::size_t i) const
    {return col_const_reverse_iterator(col_end(i));}

    col_const_reverse_iterator col_rend (std::size_t i) const
    {return col_const_reverse_iterator(col_begin(i));}

    col_const_reverse_iterator col_crbegin (std::size_t i) const
    {return col_const_reverse_iterator(col_cend(i));}

    col_const_reverse_iterator col_crend (std::size_t i) const
    {return col_const_reverse_iterator(col_cbegin(i));}
}; // class StateMatrix

/// \brief Particle::value_type subtype
/// \ingroup SMP
template <std::size_t Dim, typename T>
class StateMatrix<ColMajor, Dim, T> : public StateMatrixBase<ColMajor, Dim, T>
{
    public :

    typedef StateMatrixBase<ColMajor, Dim, T> state_matrix_base_type;
    typedef typename state_matrix_base_type::size_type size_type;
    typedef StepRandomIterator<typename std::vector<T>::iterator>
        row_iterator;
    typedef StepRandomIterator<typename std::vector<T>::const_iterator>
        row_const_iterator;
    typedef std::reverse_iterator<row_iterator>
        row_reverse_iterator;
    typedef std::reverse_iterator<row_const_iterator>
        row_const_reverse_iterator;
    typedef typename std::vector<T>::iterator
        col_iterator;
    typedef typename std::vector<T>::const_iterator
        col_const_iterator;
    typedef typename std::vector<T>::reverse_iterator
        col_reverse_iterator;
    typedef typename std::vector<T>::const_reverse_iterator
        col_const_reverse_iterator;

    explicit StateMatrix (size_type N) : state_matrix_base_type(N) {}

    static std::size_t storage_size (std::size_t N, std::size_t dim)
    {return N * (dim + 1);}

    T &state (size_type id, std::size_t pos)
    {return this->state_matrix()[pos * this->size() + id];}

    const T &state (size_type id, std::size_t pos) const
    {return this->state_matrix()[pos * this->size() + id];}

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

    row_iterator row_begin (std::size_t i)
    {return row_iterator(this->state_matrix().begin() + i, this->size());}

    row_iterator row_end (std::size_t i)
    {return row_begin(i) + this->dim();}

    row_const_iterator row_begin (std::size_t i) const
    {
        return row_const_iterator(
                this->state_matrix().begin() + i, this->size());
    }

    row_const_iterator row_end (std::size_t i) const
    {return row_begin(i) + this->dim();}

    row_const_iterator row_cbegin (std::size_t i) const
    {
        return row_const_iterator(
                this->state_matrix().begin() + i, this->size());
    }

    row_const_iterator row_cend (std::size_t i) const
    {return row_cbegin(i) + this->dim();}

    row_reverse_iterator row_rbegin (std::size_t i)
    {return row_reverse_iterator(row_end(i));}

    row_reverse_iterator row_rend (std::size_t i)
    {return row_reverse_iterator(row_begin(i));}

    row_const_reverse_iterator row_rbegin (std::size_t i) const
    {return row_const_reverse_iterator(row_end(i));}

    row_const_reverse_iterator row_rend (std::size_t i) const
    {return row_const_reverse_iterator(row_begin(i));}

    row_const_reverse_iterator row_crbegin (std::size_t i) const
    {return row_const_reverse_iterator(row_cend(i));}

    row_const_reverse_iterator row_crend (std::size_t i) const
    {return row_const_reverse_iterator(row_cbegin(i));}

    col_iterator col_begin (std::size_t i)
    {return this->state_matrix().begin() + i * this->size();}

    col_iterator col_end (std::size_t i)
    {return col_begin(i) + this->size();}

    col_const_iterator col_begin (std::size_t i) const
    {return this->state_matrix().begin() + i * this->size();}

    col_const_iterator col_end (std::size_t i) const
    {return col_begin(i) + this->size();}

    col_const_iterator col_cbegin (std::size_t i) const
    {return this->state_matrix().begin() + i * this->size();}

    col_const_iterator col_cend (std::size_t i) const
    {return col_cbegin(i) + this->size();}

    col_reverse_iterator col_rbegin (std::size_t i)
    {return col_reverse_iterator(col_end(i));}

    col_reverse_iterator col_rend (std::size_t i)
    {return col_reverse_iterator(col_begin(i));}

    col_const_reverse_iterator col_rbegin (std::size_t i) const
    {return col_const_reverse_iterator(col_end(i));}

    col_const_reverse_iterator col_rend (std::size_t i) const
    {return col_const_reverse_iterator(col_begin(i));}

    col_const_reverse_iterator col_crbegin (std::size_t i) const
    {return col_const_reverse_iterator(col_cend(i));}

    col_const_reverse_iterator col_crend (std::size_t i) const
    {return col_const_reverse_iterator(col_cbegin(i));}
}; // class StateMatrix

} // namespace vsmc

#endif // VSMC_SMP_STATE_MATRIX_HPP
