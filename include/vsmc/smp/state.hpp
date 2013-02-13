#ifndef VSMC_SMP_STATE_HPP
#define VSMC_SMP_STATE_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/single_particle.hpp>

#if VSMC_HAS_CXX11_VARIADIC_TEMPLATES && VSMC_HAS_CXX11LIB_TUPLE
#include <tuple>
#endif

namespace vsmc {

/// \brief Base type of StateTuple
/// \ingroup SMP
template <MatrixOrder Order, std::size_t Dim, typename T>
class StateMatrixBase : public traits::DimTrait<Dim>
{
    public :

    typedef std::size_t size_type;
    typedef T state_type;

    template <typename S>
    struct single_particle_type : public SingleParticleBase<S>
    {
        single_particle_type (
                typename SingleParticleBase<S>::size_type id,
                typename SingleParticleBase<S>::particle_ptr_type ptr) :
            SingleParticleBase<S>(id, ptr) {}

        std::size_t dim () const {return this->particle_ptr()->value().dim();}

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

    template <typename IntType>
    void copy (size_type N, const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_STATE_COPY_SIZE_MISMATCH(Base);

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
}; // class StateMatrix

#if VSMC_HAS_CXX11_VARIADIC_TEMPLATES && VSMC_HAS_CXX11LIB_TUPLE

namespace internal {

template <typename, typename> struct TupleCat;

template <typename FirstType, typename... Types>
struct TupleCat<FirstType, std::tuple<Types...> >
{typedef std::tuple<FirstType, Types...> type;};

template <typename FirstType, typename... Types>
struct StateTupleVectorType
{
    typedef typename TupleCat<std::vector<FirstType>,
            typename StateTupleVectorType<Types...>::type>::type type;
};

template <typename FirstType>
struct StateTupleVectorType<FirstType>
{typedef std::tuple<std::vector<FirstType> > type;};

} // namespace vsmc::internal

/// \brief Base type of StateTuple
/// \ingroup SMP
template <MatrixOrder Order, typename FirstType, typename... Types>
class StateTupleBase
{
    public :

    typedef std::size_t size_type;
    typedef std::tuple<FirstType, Types...> state_tuple_type;

    template <std::size_t Pos> struct state_type
    {typedef typename std::tuple_element<Pos, state_tuple_type>::type type;};

    template <typename S>
    struct single_particle_type : public SingleParticleBase<S>
    {
        single_particle_type (
                typename SingleParticleBase<S>::size_type id,
                typename SingleParticleBase<S>::particle_ptr_type ptr) :
            SingleParticleBase<S>(id, ptr) {}

        static VSMC_CONSTEXPR std::size_t dim () {return S::dim();}

        template <std::size_t Pos>
        typename state_type<Pos>::type &state () const
        {
            return this->mutable_particle_ptr()->value().template
                state<Pos>(this->id());
        }
    };

    template <typename S>
    struct const_single_particle_type : public ConstSingleParticleBase<S>
    {
        const_single_particle_type (
                typename ConstSingleParticleBase<S>::size_type id,
                typename ConstSingleParticleBase<S>::particle_ptr_type ptr) :
            ConstSingleParticleBase<S>(id, ptr) {}

        static VSMC_CONSTEXPR std::size_t dim () {return S::dim();}

        template <std::size_t Pos>
        const typename state_type<Pos>::type &state () const
        {
            return this->particle_ptr()->value().template
                state<Pos>(this->id());
        }
    };

    size_type size () const {return size_;}

    static VSMC_CONSTEXPR std::size_t dim () {return dim_;}

    template <typename IntType>
    void copy (size_type N, const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_STATE_COPY_SIZE_MISMATCH(Base);

        for (size_type to = 0; to != N; ++to)
            copy_particle(copy_from[to], to);
    }

    template <std::size_t Pos, typename OutputIter>
    OutputIter read_state (OutputIter first) const
    {
        const StateTuple<Order, FirstType, Types...> *sptr =
            static_cast<const StateTuple<Order, FirstType, Types...> *>(this);
        for (size_type i = 0; i != size_; ++i, ++first)
                *first = sptr->template state<Pos>(i);

        return first;
    }

    template <typename OutputStream>
    OutputStream &print (OutputStream &os, std::size_t iter = 0,
            char sepchar = ' ', char eolchar = '\n') const
    {
        for (size_type i = 0; i != size_; ++i) {
            os << iter << sepchar;
            print_particle(os, i, sepchar, eolchar, pos_tag_<0>());
        }

        return os;
    }

    protected :

    explicit StateTupleBase (size_type N) : size_(N) {}

    void copy_particle (size_type from, size_type to)
    {
        if (from != to)
            copy_particle(from, to, pos_tag_<0>());
    }

    private :

    size_type size_;
    static const std::size_t dim_ = sizeof...(Types) + 1;

    template <std::size_t Pos> struct pos_tag_ {};

    template <std::size_t Pos>
    void copy_particle (size_type from, size_type to, pos_tag_<Pos>)
    {
        StateTuple<Order, FirstType, Types...> *sptr =
            static_cast<StateTuple<Order, FirstType, Types...> *>(this);
        sptr->template state<Pos>(to) = sptr->template state<Pos>(from);
        copy_particle (from, to, pos_tag_<Pos + 1>());
    }

    void copy_particle (size_type from, size_type to, pos_tag_<dim_>) {}

    template <typename OutputStream, std::size_t Pos>
    void print_particle (OutputStream &os, size_type id,
            char sepchar, char eolchar, pos_tag_<Pos>) const
    {
        const StateTuple<Order, FirstType, Types...> *sptr =
            static_cast<const StateTuple<Order, FirstType, Types...> *>(this);
        os << sptr->template state<Pos>(id) << sepchar;
        print_particle(os, id, sepchar, eolchar, pos_tag_<Pos + 1>());
    }

    template <typename OutputStream>
    void print_particle (OutputStream &os, size_type id,
            char sepchar, char eolchar, pos_tag_<dim_ - 1>) const
    {
        const StateTuple<Order, FirstType, Types...> *sptr =
            static_cast<const StateTuple<Order, FirstType, Types...> *>(this);
        os << sptr->template state<dim_ - 1>(id) << eolchar;
    }
}; // class StateTupleBase

/// \brief Particle::value_type subtype
/// \ingroup SMP
template <typename FirstType, typename... Types>
class StateTuple<RowMajor, FirstType, Types...> :
    public StateTupleBase<RowMajor, FirstType, Types...>
{
    public :

    typedef StateTupleBase<RowMajor, FirstType, Types...>
        state_tuple_base_type;
    typedef typename state_tuple_base_type::size_type size_type;

    explicit StateTuple (size_type N) : state_tuple_base_type(N), state_(N) {}

    template <std::size_t Pos>
    typename state_tuple_base_type::template state_type<Pos>::type
    &state (size_type id) {return std::get<Pos>(state_[id]);}

    template <std::size_t Pos>
    const typename state_tuple_base_type::template state_type<Pos>::type
    &state (size_type id) const {return std::get<Pos>(state_[id]);}

    private :

    std::vector<std::tuple<FirstType, Types...> > state_;
}; // StateTuple

/// \brief Particle::value_type subtype
/// \ingroup SMP
template <typename FirstType, typename... Types>
class StateTuple<ColMajor, FirstType, Types...> :
    public StateTupleBase<ColMajor, FirstType, Types...>
{
    public :

    typedef StateTupleBase<ColMajor, FirstType, Types...>
        state_tuple_base_type;
    typedef typename state_tuple_base_type::size_type size_type;

    explicit StateTuple (size_type N) : state_tuple_base_type(N)
    {init_state(N, pos_tag_<0>());}

    template <std::size_t Pos>
    typename state_tuple_base_type::template state_type<Pos>::type
    &state (size_type id) {return std::get<Pos>(state_)[id];}

    template <std::size_t Pos>
    const typename state_tuple_base_type::template state_type<Pos>::type
    &state (size_type id) const {return std::get<Pos>(state_)[id];}

    private :

    typename internal::StateTupleVectorType<FirstType, Types...>::type
        state_;

    template <std::size_t Pos> struct pos_tag_ {};

    template <std::size_t Pos>
    void init_state (size_type N, pos_tag_<Pos>)
    {
        std::get<Pos>(state_).resize(N);
        init_state(N, pos_tag_<Pos + 1>());
    }

    void init_state (size_type N, pos_tag_<sizeof...(Types)>)
    {
        std::get<sizeof...(Types)>(state_).resize(N);
    }
}; // class StateTuple

#endif // VSMC_HAS_CXX11_VARIADIC_TEMPLATES && VSMC_HAS_CXX11LIB_TUPLE

} // namespace vsmc

#endif
