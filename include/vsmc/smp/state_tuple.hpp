#ifndef VSMC_SMP_STATE_TUPLE_HPP
#define VSMC_SMP_STATE_TUPLE_HPP

#include <vsmc/smp/base.hpp>
#include <vsmc/core/single_particle.hpp>
#include <vsmc/utility/tuple_manip.hpp>

namespace vsmc {

namespace internal {

VSMC_DEFINE_TUPLE_APPLY(Vector, std::vector)

} // namespace vsmc::internal

/// \brief Base type of StateTuple
/// \ingroup SMP
template <MatrixOrder Order, typename T, typename... Types>
class StateTupleBase
{
    public :

    typedef std::size_t size_type;
    typedef std::tuple<T, Types...> state_tuple_type;

    template <std::size_t Pos> struct state_type
    {typedef typename std::tuple_element<Pos, state_tuple_type>::type type;};

    class state_pack_type
    {
        public :

        state_pack_type () {}

        state_pack_type (const state_pack_type &other) : data_(other.data_) {}

        state_pack_type &operator= (const state_pack_type &other)
        {
            if (this != &other)
                data_ = other.data_;

            return *this;
        }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
        state_pack_type (state_pack_type &&other) :
            data_(cxx11::move(other.data_)) {}

        state_pack_type &operator= (state_pack_type &&other)
        {
            if (this != &other)
                data_ = cxx11::move(other.data_);

            return *this;
        }
#endif

        state_pack_type (const state_tuple_type &data) : data_(data) {}

        operator state_tuple_type () {return data_;}

        template <std::size_t Pos>
        typename state_type<Pos>::type &get ()
        {return std::get<Pos>(data_);}

        template <std::size_t Pos>
        const typename state_type<Pos>::type &get () const
        {return std::get<Pos>(data_);}

        template <typename Archive>
        void serialize (Archive &ar, const unsigned)
        {serialize(ar, Position<0>());}

        template <typename Archive>
        void serialize (Archive &ar, const unsigned) const
        {serialize(ar, Position<0>());}

        private :

        state_tuple_type data_;
        static const std::size_t dim_ = sizeof...(Types) + 1;

        template <typename Archive, std::size_t Pos>
        void serialize (Archive &ar, Position<Pos>)
        {
            ar & std::get<Pos>(data_);
            serialize(ar, Position<Pos + 1>());
        }

        template <typename Archive>
        void serialize (Archive &ar, Position<dim_>) {}

        template <typename Archive, std::size_t Pos>
        void serialize (Archive &ar, Position<Pos>) const
        {
            ar & std::get<Pos>(data_);
            serialize(ar, Position<Pos + 1>());
        }

        template <typename Archive>
        void serialize (Archive &ar, Position<dim_>) const {}
    };

    template <typename S>
    struct single_particle_type : public SingleParticleBase<S>
    {
        single_particle_type (typename Particle<S>::size_type id,
                Particle<S> *particle_ptr) :
            SingleParticleBase<S>(id, particle_ptr) {}

        static VSMC_CONSTEXPR std::size_t dim () {return S::dim();}

        template <std::size_t Pos>
        typename state_type<Pos>::type &state (Position<Pos>) const
        {
            return this->mutable_particle_ptr()->value().
                state(this->id(), Position<Pos>());
        }

        template <std::size_t Pos>
        typename state_type<Pos>::type &state () const
        {return this->state(Position<Pos>());}
    };

    template <typename S>
    struct const_single_particle_type : public ConstSingleParticleBase<S>
    {
        const_single_particle_type (typename Particle<S>::size_type id,
                const Particle<S> *particle_ptr) :
            ConstSingleParticleBase<S>(id, particle_ptr) {}

        static VSMC_CONSTEXPR std::size_t dim () {return S::dim();}

        template <std::size_t Pos>
        const typename state_type<Pos>::type &state (Position<Pos>) const
        {
            return this->particle_ptr()->value().
                state(this->id(), Position<Pos>());
        }

        template <std::size_t Pos>
        const typename state_type<Pos>::type &state () const
        {return this->state(Position<Pos>());}
    };

    size_type size () const {return size_;}

    static VSMC_CONSTEXPR std::size_t dim () {return dim_;}

    state_pack_type state_pack (size_type id) const
    {
        state_pack_type pack;
        pack_particle(id, pack, Position<0>());

        return pack;
    }

    void state_unpack (size_type id, const state_pack_type &pack)
    {unpack_particle(id, pack, Position<0>());}

    template <typename IntType>
    void copy (size_type N, const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_STATE_COPY_SIZE_MISMATCH(Tuple);

        for (size_type to = 0; to != N; ++to)
            copy_particle(copy_from[to], to);
    }

    template <std::size_t Pos, typename OutputIter>
    OutputIter read_state (Position<Pos>, OutputIter first) const
    {
        const StateTuple<Order, T, Types...> *sptr =
            static_cast<const StateTuple<Order, T, Types...> *>(this);
        for (size_type i = 0; i != size_; ++i, ++first)
                *first = sptr->state(i, Position<Pos>());

        return first;
    }

    template <std::size_t Pos, typename OutputIter>
    OutputIter read_state (OutputIter first) const
    {return read_state(Position<Pos>(), first);}

    template <typename OutputStream>
    OutputStream &print (OutputStream &os, std::size_t iter = 0,
            char sepchar = ' ', char eolchar = '\n') const
    {
        for (size_type i = 0; i != size_; ++i) {
            os << iter << sepchar;
            print_particle(os, i, sepchar, eolchar, Position<0>());
        }

        return os;
    }

    protected :

    explicit StateTupleBase (size_type N) : size_(N) {}

    void copy_particle (size_type from, size_type to)
    {
        if (from != to)
            copy_particle(from, to, Position<0>());
    }

    private :

    size_type size_;
    static const std::size_t dim_ = sizeof...(Types) + 1;

    template <std::size_t Pos>
    void pack_particle (size_type id, state_pack_type &pack,
            Position<Pos>) const
    {
        const StateTuple<Order, T, Types...> *sptr =
            static_cast<const StateTuple<Order, T, Types...> *>(this);
        pack.template get<Pos>() = sptr->state(id, Position<Pos>());
        pack_particle(id, pack, Position<Pos + 1>());
    }

    void pack_particle (size_type id, state_pack_type &pack,
            Position<dim_>) const {}

    template <std::size_t Pos>
    void unpack_particle (size_type id, const state_pack_type &pack,
            Position<Pos>)
    {
        StateTuple<Order, T, Types...> *sptr =
            static_cast<StateTuple<Order, T, Types...> *>(this);
        sptr->state(id, Position<Pos>()) = pack.template get<Pos>();
        unpack_particle(id, pack, Position<Pos + 1>());
    }

    void unpack_particle (size_type id, const state_pack_type &pack,
            Position<dim_>) {}

    template <std::size_t Pos>
    void copy_particle (size_type from, size_type to, Position<Pos>)
    {
        StateTuple<Order, T, Types...> *sptr =
            static_cast<StateTuple<Order, T, Types...> *>(this);
        sptr->state(to, Position<Pos>()) = sptr->state(from, Position<Pos>());
        copy_particle(from, to, Position<Pos + 1>());
    }

    void copy_particle (size_type from, size_type to, Position<dim_>) {}

    template <typename OutputStream, std::size_t Pos>
    void print_particle (OutputStream &os, size_type id,
            char sepchar, char eolchar, Position<Pos>) const
    {
        const StateTuple<Order, T, Types...> *sptr =
            static_cast<const StateTuple<Order, T, Types...> *>(this);
        os << sptr->state(id, Position<Pos>()) << sepchar;
        print_particle(os, id, sepchar, eolchar, Position<Pos + 1>());
    }

    template <typename OutputStream>
    void print_particle (OutputStream &os, size_type id,
            char sepchar, char eolchar, Position<dim_ - 1>) const
    {
        const StateTuple<Order, T, Types...> *sptr =
            static_cast<const StateTuple<Order, T, Types...> *>(this);
        os << sptr->state(id, Position<dim_ - 1>()) << eolchar;
    }
}; // class StateTupleBase

/// \brief Particle::value_type subtype
/// \ingroup SMP
template <typename T, typename... Types>
class StateTuple<RowMajor, T, Types...> :
    public StateTupleBase<RowMajor, T, Types...>
{
    public :

    typedef StateTupleBase<RowMajor, T, Types...>
        state_tuple_base_type;
    typedef typename state_tuple_base_type::size_type size_type;

    explicit StateTuple (size_type N) : state_tuple_base_type(N), state_(N) {}

    template <std::size_t Pos>
    typename state_tuple_base_type::template state_type<Pos>::type
    &state (size_type id, Position<Pos>)
    {return std::get<Pos>(state_[id]);}

    template <std::size_t Pos>
    const typename state_tuple_base_type::template state_type<Pos>::type
    &state (size_type id, Position<Pos>) const
    {return std::get<Pos>(state_[id]);}

    template <std::size_t Pos>
    typename state_tuple_base_type::template state_type<Pos>::type
    &state (size_type id)
    {return state(id, Position<Pos>());}

    template <std::size_t Pos>
    const typename state_tuple_base_type::template state_type<Pos>::type
    &state (size_type id) const
    {return state(id, Position<Pos>());}

    private :

    std::vector<std::tuple<T, Types...> > state_;
}; // StateTuple

/// \brief Particle::value_type subtype
/// \ingroup SMP
template <typename T, typename... Types>
class StateTuple<ColMajor, T, Types...> :
    public StateTupleBase<ColMajor, T, Types...>
{
    public :

    typedef StateTupleBase<ColMajor, T, Types...>
        state_tuple_base_type;
    typedef typename state_tuple_base_type::size_type size_type;

    explicit StateTuple (size_type N) : state_tuple_base_type(N)
    {init_state(N, Position<0>());}

    template <std::size_t Pos>
    typename state_tuple_base_type::template state_type<Pos>::type
    &state (size_type id, Position<Pos>)
    {return std::get<Pos>(state_)[id];}

    template <std::size_t Pos>
    const typename state_tuple_base_type::template state_type<Pos>::type
    &state (size_type id, Position<Pos>) const
    {return std::get<Pos>(state_)[id];}

    template <std::size_t Pos>
    typename state_tuple_base_type::template state_type<Pos>::type
    &state (size_type id)
    {return state(id, Position<Pos>());}

    template <std::size_t Pos>
    const typename state_tuple_base_type::template state_type<Pos>::type
    &state (size_type id) const
    {return state(id, Position<Pos>());}

    private :

    typename internal::TupleApplyVector<std::tuple<T, Types...> >::type state_;

    template <std::size_t Pos>
    void init_state (size_type N, Position<Pos>)
    {
        std::get<Pos>(state_).resize(N);
        init_state(N, Position<Pos + 1>());
    }

    void init_state (size_type N, Position<sizeof...(Types)>)
    {std::get<sizeof...(Types)>(state_).resize(N);}
}; // class StateTuple

} // namespace vsmc

#endif // VSMC_SMP_STATE_TUPLE_HPP
