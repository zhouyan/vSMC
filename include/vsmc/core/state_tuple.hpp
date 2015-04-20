//============================================================================
// vSMC/include/vsmc/core/state_tuple.hpp
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

#ifndef VSMC_CORE_STATE_TUPLE_HPP
#define VSMC_CORE_STATE_TUPLE_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/single_particle.hpp>
#include <tuple>

#define VSMC_RUNTIME_ASSERT_CORE_STATE_TUPLE_COPY_SIZE_MISMATCH              \
    VSMC_RUNTIME_ASSERT((N == static_cast<size_type>(this->size())),         \
        ("**StateTuple::copy** SIZE MISMATCH"))

namespace vsmc
{

/// \brief Base type of StateTuple
/// \ingroup Core
template <MatrixOrder Order, typename T, typename... Types>
class StateTupleBase
{
    public:
    typedef std::size_t size_type;
    typedef std::tuple<T, Types...> state_tuple_type;
    typedef std::tuple<T *, Types *...> state_tuple_ptr_type;
    typedef std::tuple<const T *, const Types *...> state_tuple_cptr_type;

    template <std::size_t Pos>
    struct state_type {
        typedef typename std::tuple_element<Pos, state_tuple_type>::type type;
    };

    struct state_pack_type {
        state_pack_type() {}

        state_pack_type(const state_pack_type &other) : data_(other.data_) {}

        state_pack_type(const state_tuple_type &tp) : data_(tp) {}

        state_pack_type &operator=(const state_pack_type &other)
        {
            if (this != &other)
                data_ = other.data_;

            return *this;
        }

        state_pack_type(state_pack_type &&other)
            : data_(std::move(other.data_))
        {
        }

        state_pack_type(state_tuple_type &&tp) : data_(std::move(tp)) {}

        state_pack_type &operator=(state_pack_type &&other)
        {
            if (this != &other)
                data_ = std::move(other.data_);

            return *this;
        }

        state_tuple_type &data() { return data_; }

        const state_tuple_type &data() const { return data_; }

        template <typename Archive>
        void serialize(Archive &ar, const unsigned)
        {
            serialize(ar, Position<0>());
        }

        template <typename Archive>
        void serialize(Archive &ar, const unsigned) const
        {
            serialize(ar, Position<0>());
        }

        private:
        state_tuple_type data_;
        static constexpr const std::size_t dim_ = sizeof...(Types) + 1;

        template <typename Archive, std::size_t Pos>
        void serialize(Archive &ar, Position<Pos>)
        {
            ar &std::get<Pos>(data_);
            serialize(ar, Position<Pos + 1>());
        }

        template <typename Archive>
        void serialize(Archive &, Position<dim_>)
        {
        }

        template <typename Archive, std::size_t Pos>
        void serialize(Archive &ar, Position<Pos>) const
        {
            ar &std::get<Pos>(data_);
            serialize(ar, Position<Pos + 1>());
        }

        template <typename Archive>
        void serialize(Archive &, Position<dim_>) const
        {
        }
    }; // struct state_pack_type

    template <typename S>
    struct single_particle_type : public SingleParticleBase<S> {
        single_particle_type(
            typename Particle<S>::size_type id, Particle<S> *particle_ptr)
            : SingleParticleBase<S>(id, particle_ptr)
        {
        }

        static constexpr std::size_t dim() { return S::dim(); }

        template <std::size_t Pos>
        typename state_type<Pos>::type &state(Position<Pos>) const
        {
            return this->mutable_particle_ptr()->value().state(
                this->id(), Position<Pos>());
        }

        template <std::size_t Pos>
        typename state_type<Pos>::type &state() const
        {
            return this->state(Position<Pos>());
        }
    }; // struct single_particle_type

    template <typename S>
    struct const_single_particle_type : public ConstSingleParticleBase<S> {
        const_single_particle_type(typename Particle<S>::size_type id,
            const Particle<S> *particle_ptr)
            : ConstSingleParticleBase<S>(id, particle_ptr)
        {
        }

        static constexpr std::size_t dim() { return S::dim(); }

        template <std::size_t Pos>
        const typename state_type<Pos>::type &state(Position<Pos>) const
        {
            return this->particle_ptr()->value().state(
                this->id(), Position<Pos>());
        }

        template <std::size_t Pos>
        const typename state_type<Pos>::type &state() const
        {
            return this->state(Position<Pos>());
        }
    }; // struct const_single_particle_type

    size_type size() const { return size_; }

    static constexpr std::size_t dim() { return dim_; }

    template <std::size_t Pos, typename OutputIter>
    void read_state(Position<Pos>, OutputIter first) const
    {
        const StateTuple<Order, T, Types...> *sptr =
            static_cast<const StateTuple<Order, T, Types...> *>(this);
        for (size_type i = 0; i != size_; ++i, ++first)
            *first = sptr->state(i, Position<Pos>());
    }

    template <std::size_t Pos, typename OutputIter>
    void read_state(OutputIter first) const
    {
        read_state(Position<Pos>(), first);
    }

    template <typename CharT, typename Traits>
    std::basic_ostream<CharT, Traits> &print(
        std::basic_ostream<CharT, Traits> &os, char sepchar = '\t') const
    {
        if (dim_ == 0 || size_ == 0 || !os.good())
            return os;

        for (size_type i = 0; i != size_; ++i)
            print_particle(os, i, sepchar, Position<0>());

        return os;
    }

    protected:
    explicit StateTupleBase(size_type N) : size_(N) {}

    private:
    size_type size_;
    static constexpr const std::size_t dim_ = sizeof...(Types) + 1;

    template <std::size_t Pos, typename CharT, typename Traits>
    void print_particle(std::basic_ostream<CharT, Traits> &os, size_type id,
        char sepchar, Position<Pos>) const
    {
        const StateTuple<Order, T, Types...> *sptr =
            static_cast<const StateTuple<Order, T, Types...> *>(this);
        os << sptr->state(id, Position<Pos>()) << sepchar;
        print_particle(os, id, sepchar, Position<Pos + 1>());
    }

    template <typename CharT, typename Traits>
    void print_particle(std::basic_ostream<CharT, Traits> &os, size_type id,
        char, Position<dim_ - 1>) const
    {
        const StateTuple<Order, T, Types...> *sptr =
            static_cast<const StateTuple<Order, T, Types...> *>(this);
        os << sptr->state(id, Position<dim_ - 1>()) << '\n';
    }
}; // class StateTupleBase

template <typename CharT, typename Traits, MatrixOrder Order, typename T,
    typename... Types>
inline std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os,
    const StateTupleBase<Order, T, Types...> &stuple)
{
    return stuple.print(os);
}

/// \brief Particle::value_type subtype
/// \ingroup Core
template <typename T, typename... Types>
class StateTuple<RowMajor, T, Types...>
    : public StateTupleBase<RowMajor, T, Types...>
{
    public:
    typedef StateTupleBase<RowMajor, T, Types...> state_tuple_base_type;
    typedef typename state_tuple_base_type::size_type size_type;
    typedef typename state_tuple_base_type::state_pack_type state_pack_type;

    explicit StateTuple(size_type N) : state_tuple_base_type(N), state_(N) {}

    template <std::size_t Pos>
    typename state_tuple_base_type::template state_type<Pos>::type &state(
        size_type id, Position<Pos>)
    {
        return std::get<Pos>(state_[id]);
    }

    template <std::size_t Pos>
    const typename state_tuple_base_type::template state_type<Pos>::type &
        state(size_type id, Position<Pos>) const
    {
        return std::get<Pos>(state_[id]);
    }

    template <std::size_t Pos>
    typename state_tuple_base_type::template state_type<Pos>::type &state(
        size_type id)
    {
        return state(id, Position<Pos>());
    }

    template <std::size_t Pos>
    const typename state_tuple_base_type::template state_type<Pos>::type &
        state(size_type id) const
    {
        return state(id, Position<Pos>());
    }

    typename state_tuple_base_type::state_tuple_type *data()
    {
        return &state_[0];
    }

    const typename state_tuple_base_type::state_tuple_type *data() const
    {
        return &state_[0];
    }

    template <typename IntType>
    void copy(size_type N, const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_TUPLE_COPY_SIZE_MISMATCH;

        for (size_type to = 0; to != N; ++to)
            copy_particle(copy_from[to], to);
    }

    void copy_particle(size_type from, size_type to)
    {
        state_[to] = state_[from];
    }

    state_pack_type state_pack(size_type id) const
    {
        return state_pack_type(state_[id]);
    }

    void state_unpack(size_type id, const state_pack_type &pack)
    {
        state_[id] = pack.data();
    }

    void state_unpack(size_type id, state_pack_type &&pack)
    {
        state_[id] = std::move(pack.data());
    }

    private:
    static constexpr const std::size_t dim_ = sizeof...(Types) + 1;
    std::vector<std::tuple<T, Types...>> state_;
}; // StateTuple

/// \brief Particle::value_type subtype
/// \ingroup Core
template <typename T, typename... Types>
class StateTuple<ColMajor, T, Types...>
    : public StateTupleBase<ColMajor, T, Types...>
{
    public:
    typedef StateTupleBase<ColMajor, T, Types...> state_tuple_base_type;
    typedef typename state_tuple_base_type::size_type size_type;
    typedef typename state_tuple_base_type::state_pack_type state_pack_type;

    explicit StateTuple(size_type N) : state_tuple_base_type(N)
    {
        init_state(N, Position<0>());
    }

    template <std::size_t Pos>
    typename state_tuple_base_type::template state_type<Pos>::type &state(
        size_type id, Position<Pos>)
    {
        return std::get<Pos>(state_)[id];
    }

    template <std::size_t Pos>
    const typename state_tuple_base_type::template state_type<Pos>::type &
        state(size_type id, Position<Pos>) const
    {
        return std::get<Pos>(state_)[id];
    }

    template <std::size_t Pos>
    typename state_tuple_base_type::template state_type<Pos>::type &state(
        size_type id)
    {
        return state(id, Position<Pos>());
    }

    template <std::size_t Pos>
    const typename state_tuple_base_type::template state_type<Pos>::type &
        state(size_type id) const
    {
        return state(id, Position<Pos>());
    }

    template <std::size_t Pos>
    typename state_tuple_base_type::template state_type<Pos>::type *data(
        Position<Pos>)
    {
        return &std::get<Pos>(state_)[0];
    }

    template <std::size_t Pos>
    const typename state_tuple_base_type::template state_type<Pos>::type *
        data(Position<Pos>) const
    {
        return &std::get<Pos>(state_)[0];
    }

    template <std::size_t Pos>
    typename state_tuple_base_type::template state_type<Pos>::type *data()
    {
        return &std::get<Pos>(state_)[0];
    }

    template <std::size_t Pos>
    const typename state_tuple_base_type::template state_type<Pos>::type *
        data() const
    {
        return &std::get<Pos>(state_)[0];
    }

    typename state_tuple_base_type::state_tuple_ptr_type data()
    {
        typename state_tuple_base_type::state_tuple_ptr_type dptr;
        insert_data(dptr, Position<0>());

        return dptr;
    }

    typename state_tuple_base_type::state_tuple_cptr_type data() const
    {
        typename state_tuple_base_type::state_tuple_cptr_type dptr;
        insert_data(dptr, Position<0>());

        return dptr;
    }

    template <typename IntType>
    void copy(size_type N, const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_TUPLE_COPY_SIZE_MISMATCH;

        for (size_type to = 0; to != N; ++to)
            copy_particle(copy_from[to], to);
    }

    void copy_particle(size_type from, size_type to)
    {
        if (from == to)
            return;

        copy_particle(from, to, Position<0>());
    }

    state_pack_type state_pack(size_type id) const
    {
        state_pack_type pack;
        pack_particle(id, pack, Position<0>());

        return pack;
    }

    void state_unpack(size_type id, const state_pack_type &pack)
    {
        unpack_particle(id, pack, Position<0>());
    }

    void state_unpack(size_type id, state_pack_type &&pack)
    {
        unpack_particle(id, std::move(pack), Position<0>());
    }

    private:
    static constexpr const std::size_t dim_ = sizeof...(Types) + 1;
    std::tuple<std::vector<T>, std::vector<Types>...> state_;

    template <std::size_t Pos>
    void init_state(size_type N, Position<Pos>)
    {
        std::get<Pos>(state_).resize(N);
        init_state(N, Position<Pos + 1>());
    }

    void init_state(size_type N, Position<sizeof...(Types)>)
    {
        std::get<sizeof...(Types)>(state_).resize(N);
    }

    template <std::size_t Pos, typename PTRType>
    void insert_data(PTRType &dptr, Position<Pos>) const
    {
        std::get<Pos>(dptr) = data<Pos>();
        insert_data(dptr, Position<Pos + 1>());
    }

    template <typename PTRType>
    void insert_data(PTRType &dptr, Position<sizeof...(Types)>) const
    {
        std::get<sizeof...(Types)>(dptr) = data<sizeof...(Types)>();
    }

    template <std::size_t Pos>
    void copy_particle(size_type from, size_type to, Position<Pos>)
    {
        state(to, Position<Pos>()) = state(from, Position<Pos>());
        copy_particle(from, to, Position<Pos + 1>());
    }

    void copy_particle(size_type, size_type, Position<dim_>) {}

    template <std::size_t Pos>
    void pack_particle(
        size_type id, state_pack_type &pack, Position<Pos>) const
    {
        std::get<Pos>(pack.data()) = state(id, Position<Pos>());
        pack_particle(id, pack, Position<Pos + 1>());
    }

    void pack_particle(size_type, state_pack_type &, Position<dim_>) const {}

    template <std::size_t Pos>
    void unpack_particle(
        size_type id, const state_pack_type &pack, Position<Pos>)
    {
        state(id, Position<Pos>()) = std::get<Pos>(pack.data());
        unpack_particle(id, pack, Position<Pos + 1>());
    }

    void unpack_particle(size_type, const state_pack_type &, Position<dim_>)
    {
    }

    template <std::size_t Pos>
    void unpack_particle(size_type id, state_pack_type &&pack, Position<Pos>)
    {
        state(id, Position<Pos>()) = std::move(std::get<Pos>(pack.data()));
        unpack_particle(id, std::move(pack), Position<Pos + 1>());
    }

    void unpack_particle(size_type, state_pack_type &&, Position<dim_>) {}
}; // class StateTuple

} // namespace vsmc

#endif // VSMC_CORE_STATE_TUPLE_HPP
