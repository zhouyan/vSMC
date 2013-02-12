#ifndef VSMC_CORE_SINGLE_PARTICLE_HPP
#define VSMC_CORE_SINGLE_PARTICLE_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

/// \brief Particle iterator
/// \ingroup Core
template <typename T, template <typename> class SPType>
class ParticleIterator :
    public std::iterator<std::random_access_iterator_tag, SPType<T>,
    std::ptrdiff_t, const SPType<T> *, const SPType<T> &>
{
    public :

    typedef std::iterator<std::random_access_iterator_tag, SPType<T>,
        std::ptrdiff_t, const SPType<T> *, const SPType<T> &>
        base_iterator_type;
    typedef typename base_iterator_type::value_type      value_type;
    typedef typename base_iterator_type::difference_type difference_type;
    typedef typename base_iterator_type::pointer         pointer;
    typedef typename base_iterator_type::reference       reference;

    ParticleIterator () : ptr_(VSMC_NULLPTR) {}

    ParticleIterator (pointer ptr) : ptr_(ptr) {}

    ParticleIterator (const ParticleIterator<T, SPType> &other) :
        ptr_(other.ptr_) {}

    template <template <typename> class OtherSPType>
    ParticleIterator (const ParticleIterator<T, OtherSPType> &other) :
        ptr_(&other->particle().sp(other->id())) {}

    ParticleIterator<T, SPType> &operator= (
            const ParticleIterator<T, SPType> &other)
    {
        ptr_ = other.ptr_;

        return *this;
    }

    template <template <typename> class OtherSPType>
    ParticleIterator<T, SPType> &operator= (
            const ParticleIterator<T, OtherSPType> &other)
    {
        /// \internal
        /// If SPType is SingleParticle, OtherSPType is ConstSingleParticle,
        /// The following shall results in a compile time error.
        /// If SPType is ConstSingleParticle, OtherSPType is SingleParticle,
        /// The following shall be fine.
        /// The copy constructor is similar.
        /// It is still a little too tricky here.
        ptr_ = &other->particle().sp(other->id());

        return *this;
    }

    reference operator* () const
    {
        return *ptr_;
    }

    pointer operator-> () const
    {
        return ptr_;
    }

    value_type operator[] (difference_type diff) const
    {
        return *(ptr_ + diff);
    }

    ParticleIterator<T, SPType> &operator++ ()
    {
        ++ptr_;

        return *this;
    }

    ParticleIterator<T, SPType> operator++ (int)
    {
        ParticleIterator<T, SPType> iter(*this);

        return ++iter;
    }

    ParticleIterator<T, SPType> &operator-- ()
    {
        --ptr_;

        return *this;
    }

    ParticleIterator<T, SPType> operator-- (int)
    {
        ParticleIterator<T, SPType> iter(*this);

        return --iter;
    }

    ParticleIterator<T, SPType> &operator+= (difference_type diff)
    {
        ptr_ += diff;

        return *this;
    }


    ParticleIterator<T, SPType> &operator-= (difference_type diff)
    {
        ptr_ -= diff;

        return *this;
    }

    private :

    pointer ptr_;
}; // class ParticleIterator

/// \brief Particle iterator operator==
/// \ingroup Core
template <typename T,
         template <typename> class SPType1, template <typename> class SPType2>
bool operator== (
        const ParticleIterator<T, SPType1> &iter1,
        const ParticleIterator<T, SPType2> &iter2)
{
    return
        (iter1->id() == iter2->id()) &&
        (iter1->particle_ptr() == iter2->particle_ptr());
}

/// \brief Particle iterator operator!=
/// \ingroup Core
template <typename T,
         template <typename> class SPType1, template <typename> class SPType2>
bool operator!= (
        const ParticleIterator<T, SPType1> &iter1,
        const ParticleIterator<T, SPType2> &iter2)
{
    return
        (iter1->id() != iter2->id()) ||
        (iter1->particle_ptr() != iter2->particle_ptr());
}

/// \brief Particle iterator operator-
/// \ingroup Core
template <typename T,
         template <typename> class SPType1, template <typename> class SPType2>
std::ptrdiff_t operator- (
        const ParticleIterator<T, SPType1> &iter1,
        const ParticleIterator<T, SPType2> &iter2)
{
    VSMC_RUNTIME_ASSERT_PARTICLE_ITERATOR_BINARY_OP;

    return iter1->id() - iter2->id();
}

/// \brief Particle iterator operator+
/// \ingroup Core
template <typename T, template <typename> class SPType>
ParticleIterator<T, SPType> operator+ (
        const ParticleIterator<T, SPType> &iter,
        typename ParticleIterator<T, SPType>::difference_type diff)
{
    ParticleIterator<T, SPType> new_iter(iter);
    new_iter += diff;

    return new_iter;
}

/// \brief Particle iterator operator+
/// \ingroup Core
template <typename T, template <typename> class SPType>
ParticleIterator<T, SPType> operator+ (
        typename ParticleIterator<T, SPType>::difference_type diff,
        const ParticleIterator<T, SPType> &iter)
{
    return iter + diff;
}

/// \brief Particle iterator operator-
/// \ingroup Core
template <typename T, template <typename> class SPType>
ParticleIterator<T, SPType> operator- (
        const ParticleIterator<T, SPType> &iter,
        typename ParticleIterator<T, SPType>::difference_type diff)
{
    ParticleIterator<T, SPType> new_iter(iter);
    new_iter -= diff;

    return new_iter;
}

/// \brief A thin wrapper over a complete Particle
/// \ingroup Core
template <typename T>
class SingleParticleBase
{
    public :

    typedef Particle<T> particle_type;
    typedef particle_type * particle_ptr_type;
    typedef typename particle_type::size_type size_type;
    typedef typename particle_type::rng_type rng_type;

    SingleParticleBase (size_type id, particle_ptr_type particle_ptr) :
        id_(id), particle_ptr_(particle_ptr) {}

    SingleParticleBase (const SingleParticle<T> &other) :
        id_(other.id_), particle_ptr_(other.particle_ptr_) {}

    SingleParticleBase &operator= (const SingleParticle<T> &other)
    {
        id_ = other.id_;
        particle_ptr_ = other.particle_ptr_;

        return *this;
    }

    size_type id () const
    {
        return id_;
    }

    const particle_type &particle () const
    {
        return *particle_ptr_;
    }

    const Particle<T> *particle_ptr () const
    {
        return particle_ptr_;
    }

    rng_type &rng () const
    {
        return particle_ptr_->rng(id_);
    }

    protected :

    Particle<T> *mutable_particle_ptr () const
    {
        return particle_ptr_;
    }

    private :

    size_type id_;
    particle_ptr_type particle_ptr_;
}; // class SingleParticle

/// \brief A const variant to SingleParticle
/// \ingroup Core
template <typename T>
class ConstSingleParticleBase
{
    public :

    typedef Particle<T> particle_type;
    typedef const particle_type * particle_ptr_type;
    typedef typename particle_type::size_type size_type;

    ConstSingleParticleBase (size_type id, particle_ptr_type particle_ptr) :
        id_(id), particle_ptr_(particle_ptr) {}

    ConstSingleParticleBase (const ConstSingleParticle<T> &other) :
        id_(other.id_), particle_ptr_(other.particle_ptr_) {}

    ConstSingleParticleBase &operator= (
            const ConstSingleParticleBase<T> &other)
    {
        id_ = other.id_;
        particle_ptr_ = other.particle_ptr_;

        return *this;
    }

    size_type id () const
    {
        return id_;
    }

    const particle_type &particle () const
    {
        return *particle_ptr_;
    }

    const Particle<T> *particle_ptr () const
    {
        return particle_ptr_;
    }

    private :

    size_type id_;
    particle_ptr_type particle_ptr_;
}; // class ConstSingleParticle

template <typename T>
class SingleParticle :
    public traits::SingleParticleTypeTrait<T, T>::type
{
    public :

    typedef typename traits::SingleParticleTypeTrait<T, T>::type
        base_sp_type;

    SingleParticle (
            typename base_sp_type::size_type id,
            typename base_sp_type::particle_ptr_type particle_ptr) :
        base_sp_type(id, particle_ptr) {}
};

template <typename T>
class ConstSingleParticle :
    public traits::ConstSingleParticleTypeTrait<T, T>::type
{
    public :

    typedef typename traits::ConstSingleParticleTypeTrait<T, T>::type
        base_csp_type;

    ConstSingleParticle (
            typename base_csp_type::size_type id,
            typename base_csp_type::particle_ptr_type particle_ptr) :
        base_csp_type(id, particle_ptr) {}

    ConstSingleParticle (const SingleParticle<T> &other) :
        base_csp_type(other.id(), other.particle_ptr()) {}

    ConstSingleParticle &operator= (const SingleParticle<T> &other)
    {
        base_csp_type::operator=(base(other.id(), other.particle_ptr()));

        return *this;
    }
};

} // namespace vsmc

#endif // VSMC_CORE_SINGLE_PARTICLE_HPP
