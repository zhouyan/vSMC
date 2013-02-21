#ifndef VSMC_CORE_SINGLE_PARTICLE_HPP
#define VSMC_CORE_SINGLE_PARTICLE_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

/// \brief Particle iterator
/// \ingroup Core
///
/// \details
/// A particle iterator is a random access iterator. However it has some
/// special properties.
/// - ParticleIterator::poiner is a pointer to a constant SingleParticle or
/// ConstSingleParticle object
/// - ParticleIterator::reference is a reference to a constant SingleParticle
/// or ConstSingleParticle object
/// These means that when calling SingleParticle or ConstSingleParticle member
/// functions through an iterator, only const member functions are possible.
/// However, this is not really a restriction, since SingleParticle and
/// ConstSingleParticle are supposed to be used to access Particle object. They
/// may provide write access to the Particle object. However, themselves cannot
/// be changed. That is, the following will be ill-formed,
/// \code
/// auto iter = particle.begin();
/// *iter = SingleParticle(0, 0);
/// \endcode
/// This is the supposed behavior, since change the SingleParticle object
/// pointed by the iterator is meaningless and logically wrong. The
/// constantness of the iterators are related to the constantness of the
/// Particle object.
template <typename T, template <typename> class SPType>
class ParticleIterator :
    public std::iterator<std::random_access_iterator_tag, SPType<T>,
    std::ptrdiff_t, const SPType<T> *, const SPType<T> &>
{
    typedef std::iterator<std::random_access_iterator_tag, SPType<T>,
        std::ptrdiff_t, const SPType<T> *, const SPType<T> &>
        base_iterator_type;

    public :

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
    {ptr_ = other.ptr_; return *this;}

    template <template <typename> class OtherSPType>
    ParticleIterator<T, SPType> &operator= (
            const ParticleIterator<T, OtherSPType> &other)
    {ptr_ = &other->particle().sp(other->id()); return *this;}

    reference operator* () const {return *ptr_;}

    pointer operator-> () const {return ptr_;}

    value_type operator[] (difference_type diff) const {return *(ptr_ + diff);}

    ParticleIterator<T, SPType> &operator++ () {++ptr_; return *this;}

    ParticleIterator<T, SPType> operator++ (int)
    {ParticleIterator<T, SPType> iter(*this); return ++iter;}

    ParticleIterator<T, SPType> &operator-- () {--ptr_; return *this;}

    ParticleIterator<T, SPType> operator-- (int)
    {ParticleIterator<T, SPType> iter(*this); return --iter;}

    ParticleIterator<T, SPType> &operator+= (difference_type diff)
    {ptr_ += diff; return *this;}

    ParticleIterator<T, SPType> &operator-= (difference_type diff)
    {ptr_ -= diff; return *this;}

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
        const ParticleIterator<T, SPType> &iter) {return iter + diff;}

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

    SingleParticleBase (typename Particle<T>::size_type id,
            Particle<T> *particle_ptr) :
        id_(id), particle_ptr_(particle_ptr) {}

    SingleParticleBase (const SingleParticle<T> &other) :
        id_(other.id_), particle_ptr_(other.particle_ptr_) {}

    SingleParticleBase &operator= (const SingleParticle<T> &other)
    {
        id_ = other.id_;
        particle_ptr_ = other.particle_ptr_;

        return *this;
    }

    typename Particle<T>::size_type id () const {return id_;}

    const Particle<T> &particle () const {return *particle_ptr_;}

    const Particle<T> *particle_ptr () const {return particle_ptr_;}

    typename Particle<T>::rng_type &rng () const
    {return particle_ptr_->rng(id_);}

    protected :

    Particle<T> *mutable_particle_ptr () const {return particle_ptr_;}

    private :

    typename Particle<T>::size_type id_;
    Particle<T> *particle_ptr_;
}; // class SingleParticle

/// \brief A const variant to SingleParticle
/// \ingroup Core
template <typename T>
class ConstSingleParticleBase
{
    public :

    ConstSingleParticleBase (typename Particle<T>::size_type id,
            const Particle<T> *particle_ptr) :
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

    typename Particle<T>::size_type id () const {return id_;}

    const Particle<T> &particle () const {return *particle_ptr_;}

    const Particle<T> *particle_ptr () const {return particle_ptr_;}

    private :

    typename Particle<T>::size_type id_;
    const Particle<T> *particle_ptr_;
}; // class ConstSingleParticle

/// \brief A thin wrapper over a complete Particle
/// \ingroup Core
///
/// \details
/// This is the basic SingleParticle available for any type of Particle. To
/// extend it for type `T`. One can either specialize
/// vsmc::traits::SingleParticleTypeTrait<T,T> or define a class template
/// named `single_particle_type` within `T` with the following minimum
/// requirement.
/// \code
/// template <typename S> // S: StateType, such as StateMatrix<Dim, T>
/// struct single_particle_type
/// {
///     typedef IntType size_type;
///     single_particle_type (size_type id, Particle<S> *particle_ptr);
///     size_type id () const;
///     const Particle<S> *particle_ptr () const;
/// };
/// \endcode
/// Usually you can safely derive `single_particle_type<S>` from
/// SingleParticleBase<S> and add methods specific to `S`.
template <typename T>
class SingleParticle :
    public traits::SingleParticleTypeTrait<T, T>::type
{
    typedef typename traits::SingleParticleTypeTrait<T, T>::type base;

    public :

    SingleParticle (typename Particle<T>::size_type id,
            Particle<T> *particle_ptr) : base(id, particle_ptr) {}
};

/// \brief A const variant to SingleParticle
/// \ingroup Core
template <typename T>
class ConstSingleParticle :
    public traits::ConstSingleParticleTypeTrait<T, T>::type
{
    typedef typename traits::ConstSingleParticleTypeTrait<T, T>::type base;

    public :

    ConstSingleParticle (typename Particle<T>::size_type id,
            const Particle<T> *particle_ptr) : base(id, particle_ptr) {}

    ConstSingleParticle (const SingleParticle<T> &other) :
        base(other.id(), other.particle_ptr()) {}

    ConstSingleParticle &operator= (const SingleParticle<T> &other)
    {base::operator=(base(other.id(), other.particle_ptr())); return *this;}
};

} // namespace vsmc

#endif // VSMC_CORE_SINGLE_PARTICLE_HPP
