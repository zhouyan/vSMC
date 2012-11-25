#ifndef VSMC_CORE_PARTICLE_HPP
#define VSMC_CORE_PARTICLE_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/resample.hpp>
#include <vsmc/core/rng.hpp>
#include <vsmc/core/weight.hpp>

#define VSMC_CONST_SINGLE_PARTICLE_VALID_RUNTIME_ASSERT \
    VSMC_RUNTIME_ASSERT(particle_ptr_,                              \
            ("A **ConstSingleParticle** object "                    \
             "is contructed with 0 **Particle** pointer"));         \
    VSMC_RUNTIME_ASSERT((id_ >= 0 && id_ <= particle_ptr_->size()), \
            ("A **ConstSignleParticle** object "                    \
             "is contructed with an out of range id"));

#define VSMC_SINGLE_PARTICLE_VALID_RUNTIME_ASSERT \
    VSMC_RUNTIME_ASSERT(particle_ptr_,                              \
            ("A **SingleParticle** object "                         \
             "is contructed with 0 **Particle** pointer"));         \
    VSMC_RUNTIME_ASSERT((id_ >= 0 && id_ <= particle_ptr_->size()), \
            ("A **SignleParticle** object "                         \
             "is contructed with an out of range id"));

VSMC_DEFINE_TYPE_DISPATCH_TRAIT(StateType, state_type, void);

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
    VSMC_RUNTIME_ASSERT((iter1->particle_ptr() == iter2->particle_ptr()),
            "**ParticleIterator** BELONG TO TWO DIFFERENT PARTICLE COLLECTION"
            "CAN NOT BE SUBSTRACTED");

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

/// \brief A const variant to SingleParticle
/// \ingroup Core
template <typename T>
class ConstSingleParticle
{
    public :

    typedef T value_type;
    typedef Particle<T> particle_type;
    typedef const particle_type * particle_ptr_type;
    typedef typename particle_type::size_type size_type;
    typedef typename particle_type::rng_type rng_type;

    ConstSingleParticle (size_type id, particle_ptr_type particle_ptr) :
        id_(id), particle_ptr_(particle_ptr) {}

    ConstSingleParticle (const ConstSingleParticle<T> &other) :
        id_(other.id_), particle_ptr_(other.particle_ptr_) {}

    ConstSingleParticle (const SingleParticle<T> &other) :
        id_(other.id()), particle_ptr_(other.particle_ptr()) {}

    ConstSingleParticle &operator= (const ConstSingleParticle<T> &other)
    {
        id_ = other.id_;
        particle_ptr_ = other.particle_ptr_;

        return *this;
    }

    ConstSingleParticle &operator= (const SingleParticle<T> &other)
    {
        id_ = other.id_;
        particle_ptr_ = other.particle_ptr_;

        return *this;
    }

    size_type id () const
    {
        return id_;
    }

    template <typename UIntType>
    const typename cxx11::enable_if<
        traits::HasStateType<T>::value && sizeof(UIntType),
        typename traits::StateTypeTrait<T>::type>::type
    &state (UIntType pos) const
    {

        return particle_ptr_->value().state(id_, pos);
    }

    double weight () const
    {
        VSMC_CONST_SINGLE_PARTICLE_VALID_RUNTIME_ASSERT;

        return particle_ptr_->weight()[id_];
    }

    double log_weight () const
    {
        VSMC_CONST_SINGLE_PARTICLE_VALID_RUNTIME_ASSERT;

        return particle_ptr_->log_weight()[id_];
    }

    const particle_type &particle () const
    {
        VSMC_CONST_SINGLE_PARTICLE_VALID_RUNTIME_ASSERT;

        return *particle_ptr_;
    }

    particle_ptr_type particle_ptr () const
    {
        VSMC_CONST_SINGLE_PARTICLE_VALID_RUNTIME_ASSERT;

        return particle_ptr_;
    }

    friend class Particle<T>;

    private :

    size_type id_;
    particle_ptr_type particle_ptr_;

    void set_id (size_type new_id)
    {
        id_ = new_id;
    }
}; // class ConstSingleParticle

/// \brief A thin wrapper over a complete Particle
/// \ingroup Core
template <typename T>
class SingleParticle
{
    public :

    typedef T value_type;
    typedef Particle<T> particle_type;
    typedef particle_type * particle_ptr_type;
    typedef typename particle_type::size_type size_type;
    typedef typename particle_type::rng_type rng_type;

    SingleParticle (size_type id, particle_ptr_type particle_ptr) :
        id_(id), particle_ptr_(particle_ptr) {}

    SingleParticle (const SingleParticle<T> &other) :
        id_(other.id_), particle_ptr_(other.particle_ptr_) {}

    SingleParticle &operator= (const SingleParticle<T> &other)
    {
        id_ = other.id_;
        particle_ptr_ = other.particle_ptr_;

        return *this;
    }

    size_type id () const
    {
        return id_;
    }

    template <typename UIntType>
    typename cxx11::enable_if<
        traits::HasStateType<T>::value && sizeof(UIntType),
        typename traits::StateTypeTrait<T>::type>::type
    &state (UIntType pos) const
    {
        VSMC_SINGLE_PARTICLE_VALID_RUNTIME_ASSERT;

        return particle_ptr_->value().state(id_, pos);
    }

    double weight () const
    {
        VSMC_SINGLE_PARTICLE_VALID_RUNTIME_ASSERT;

        return particle_ptr_->weight()[id_];
    }

    double log_weight () const
    {
        VSMC_SINGLE_PARTICLE_VALID_RUNTIME_ASSERT;

        return particle_ptr_->log_weight()[id_];
    }

    const particle_type &particle () const
    {
        VSMC_SINGLE_PARTICLE_VALID_RUNTIME_ASSERT;

        return *particle_ptr_;
    }

    particle_ptr_type particle_ptr () const
    {
        VSMC_SINGLE_PARTICLE_VALID_RUNTIME_ASSERT;

        return particle_ptr_;
    }

    rng_type &rng () const
    {
        VSMC_SINGLE_PARTICLE_VALID_RUNTIME_ASSERT;

        return particle_ptr_->rng(id_);
    }

    friend class Particle<T>;

    private :

    size_type id_;
    particle_ptr_type particle_ptr_;

    void set_id (size_type new_id)
    {
        id_ = new_id;
    }
}; // class SingleParticle

/// \brief Particle class representing the whole particle set
/// \ingroup Core
/// \sa WeightSetBase
/// \sa RngSetSeq RngSetPrl
template <typename T>
class Particle
{
    public :

    typedef typename traits::SizeTypeTrait<T>::type size_type;
    typedef T value_type;
    typedef typename traits::WeightSetTypeTrait<T>::type
        weight_set_type;
    typedef typename traits::RngSetTypeTrait<T>::type
        rng_set_type;
    typedef typename rng_set_type::rng_type rng_type;
    typedef typename traits::ResampleRngSetTypeTrait<T>::type
        resample_rng_set_type;
    typedef cxx11::function<
        void (size_type, resample_rng_set_type &, double *, size_type *)>
        resample_op_type;
    typedef ParticleIterator<T, SingleParticle> iterator;
    typedef ParticleIterator<T, ConstSingleParticle> const_iterator;
    typedef std::reverse_iterator<iterator> reverse_iterator;
    typedef std::reverse_iterator<const_iterator> const_reverse_iterator;

    explicit Particle (size_type N) :
        size_(N), value_(N), weight_set_(N), rng_set_(N),
        replication_(N), copy_from_(N), weight_(N), resampled_(false),
        resample_rng_set_(N),
        sp_(N + 2, SingleParticle<value_type>(0, this)),
        csp_(N + 2, ConstSingleParticle<value_type>(0, this))
    {
        weight_set_.set_equal_weight();
        if (cxx11::is_signed<size_type>::value) {
            sp_[0].set_id(-1);
            csp_[0].set_id(-1);
        } else {
            sp_[0].set_id(std::numeric_limits<size_type>::max
                    VSMC_MINMAX_NO_EXPANSION ());
            csp_[0].set_id(std::numeric_limits<size_type>::max
                    VSMC_MINMAX_NO_EXPANSION ());
        }
        for (size_type i = 1; i != size_ + 2; ++i) {
            sp_[i].set_id(i - 1);
            csp_[i].set_id(i - 1);
        }
    }

    /// \brief Number of particles
    size_type size () const
    {
        return size_;
    }

    const SingleParticle<T> &sp (size_type id)
    {
        return sp_[id + 1];
    }

    const ConstSingleParticle<T> &sp (size_type id) const
    {
        return csp_[id + 1];
    }

    iterator begin()
    {
        return iterator(&sp(0));
    }

    iterator end()
    {
        return iterator(&sp(size_));
    }

    const_iterator begin() const
    {
        return cosnt_iterator(&sp(0));
    }

    const_iterator end() const
    {
        return const_iterator(&sp(size_));
    }

    const_iterator cbegin() const
    {
        return const_iterator(&sp(0));
    }

    const_iterator cend() const
    {
        return const_iterator(&sp(size_));
    }

    reverse_iterator rbegin()
    {
        return reverse_iterator(end());
    }

    reverse_iterator rend()
    {
        return reverse_iterator(begin());
    }

    const_reverse_iterator rbegin() const
    {
        return const_reverse_iterator(end());
    }

    const_reverse_iterator rend() const
    {
        return const_reverse_iterator(begin());
    }

    const_reverse_iterator crbegin() const
    {
        return const_reverse_iterator(cend());
    }

    const_reverse_iterator crend() const
    {
        return const_reverse_iterator(cbegin());
    }

    /// \brief Read and write access to the value collection object
    value_type &value ()
    {
        return value_;
    }

    /// \brief Read only access to the value collection object
    const value_type &value () const
    {
        return value_;
    }

    /// \brief Read and write access to the weight collection object
    weight_set_type &weight_set ()
    {
        return weight_set_;
    }

    /// \brief Read only access to the weight collection object
    const weight_set_type &weight_set () const
    {
        return weight_set_;
    }

    /// \brief Read and write access to the RNG collection object
    rng_set_type &rng_set ()
    {
        return rng_set_;
    }

    /// \brief Read only access to the RNG collection object
    const rng_set_type &rng_set () const
    {
        return rng_set_;
    }

    /// \brief Read normalized weights through an output iterator
    template <typename OutputIter>
    OutputIter read_weight (OutputIter first) const
    {
        return weight_set_.read_weight(first);
    }

    /// \brief Read normalized weights through a random access iterator with
    /// (possible non-uniform stride)
    template <typename RandomIter>
    RandomIter read_weight (RandomIter first, int stride) const
    {
        return weight_set_.read_weight(first, stride);
    }

    /// \brief Read normalized weights through a pointer
    double *read_weight (double *first) const
    {
        return weight_set_.read_weight(first);
    }

    /// \brief Read unnormalized logarithm weights through an output iterator
    template <typename OutputIter>
    OutputIter read_log_weight (OutputIter first) const
    {
        return weight_set_.read_log_weight(first);
    }

    /// \brief Read unnormalized logarithm weights through a random access
    /// iterator with (possible non-uniform stride)
    template <typename RandomIter>
    RandomIter read_log_weight (RandomIter first, int stride) const
    {
        return weight_set_.read_log_weight(first, stride);
    }

    /// \brief Read unnormalized logarithm weights through a pointer
    double *read_log_weight (double *first) const
    {
        return weight_set_.read_log_weight(first);
    }

    /// \brief Get the normalized weight of the id'th particle
    double weight (size_type id) const
    {
        return weight_set_.weight(id);
    }

    /// \brief Get the unnormalized logarithm weight of the id'th particle
    double log_weight (size_type id) const
    {
        return weight_set_.log_weight(id);
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS
    /// such that each particle has a equal weight
    void set_equal_weight ()
    {
        weight_set_.set_equal_weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) weights directly through an input
    /// iterator
    template <typename InputIter>
    void set_weight (InputIter first)
    {
        weight_set_.set_weight(first);
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) weights directly through a random
    /// access iterator with (possible non-uniform) stride
    template <typename RandomIter>
    void set_weight (RandomIter first, int stride)
    {
        weight_set_.set_weight(first, stride);
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) weights directly through a pointer
    void set_weight (const double *first)
    {
        weight_set_.set_weight(first);
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// multiply the normalized weight with (possible unnormalized) incremental
    /// weights through an input iterator
    template <typename InputIter>
    void mul_weight (InputIter first)
    {
        weight_set_.mul_weight(first);
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// multiply the normalized weight with (possible unnormalized) incremental
    /// weights through a random access iterator with (possible non-uniform)
    /// stride
    template <typename RandomIter>
    void mul_weight (RandomIter first, int stride)
    {
        weight_set_.mul_weight(first, stride);
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) logarithm weights directly through
    /// an input iterator
    template <typename InputIter>
    void set_log_weight (InputIter first)
    {
        weight_set_.set_log_weight(first);
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) logarithm weights directly through
    /// a random access iterator with (possible non-uniform) stride
    template <typename RandomIter>
    void set_log_weight (RandomIter first, int stride)
    {
        weight_set_.set_log_weight(first, stride);
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) logarithm weights directly through
    /// a pointer
    void set_log_weight (const double *first)
    {
        weight_set_.set_log_weight(first);
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// adding to the unnormalized logarithm weights with (possible
    /// unormalized) logarithm incremental weights through an input iterator
    template <typename InputIter>
    void add_log_weight (InputIter first)
    {
        weight_set_.add_log_weight(first);
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// adding to the unnormalized logarithm weights with (possible
    /// unormalized) logarithm incremental weights through a ranodm access
    /// iterator with (possible non-uniform) stride
    template <typename RandomIter>
    void add_log_weight (RandomIter first, int stride)
    {
        weight_set_.add_log_weight(first, stride);
    }

    /// \brief Get the ESS of the particle collection based on the current
    /// weights
    double ess () const
    {
        return weight_set_.ess();
    }

    /// \brief Get an RNG stream for a given particle
    rng_type &rng (size_type id)
    {
        return rng_set_.rng(id);
    }

    /// \brief Performing resampling if ESS/N < threshold
    ///
    /// \param threshold The threshold of ESS/N below which resampling will be
    /// performed
    ///
    /// \return true if resampling was performed
    bool resample (double threshold)
    {
        resampled_ = weight_set_.ess() < threshold * size_;
        if (resampled_) {
            weight_set_.read_weight(&weight_[0]);
            resample_op_(size_, resample_rng_set_,
                    &weight_[0], &replication_[0]);
            resample_do();
        }

        return resampled_;
    }

    /// \brief Whether last attempt of resample is actually performed
    bool resampled () const
    {
        return resampled_;
    }

    /// \brief Set resampling method by a resample_op_type object
    void resample_scheme (const resample_op_type &res_op)
    {
        resample_op_ = res_op;
    }

    /// \brief Set resampling method by a built-in scheme name
    ///
    /// \param scheme A ResampleScheme scheme name
    ///
    /// \return  true if scheme is valid and the resampling scheme is actually
    /// changed, false otherwise
    bool resample_scheme (ResampleScheme scheme)
    {
        switch (scheme) {
            case Multinomial :
                resample_op_ =
                    Resample<ResampleType<ResampleScheme, Multinomial>,
                    size_type, resample_rng_set_type>();
                break;
            case Residual :
                resample_op_ =
                    Resample<ResampleType<ResampleScheme, Residual>,
                    size_type, resample_rng_set_type>();
                break;
            case Stratified :
                resample_op_ =
                    Resample<ResampleType<ResampleScheme, Stratified>,
                    size_type, resample_rng_set_type>();
                break;
            case Systematic :
                resample_op_ =
                    Resample<ResampleType<ResampleScheme, Systematic>,
                    size_type, resample_rng_set_type>();
                break;
            case ResidualStratified :
                resample_op_ =
                    Resample<ResampleType<ResampleScheme, ResidualStratified>,
                    size_type, resample_rng_set_type>();
                break;
            case ResidualSystematic :
                resample_op_ =
                    Resample<ResampleType<ResampleScheme, ResidualSystematic>,
                    size_type, resample_rng_set_type>();
                break;
            default :
                return false;
                break;
        }

        return true;
    }

    /// \brief Set resampling method by a scheme name from a collection
    ///
    /// \details
    /// An object of type Resample<ResampleType<EnumType, S>, size_type,
    /// resample_rng_set_type> will constructed as the resampling method. This
    /// can be a user defined partial specializing of Resample clas template
    ///
    /// For example, resample_scheme<ResampleScheme, Stratified>() is
    /// equivalent to resample_scheme(Stratified)
    template <typename EnumType, EnumType S>
    void resample_scheme ()
    {
        resample_scheme<ResampleType<EnumType, S> >();
    }

    /// \brief Set resampling method by the type of resampling object
    ///
    /// \details
    /// An object of type Resample<ResType, size_type, resample_rng_set_type>,
    /// will constructed as the resampling method. This can be a user defined
    /// partial specializing of Resample class template
    template <typename ResType>
    void resample_scheme ()
    {
        resample_op_ = Resample<ResType, size_type, resample_rng_set_type>();
    }

    private :

    size_type size_;
    value_type value_;
    weight_set_type weight_set_;
    rng_set_type rng_set_;

    std::vector<size_type> replication_;
    std::vector<size_type> copy_from_;
    std::vector<double> weight_;
    bool resampled_;
    resample_op_type resample_op_;
    resample_rng_set_type resample_rng_set_;
    std::vector<SingleParticle<value_type> > sp_;
    std::vector<ConstSingleParticle<value_type> > csp_;

    void resample_do ()
    {
        size_type sum = std::accumulate(
                replication_.begin(), replication_.end(),
                static_cast<size_type>(0));
        if (sum != size_) {
            typename std::vector<size_type>::iterator id_max =
                std::max_element(replication_.begin(), replication_.end());
            *id_max += size_ - sum;
        }

        size_type from = 0;
        size_type time = 0;
        for (size_type to = 0; to != size_; ++to) {
            if (replication_[to]) {
                copy_from_[to] = to;
            } else {
                // replication_[to] has zero child, copy from elsewhere
                if (replication_[from] - time <= 1) {
                    // only 1 child left on replication_[from]
                    time = 0;
                    do // move from to some position with at least 2 children
                        ++from;
                    while (replication_[from] < 2);
                }
                copy_from_[to] = from;
                ++time;
            }
        }

        value_.copy(&copy_from_[0]);
        weight_set_.set_equal_weight();
    }
}; // class Particle

} // namespace vsmc

#endif // VSMC_CORE_PARTICLE_HPP
