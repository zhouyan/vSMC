#ifndef V_SMC_PARTICLE_SEQ_HPP
#define V_SMC_PARTICLE_SEQ_HPP

namespace vSMC {

/// \brief Type of weight returned by MoveSeq::move_state
///
/// \li No_ACTION The weight is discarded without further action
/// \li SET_WEIGHT The weight is set directly as the new weight
/// \li SET_LOG_WEIGHT The weight is set direclty as the new log weight
/// \li MUL_WEIGHT The weight is the incremental weight
/// \li ADD_LOG_WEIGHT The weight is the log of the incremental weight
enum WeightAction {
    NO_ACTION, SET_WEIGHT, SET_LOG_WEIGHT, MUL_WEIGHT, ADD_LOG_WEIGHT};

/// \brief Particle type for helping implementing SMC sequentially
///
/// ParticleSeq or its derived class can be used as the template argument of
/// Particle. It targets the particular problems where the parameters to be
/// sampled can be viewed as a vector of dimension Dim and type T.
template <int Dim, typename T = double>
class ParticleSeq
{
    public :

    typedef T value_type;

    /// \brief Construct a ParticleSeq object with given number of particles
    ///
    /// \param N The number of particles
    ParticleSeq (std::size_t N) : size_(N), state_(N * Dim) {}

    /// \brief Dimension of the problem
    /// 
    /// \return The dimension of the parameter vector
    static int dim ()
    {
        return Dim;
    }

    /// \brief Read and write access to the array of particle states
    ///
    /// \return A pointer to the underlying array particle states
    ///
    /// \note The array is of row major order. In other words, it is ordered
    /// as the first Dim elements are the states of first particle, the next
    /// Dim elements are the states of the second particle, and so on.
    T *state ()
    {
        return state_.get();
    }

    /// \brief Read only access to the array of particle states
    ///
    /// \return A const pointer to the underlying array particle states
    const T *state () const
    {
        return state_.get();
    }

    /// \brief Read only access to the states of a particular paramter
    ///
    /// \param id The index, starting from 0, of the paramter
    /// \param first An iterator point to where writing starts
    template<typename OIter>
    void state (int id, OIter first) const
    {
        const T *src = state_.get() + id;
        for (std::size_t i = 0; i != size_; ++i, src += Dim)
            *first++ = *src;
    }

    /// \brief The copy method used by the Sampler
    ///
    /// \param from The index of particle whose state to be copied
    /// \param to The index of particle to which new state to be written
    void copy (std::size_t from, std::size_t to)
    {
        const T *state_from = state_.get() + from * Dim;
        T *state_to = state_.get() + to * Dim;
        for (int i = 0; i != Dim; ++i, ++state_from, ++state_to)
            *state_to = *state_from;
    }

    private :

    std::size_t size_;
    vDist::tool::Buffer<T> state_;
}; // class ParticleSeq

/// \brief Sampler::init_type class for helping implementing SMC sequentially
///
/// This is a abstract factory class. Object of its derived type can used as
/// the argument \b init of Sampler constructor. The derived class need to at
/// least define method initialize_state. Optionally, it can also overload the
/// method initialize_param, which will be called before initialization and
/// accept the last argument of Sampler::init_type functor as its parameter.
///
/// \note The template parameter has to be type ParticleSeq or its derived
/// class.
template <typename T>
class InitializeSeq
{
    public :

    /// \brief Operator called by Sampler for initialize the particle set
    ///
    /// \param particle The Particle set passed by Sampler
    /// \param param Additional parameters
    ///
    /// \return Accept count
    std::size_t operator() (vSMC::Particle<T> &particle, void *param)
    {
        initialize_param(particle, param);

        weight_.resize(particle.size());
        typename T::value_type *state = particle.value().state();
        std::size_t accept = 0;
        for (std::size_t i = 0; i != particle.size();
                ++i, state += T::dim()) {
            accept += initialize_state(particle, state, weight_[i]);
        }

        particle.set_log_weight(weight_);

        return accept;
    }

    /// \brief Initialize a single particle
    ///
    /// \param particle The Particle set passed by Sampler
    /// \param state The array contains the states of a single particle
    /// \param weight The log weight of the particle
    ///
    /// \return Accept count, normally should be zero or one
    virtual int initialize_state (vSMC::Particle<T> &particle,
            typename T::value_type *state, double &weight) = 0;

    /// \brief Initialize the Particle set
    ///
    /// \param particle The Particle set passed by Sampler
    /// \param param Additional parameter passed by Sampler
    virtual void initialize_param (vSMC::Particle<T> &particle,
            void *param) {};

    private :

    vDist::tool::Buffer<double> weight_;
}; // class InitializeSeq

/// \brief Sampler::move_type class for helping implementing SMC sequentially
///
/// This is a abstract factory class. Object of its derived type can be used as
/// the argument \b move or \b mcmc of Sampler constructor. The derived class
/// need to at least define method move_state. Optionally, it can also overload
/// the weight_action method to change how the returned weighted should be
/// treated.
///
/// \note The template parameter has to be type ParticleSeq or its derived
/// class.
template <typename T>
class MoveSeq
{
    public :

    /// \brief Operator called by Sampler for move the particle set
    ///
    /// \param iter The iteration number
    /// \param particle The Particle set passed by Sampler
    ///
    /// \return Accept count
    std::size_t operator () (std::size_t iter, vSMC::Particle<T> &particle)
    {
        weight_.resize(particle.size());
        typename T::value_type *state = particle.value().state();
        std::size_t accept = 0;
        for (std::size_t i = 0; i != particle.size();
                ++i, state += T::dim()) {
            accept += move_state(iter, particle, state, weight_[i]);
        }

        switch (weight_action()) {
            case NO_ACTION :
                break;
            case SET_WEIGHT :
                vdLn(particle.size(), weight_, weight_);
            case SET_LOG_WEIGHT :
                particle.set_log_weight(weight_);
                break;
            case MUL_WEIGHT :
                vdLn(particle.size(), weight_, weight_);
            case ADD_LOG_WEIGHT :
                particle.add_log_weight(weight_);
                break;
            default :
                particle.add_log_weight(weight_);
                break;
        }

        return accept;
    }

    /// \brief Move a single particle
    ///
    /// \param iter The iteration number
    /// \param particle The Particle set passed by Sampler
    /// \param state The array contains the states of a single particle
    /// \param weight The weight of the particle. This may not be the actual
    /// weight. How the returned weight is treated depend on the return from
    /// another method, weight_action. The default behavior is that it is
    /// treated as the log of the incremental weight. It can also be treated
    /// as multiplier to the original weight, or the acutal value of the (log
    /// of) weight. It can even be meaningless, namely no action is taken with
    /// this weight. See WeightAction and weight_action.
    virtual int move_state (std::size_t iter, vSMC::Particle<T> &particle,
            typename T::value_type *state, double &weight) = 0;

    /// \brief Determine how weight returned by move_state shall be treated
    ///
    /// \return One of the enumerators of WeightAction
    virtual WeightAction weight_action ()
    {
        return ADD_LOG_WEIGHT;
    }

    private :

    vDist::tool::Buffer<double> weight_;
}; // class MoveSeq

} // namespace vSMC

#endif // V_SMC_PARTICLE_SEQ_HPP
