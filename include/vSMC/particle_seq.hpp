#ifndef V_SMC_PARTICLE_SEQ_HPP
#define V_SMC_PARTICLE_SEQ_HPP

namespace vSMC {

enum WeightAction {
    NO_ACTION, SET_WEIGHT, SET_LOG_WEIGHT, MUL_WEIGHT, ADD_LOG_WEIGHT};

template <int Dim, typename T = double>
class ParticleSeq
{
    public :

    typedef T value_type;

    ParticleSeq (std::size_t N) : size_(N), state_(N * Dim) {}

    static int dim ()
    {
        return Dim;
    }

    T *state ()
    {
        return state_.get();
    }

    const T *state () const
    {
        return state_.get();
    }

    template<typename OIter> 
    void state (int id, OIter first) const
    {
        const T *src = state_.get() + id;
        for (std::size_t i = 0; i != size_; ++i, src += Dim)
            *first++ = *src;
    }

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

template <typename T>
class InitializeSeq
{
    public :

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

    virtual int initialize_state (vSMC::Particle<T> &particle,
            typename T::value_type *state, double &weight) = 0;

    virtual void initialize_param (vSMC::Particle<T> &particle,
            void *param) {};

    private :

    vDist::tool::Buffer<double> weight_;
}; // class InitializeSeq

template <typename T>
class MoveSeq
{
    public :

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

    virtual int move_state (std::size_t iter, vSMC::Particle<T> &particle,
            typename T::value_type *state, double &weight) = 0;

    virtual WeightAction weight_action ()
    {
        return ADD_LOG_WEIGHT;
    }

    private :

    vDist::tool::Buffer<double> weight_;
}; // class MoveSeq

} // namespace vSMC

#endif // V_SMC_PARTICLE_SEQ_HPP
