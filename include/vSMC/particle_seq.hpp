#ifndef V_SMC_PARTICLE_SEQ_HPP
#define V_SMC_PARTICLE_SEQ_HPP

namespace vSMC {

enum WeightAction {
    NO_ACTION, SET_WEIGHT, SET_LOG_WEIGHT, MUL_WEIGHT, ADD_LOG_WEIGHT};

template <int dim_>
class ParticleSeq
{
    public :

    ParticleSeq (std::size_t N) : size_(N), state_(N * dim_) {}

    static int dim ()
    {
        return dim_;
    }

    double *state ()
    {
        return state_.get();
    }

    const double *state () const
    {
        return state_.get();
    }

    template<typename OIter> 
    void state (int id, OIter first) const
    {
        const double *src = state_.get() + id;
        for (std::size_t i = 0; i != size_; ++i, src += dim_)
            *first++ = *src;
    }

    void copy (std::size_t from, std::size_t to)
    {
        const double *state_from = state_.get() + from * dim_;
        double *state_to = state_.get() + to * dim_;
        for (int i = 0; i != dim_; ++i, ++state_from, ++state_to)
            *state_to = *state_from;
    }

    private :

    std::size_t size_;
    vDist::tool::Buffer<double> state_;
}; // class ParticleSeq

template <typename T>
class InitializeSeq
{
    public :

    std::size_t operator() (vSMC::Particle<T> &particle, void *param)
    {
        initialize_param(particle, param);

        weight_.resize(particle.size());
        double *state = particle.value().state();
        std::size_t accept = 0;
        for (std::size_t i = 0; i != particle.size();
                ++i, state += T::dim()) {
            accept += initialize_state(particle, state, weight_[i]);
        }

        particle.set_log_weight(weight_);

        return accept;
    }

    virtual int initialize_state (vSMC::Particle<T> &particle,
            double *state, double &weight) = 0;

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
        double *state = particle.value().state();
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
            double *state, double &weight) = 0;

    virtual WeightAction weight_action ()
    {
        return ADD_LOG_WEIGHT;
    }

    private :

    vDist::tool::Buffer<double> weight_;
}; // class MoveSeq

} // namespace vSMC

#endif // V_SMC_PARTICLE_SEQ_HPP
