#ifndef V_SMC_HELPER_SEQUENTIAL_HPP
#define V_SMC_HELPER_SEQUENTIAL_HPP

#include <vSMC/internal/common.hpp>

namespace vSMC {

/// \brief Sampler::init_type subtype
///
/// \tparam T A subtype of StateBase
template <typename T>
class InitializeSeq
{
    public :

    typedef internal::function<unsigned (SingleParticle<T>)>
        initialize_state_type;
    typedef internal::function<void (Particle<T> &, void *)>
        initialize_param_type;
    typedef internal::function<void (Particle<T> &)>
        pre_processor_type;
    typedef internal::function<void (Particle<T> &)>
        post_processor_type;

    InitializeSeq () {}
    InitializeSeq (const InitializeSeq<T> &init) {}
    InitializeSeq<T> & operator= (const InitializeSeq<T> &init) {return *this;}
    virtual ~InitializeSeq () {}

    virtual unsigned operator() (Particle<T> &particle, void *param)
    {
        initialize_param(particle, param);
        pre_processor(particle);
        log_weight_.resize(particle.size());
        unsigned accept = 0;
        for (typename Particle<T>::size_type i = 0;
                i != particle.size(); ++i) {
            accept += initialize_state(SingleParticle<T>(
                        i, log_weight_.data() + i, &particle));
        }
        particle.set_log_weight(log_weight_);
        post_processor(particle);

        return accept;
    }

    virtual unsigned initialize_state (SingleParticle<T> part) = 0;
    virtual void initialize_param (Particle<T> &particle, void *param) {}
    virtual void pre_processor (Particle<T> &particle) {}
    virtual void post_processor (Particle<T> &particle) {}

    private :

    typename Particle<T>::weight_type log_weight_;
}; // class InitializeSeq

/// \brief Sampler::move_type subtype
///
/// \tparam T A subtype of StateBase
template <typename T>
class MoveSeq
{
    public :

    typedef internal::function<unsigned (unsigned, SingleParticle<T>)>
        move_state_type;
    typedef internal::function<WeightAction ()>
        weight_action_type;
    typedef internal::function<void (unsigned, Particle<T> &)>
        pre_processor_type;
    typedef internal::function<void (unsigned, Particle<T> &)>
        post_processor_type;

    MoveSeq () {}
    MoveSeq (const MoveSeq<T> &move) {}
    MoveSeq<T> & operator= (const MoveSeq<T> &move) {return *this;}
    virtual ~MoveSeq () {}

    virtual unsigned operator () (unsigned iter, Particle<T> &particle)
    {
        pre_processor(iter, particle);
        weight_.resize(particle.size());
        unsigned accept = 0;
        for (typename Particle<T>::size_type i = 0;
                i != particle.size(); ++i) {
            accept += move_state(iter, SingleParticle<T>(
                        i, weight_.data() + i, &particle));
        }
        set_weight(weight_action(), particle, weight_);
        post_processor(iter, particle);

        return accept;
    }

    virtual unsigned move_state (unsigned iter, SingleParticle<T> part) = 0;
    virtual void pre_processor (unsigned iter, Particle<T> &particle) {}
    virtual void post_processor (unsigned iter, Particle<T> &particle) {}

    virtual WeightAction weight_action ()
    {
        return ADD_LOG_WEIGHT;
    }


    protected :

    void set_weight (WeightAction action, Particle<T> &particle,
            const typename Particle<T>::weight_type &weight)
    {
        switch (action) {
            case NO_ACTION :
                break;
            case SET_WEIGHT :
                particle.set_log_weight(weight.array().log());
                break;
            case SET_LOG_WEIGHT :
                particle.set_log_weight(weight);
                break;
            case MUL_WEIGHT :
                particle.add_log_weight(weight.array().log());
                break;
            case ADD_LOG_WEIGHT :
                particle.add_log_weight(weight);
                break;
            default :
                particle.add_log_weight(weight);
                break;
        }
    }

    private :

    typename Particle<T>::weight_type weight_;
}; // class MoveSeq

/// \brief Monitor::integral_type subtype
///
/// \tparam T A subtype of StateBase
template <typename T, unsigned Dim>
class MonitorSeq
{
    public :

    typedef internal::function<void (
            unsigned, ConstSingleParticle<T>, double *)> monitor_state_type;

    virtual ~MonitorSeq () {}

    virtual void operator () (unsigned iter, const Particle<T> &particle,
            double *res)
    {
        for (typename Particle<T>::size_type i = 0;
                i != particle.size(); ++i) {
            monitor_state(iter, ConstSingleParticle<T>(i, &particle),
                    res + i * dim());
        }
    }

    virtual void monitor_state (unsigned iter, ConstSingleParticle<T> part,
            double *res) = 0;

    static unsigned dim ()
    {
        return Dim;
    }
}; // class MonitorSeq

/// \brief Path::integral_type subtype
///
/// \tparam T A subtype of StateBase
template <typename T>
class PathSeq
{
    public :

    typedef internal::function<double (unsigned, ConstSingleParticle<T>)>
        path_state_type;
    typedef internal::function<double (unsigned, const Particle<T> &)>
        width_state_type;

    virtual ~PathSeq () {}

    virtual double operator () (unsigned iter, const Particle<T> &particle,
            double *res)
    {
        for (typename Particle<T>::size_type i = 0; i != particle.size(); ++i)
            res[i] = path_state(iter, ConstSingleParticle<T>(i, &particle));

        return width_state(iter, particle);
    }

    virtual double path_state (unsigned iter,
            ConstSingleParticle<T> part) = 0;
    virtual double width_state (unsigned iter,
            const Particle<T> &particle) = 0;
};

} // namespace vSMC

#endif // V_SMC_HELPER_SEQUENTIAL_HPP
