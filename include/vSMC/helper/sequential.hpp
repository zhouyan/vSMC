#ifndef V_SMC_HELPER_SEQUENTIAL_HPP
#define V_SMC_HELPER_SEQUENTIAL_HPP

#include <vSMC/internal/common.hpp>

namespace vSMC {

/// \brief Sampler::init_type class for helping implementing SMC sequentially
///
/// This is a abstract factory class. Object of its derived type can used as
/// the argument \b init of Sampler constructor. The derived class need to at
/// least define method initialize_state. Optionally, it can also overload the
/// method initialize_param, which will be called before initialization and
/// accept the last argument of Sampler::init_type functor as its parameter.
///
/// \note The template parameter has to be type StateBase or its derived class.
template <typename T>
class InitializeSeq
{
    public :

    typedef internal::function<int (SingleParticle<T>)>
        initialize_state_type;
    typedef internal::function<void (Particle<T> &, void *)>
        initialize_param_type;
    typedef internal::function<void (Particle<T> &)>
        pre_processor_type;
    typedef internal::function<void (Particle<T> &)>
        post_processor_type;

    explicit InitializeSeq (
            initialize_state_type initialize_state = NULL,
            initialize_param_type initialize_param = NULL,
            pre_processor_type    pre_processor    = NULL,
            post_processor_type   post_processor   = NULL) :
        initialize_state_(initialize_state),
        initialize_param_(initialize_param),
        pre_processor_(pre_processor),
        post_processor_(post_processor) {}

    InitializeSeq (const InitializeSeq<T> &init) :
        initialize_state_(init.initialize_state_),
        initialize_param_(init.initialize_param_),
        pre_processor_(init.pre_processor_),
        post_processor_(init.post_processor_) {}

    InitializeSeq<T> & operator= (const InitializeSeq<T> &init)
    {
        if (this != &init) {
            initialize_state_ = init.initialize_state_;
            initialize_param_ = init.initialize_param_;
            pre_processor_ = init.pre_processor_;
            post_processor_ = init.post_processor_;
        }

        return *this;
    }

    /// \brief Operator called by Sampler for initialize the particle set
    ///
    /// \param particle The Particle set passed by Sampler
    /// \param param Additional parameters
    ///
    /// \return Accept count
    virtual std::size_t operator() (Particle<T> &particle, void *param)
    {
        initialize_param(particle, param);
        pre_processor(particle);
        log_weight_.resize(particle.size());
        std::size_t accept = 0;
        for (typename Particle<T>::size_type i = 0;
                i != particle.size(); ++i) {
            accept += initialize_state(SingleParticle<T>(
                        i, log_weight_.data() + i, &particle));
        }
        particle.set_log_weight(log_weight_.data());
        post_processor(particle);

        return accept;
    }

    /// \brief Initialize a single particle
    ///
    /// \param id The id of the particle to be processed
    /// \param log_weight The log weight of the particle
    /// \param particle The Particle set passed by Sampler
    /// \param state The array contains the states of a single particle
    /// \param rng A Boost Random eigen unique to the id'th particle
    ///
    /// \return Accept count, normally should be zero or one
    virtual int initialize_state (SingleParticle<T> part)
    {
        return initialize_state_(part);
    }

    /// \brief Initialize the Particle set
    ///
    /// \param particle The Particle set passed by Sampler
    /// \param param Additional parameter passed by Sampler
    virtual void initialize_param (Particle<T> &particle, void *param)
    {
        if (bool(initialize_param_))
            initialize_param_(particle, param);
    }

    virtual void pre_processor (Particle<T> &particle)
    {
        if (bool(pre_processor_))
            pre_processor_(particle);
    }

    virtual void post_processor (Particle<T> &particle)
    {
        if (bool(post_processor_))
            post_processor_(particle);
    }

    private :

    initialize_state_type initialize_state_;
    initialize_param_type initialize_param_;
    pre_processor_type pre_processor_;
    post_processor_type post_processor_;
    Eigen::VectorXd log_weight_;
}; // class InitializeSeq

/// \brief Sampler::move_type class for helping implementing SMC sequentially
///
/// This is a abstract factory class. Object of its derived type can be used as
/// the argument \b move or \b mcmc of Sampler constructor. The derived class
/// need to at least define method move_state. Optionally, it can also overload
/// the weight_action method to change how the returned weighted should be
/// treated.
///
/// \note The template parameter has to be type StateBase or its derived class.
template <typename T>
class MoveSeq
{
    public :

    typedef internal::function<int (std::size_t, SingleParticle<T>)>
        move_state_type;
    typedef internal::function<WeightAction ()>
        weight_action_type;
    typedef internal::function<void (std::size_t, Particle<T> &)>
        pre_processor_type;
    typedef internal::function<void (std::size_t, Particle<T> &)>
        post_processor_type;

    explicit MoveSeq (
            move_state_type     move_state     = NULL,
            weight_action_type  weight_action  = NULL,
            pre_processor_type  pre_processor  = NULL,
            post_processor_type post_processor = NULL) :
        move_state_(move_state), weight_action_(weight_action),
        pre_processor_(pre_processor), post_processor_(post_processor) {}

    MoveSeq (const MoveSeq<T> &move) :
        move_state_(move.move_state_), weight_action_(move.weight_action_),
        pre_processor_(move.pre_processor_),
        post_processor_(move.post_processor_) {}

    MoveSeq<T> & operator= (const MoveSeq<T> &move)
    {
        if (this != &move) {
            move_state_ = move.move_state_;
            weight_action_ = move.weight_action_;
            pre_processor_ = move.pre_processor_;
            post_processor_ = move.post_processor_;
        }

        return *this;
    }

    /// \brief Operator called by Sampler for move the particle set
    ///
    /// \param iter The iteration number
    /// \param particle The Particle set passed by Sampler
    ///
    /// \return Accept count
    virtual std::size_t operator () (std::size_t iter, Particle<T> &particle)
    {
        pre_processor(iter, particle);
        weight_.resize(particle.size());
        std::size_t accept = 0;
        for (typename Particle<T>::size_type i = 0;
                i != particle.size(); ++i) {
            accept += move_state(iter, SingleParticle<T>(
                        i, weight_.data() + i, &particle));
        }
        set_weight(weight_action(), particle, weight_.data());
        post_processor(iter, particle);

        return accept;
    }

    /// \brief Move a single particle
    ///
    /// \param id The id of the particle to be processed
    /// \param iter The iteration number
    /// \param state The array contains the states of a single particle
    /// \param weight The weight of the particle. This may not be the actual
    /// weight. How the returned weight is treated depend on the return from
    /// another method, weight_action. The default behavior is that it is
    /// treated as the log of the incremental weight. It can also be treated
    /// as multiplier to the original weight, or the acutal value of the (log
    /// of) weight. It can even be meaningless, namely no action is taken with
    /// this weight. See WeightAction and weight_action.
    /// \param particle The Particle set passed by Sampler
    /// \param rng A Boost Random eigen unique to the id'th particle
    virtual int move_state (std::size_t iter, SingleParticle<T> part)
    {
        return move_state_(iter, part);
    }

    /// \brief Determine how weight returned by move_state shall be treated
    ///
    /// \return One of the enumerators of WeightAction
    virtual WeightAction weight_action ()
    {
        if (bool(weight_action_))
            return weight_action_();
        else
            return ADD_LOG_WEIGHT;
    }

    virtual void pre_processor (std::size_t iter, Particle<T> &particle)
    {
        if (bool(pre_processor_))
            pre_processor_(iter, particle);
    }

    virtual void post_processor (std::size_t iter, Particle<T> &particle)
    {
        if (bool(post_processor_))
            post_processor_(iter, particle);
    }

    void set_weight (WeightAction action,
            Particle<T> &particle, double *weight)
    {
        switch (action) {
            case NO_ACTION :
                break;
            case SET_WEIGHT :
                for (typename Particle<T>::size_type i = 0;
                        i != particle.size(); ++i)
                    weight[i] = std::log(weight[i]);
            case SET_LOG_WEIGHT :
                particle.set_log_weight(weight);
                break;
            case MUL_WEIGHT :
                for (typename Particle<T>::size_type i = 0;
                        i != particle.size(); ++i)
                    weight[i] = std::log(weight[i]);
            case ADD_LOG_WEIGHT :
                particle.add_log_weight(weight);
                break;
            default :
                particle.add_log_weight(weight);
                break;
        }
    }

    private :

    move_state_type move_state_;
    weight_action_type weight_action_;
    pre_processor_type pre_processor_;
    post_processor_type post_processor_;
    Eigen::VectorXd weight_;
}; // class MoveSeq

/// \brief Monitor::integral_type class for helping implementing SMC
/// sequentially
///
/// This is a abstract factory class. Object of its derived type can be used
/// as the argument integral of Sampler::monitor(std::string,
/// Monitor<T>::integral_type integral). The derived class need to at least
/// define method monitor_state.
///
/// \note The template parameter has to be type StateBase or its derived class.
template <typename T, unsigned Dim>
class MonitorSeq
{
    public :

    typedef internal::function<void (std::size_t, SingleParticle<T>, double *)>
        monitor_state_type;

    explicit MonitorSeq (monitor_state_type monitor = NULL) :
        monitor_state_(monitor) {}

    /// \brief Operator called by Monitor to record Monte Carlo integration
    ///
    /// \param iter The iteration number
    /// \param particle The Particle set passed by Sampler
    /// \param res The integrands. Sum(res * weight) is the Monte Carlo
    /// integration result.
    virtual void operator () (std::size_t iter, Particle<T> &particle,
            double *res)
    {
        for (typename Particle<T>::size_type i = 0;
                i != particle.size(); ++i) {
            monitor_state(iter, SingleParticle<T>(i, NULL, &particle),
                    res + i * dim());
        }
    }

    /// \brief Record the integrand from a single particle
    ///
    /// \param id The id of the particle to be processed
    /// \param iter The iteration number
    /// \param state The array contains the states of a single particle
    /// \param particle The Particle set passed by Sampler
    /// \param res The integrands
    ///
    /// \return The value to be estimated
    virtual void monitor_state (std::size_t iter, SingleParticle<T> part,
            double *res)
    {
        monitor_state_(iter, part, res);
    }

    /// \brief The dimension of the Monitor
    ///
    /// \return The dimension of res arugment in monitor_state
    static unsigned dim ()
    {
        return Dim;
    }

    private :

    monitor_state_type monitor_state_;
}; // class MonitorSeq

/// \brief Path::integral_type class for helping implementing SMC sequentially
///
/// This is a abstract factory class. Object of its derived type can be used
/// as the argument integral of Sampler::path_sampling(Path<T>::integral_type
/// integral). The derived class need to at least define method path_state and
/// width_state.
///
/// \note The template parameter has to be type StateBase or its derived class.
template <typename T>
class PathSeq
{
    public :

    typedef internal::function<double (std::size_t, SingleParticle<T>)>
        path_state_type;
    typedef internal::function<double (std::size_t, const Particle<T> &)>
        width_state_type;

    explicit PathSeq (
            path_state_type path = NULL, width_state_type width = NULL) :
        path_state_(path), width_state_(width) {}

    /// \brief Operator called by Path to record path sampling integrands and
    /// widths
    ///
    /// \param iter The iteration number
    /// \param particle The particle set passed by Sampler
    /// \param res The integrands. Sum(res * weight) is the path
    ///
    /// \return The width
    /// sampling integrand.
    virtual double operator () (std::size_t iter, Particle<T> &particle,
            double *res)
    {
        for (typename Particle<T>::size_type i = 0; i != particle.size(); ++i)
            res[i] = path_state(iter, SingleParticle<T>(i, NULL, &particle));

        return width_state(iter, particle);
    }

    /// \brief Evaluate the path sampling integrand for a single particle
    ///
    /// \param id The id of the particle to be processed
    /// \param iter The iteration number
    /// \param particle The particle set passed by Sampler
    /// \param state The array contains the states of a single particle
    ///
    /// \return The value of the integrand
    virtual double path_state (std::size_t iter, SingleParticle<T> part)
    {
        return path_state_(iter, part);
    }

    /// \brief Evaluate the path sampling width
    ///
    /// \param iter The iteration number
    /// \param particle The particle set passed by Sampler
    ///
    /// \return The value of the width
    virtual double width_state (std::size_t iter, const Particle<T> &particle)
    {
        return width_state_(iter, particle);
    }

    private :

    path_state_type path_state_;
    width_state_type width_state_;
};

} // namespace vSMC

#endif // V_SMC_HELPER_SEQUENTIAL_HPP
