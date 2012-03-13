#ifndef V_SMC_HELPER_SEQUENTIAL_HPP
#define V_SMC_HELPER_SEQUENTIAL_HPP

#include <Eigen/Dense>
#include <vSMC/internal/config.hpp>
#include <vSMC/core/monitor.hpp>
#include <vSMC/core/particle.hpp>
#include <vSMC/core/path.hpp>

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
/// StateSeq or its derived class can be used as the template argument of
/// Particle. It targets the particular problems where the parameters to be
/// sampled can be viewed as a vector of dimension Dim and type T.
template <unsigned Dim, typename T = double>
class StateSeq
{
    public :

    /// The type of parameters
    typedef T value_type;

    /// \brief Construct a StateSeq object with given number of particles
    ///
    /// \param N The number of particles
    StateSeq (std::size_t N) : size_(N), state_(Dim, N) {}

    /// \brief The dimension of the problem
    ///
    /// \return The dimension of the parameter vector
    static unsigned dim ()
    {
        return Dim;
    }

    /// \brief The number of particles
    ///
    /// \return The number of particles in the current particle set
    std::size_t size () const
    {
        return size_;
    }

    /// \brief Read and write access to the array of a single particle states
    ///
    /// \return A pointer to the states of a single particle
    T *state (std::size_t n)
    {
        return state_.col(n).data();
    }

    /// \brief Read only access to the array of a single particle states
    ///
    /// \return A const pointer to the states of a single array particle
    const T *state (std::size_t n) const
    {
        return state_.col(n).data();
    }

    /// \brief Read and write access to the array of all particle states
    ///
    /// \return A pointer to the states of all particles
    ///
    /// \note The array is of row major order. In other words, it is ordered
    /// as the first Dim elements are the states of first particle, the next
    /// Dim elements are the states of the second particle, and so on.
    T *state ()
    {
        return state_.data();
    }

    /// \brief Read only access to the array of all particle states
    ///
    /// \return A const pointer to the states of all particles
    const T *state () const
    {
        return state_.data();
    }

    /// \brief The copy method used by the Sampler
    ///
    /// \param from The index of particle whose state to be copied
    /// \param to The index of particle to which new state to be written
    void copy (std::size_t from, std::size_t to)
    {
	state_.col(to) = state_.col(from);
    }

    private :

    std::size_t size_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> state_;
}; // class StateSeq

/// \brief Sampler::init_type class for helping implementing SMC sequentially
///
/// This is a abstract factory class. Object of its derived type can used as
/// the argument \b init of Sampler constructor. The derived class need to at
/// least define method initialize_state. Optionally, it can also overload the
/// method initialize_param, which will be called before initialization and
/// accept the last argument of Sampler::init_type functor as its parameter.
///
/// \note The template parameter has to be type StateSeq or its derived
/// class.
template <typename T>
class InitializeSeq
{
    public :

    InitializeSeq () {}
    InitializeSeq (const InitializeSeq<T> &init) {}
    InitializeSeq<T> & operator= (const InitializeSeq<T> &init) {return *this;}

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
        weight_.resize(particle.size());
        std::size_t accept = 0;
        for (std::size_t i = 0; i != particle.size(); ++i) {
            accept += initialize_state(i, particle.value().state(i),
                    weight_[i], particle, particle.prng(i));
        }
        particle.set_log_weight(weight_.data());
        post_processor(particle);

        return accept;
    }

    /// \brief Initialize a single particle
    ///
    /// \param id The id of the particle to be processed
    /// \param weight The log weight of the particle
    /// \param particle The Particle set passed by Sampler
    /// \param state The array contains the states of a single particle
    /// \param rng A Boost Random eigen unique to the id'th particle
    ///
    /// \return Accept count, normally should be zero or one
    virtual int initialize_state (std::size_t id,
            typename T::value_type *state, double &weight,
            const Particle<T> &particle, 
            typename Particle<T>::rng_type &rng) = 0;

    /// \brief Initialize the Particle set
    ///
    /// \param particle The Particle set passed by Sampler
    /// \param param Additional parameter passed by Sampler
    virtual void initialize_param (Particle<T> &particle, void *param) {}
    virtual void pre_processor (Particle<T> &particle) {}
    virtual void post_processor (Particle<T> &particle) {}

    private :

    Eigen::VectorXd weight_;
}; // class InitializeSeq

/// \brief Sampler::move_type class for helping implementing SMC sequentially
///
/// This is a abstract factory class. Object of its derived type can be used as
/// the argument \b move or \b mcmc of Sampler constructor. The derived class
/// need to at least define method move_state. Optionally, it can also overload
/// the weight_action method to change how the returned weighted should be
/// treated.
///
/// \note The template parameter has to be type StateSeq or its derived
/// class.
template <typename T>
class MoveSeq
{
    public :

    MoveSeq () {}
    MoveSeq (const MoveSeq<T> &move) {}
    MoveSeq<T> & operator= (const MoveSeq<T> &move) {return *this;}

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
        for (std::size_t i = 0; i != particle.size(); ++i) {
            accept += move_state(i, iter, particle.value().state(i),
                    weight_[i], particle, particle.prng(i));
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
    virtual int move_state (std::size_t id, std::size_t iter,
            typename T::value_type *state, double &weight,
            const Particle<T> &particle,
            typename Particle<T>::rng_type &rng) = 0;

    /// \brief Determine how weight returned by move_state shall be treated
    ///
    /// \return One of the enumerators of WeightAction
    virtual WeightAction weight_action ()
    {
        return ADD_LOG_WEIGHT;
    }

    virtual void pre_processor (std::size_t iter, Particle<T> &particle) {}
    virtual void post_processor (std::size_t iter, Particle<T> &particle) {}

    void set_weight (WeightAction action,
            Particle<T> &particle, double *weight)
    {
        switch (action) {
            case NO_ACTION :
                break;
            case SET_WEIGHT :
                for (std::size_t i = 0; i != particle.size(); ++i)
                    weight[i] = std::log(weight[i]);
            case SET_LOG_WEIGHT :
                particle.set_log_weight(weight);
                break;
            case MUL_WEIGHT :
                for (std::size_t i = 0; i != particle.size(); ++i)
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

    Eigen::VectorXd weight_;
}; // class MoveSeq

/// \brief Monitor::integral_type class for helping implementing SMC
/// sequentially
///
/// This is a abstract factory class. Object of its derived type can be used
/// as the argument integral of Sampler::monitor(std::string,
/// Monitor<T>::integral_type integral). The derived class need to at least
/// define method monitor_state.
template <typename T, unsigned Dim = 1>
class MonitorSeq
{
    public :

    /// \brief Operator called by Monitor to record Monte Carlo integration
    ///
    /// \param iter The iteration number
    /// \param particle The Particle set passed by Sampler
    /// \param res The integrands. Sum(res * weight) is the Monte Carlo
    /// integration result.
    virtual void operator () (std::size_t iter, Particle<T> &particle,
            double *res)
    {
        for (std::size_t i = 0; i != particle.size(); ++i)
            monitor_state(i, iter, particle.value().state(i), particle,
                    res + i * dim());
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
    virtual void monitor_state (std::size_t id, std::size_t iter,
            const typename T::value_type *state,
            const Particle<T> &particle, double *res) = 0;

    /// \brief The dimension of the Monitor
    ///
    /// \return The dimension of res arugment in monitor_state
    static unsigned dim ()
    {
        return Dim;
    }
}; // class MonitorSeq

/// \brief Path::integral_type class for helping implementing SMC sequentially
///
/// This is a abstract factory class. Object of its derived type can be used
/// as the argument integral of Sampler::path_sampling(Path<T>::integral_type
/// integral). The derived class need to at least define method path_state and
/// width_state.
template <typename T>
class PathSeq
{
    public :

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
        for (std::size_t i = 0; i != particle.size(); ++i)
            res[i] = path_state(i, iter, particle.value().state(i), particle);

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
    virtual double path_state (std::size_t id, std::size_t iter,
            const typename T::value_type *state,
            const Particle<T> &particle) = 0;

    /// \brief Evaluate the path sampling width
    ///
    /// \param iter The iteration number
    /// \param particle The particle set passed by Sampler
    ///
    /// \return The value of the width
    virtual double width_state (std::size_t iter,
            const Particle<T> &particle) = 0;
};

} // namespace vSMC

#endif // V_SMC_HELPER_SEQUENTIAL_HPP
