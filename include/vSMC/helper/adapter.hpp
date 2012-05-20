#ifndef V_SMC_HELPER_ADPATER_HPP
#define V_SMC_HELPER_ADPATER_HPP

#include <vSMC/internal/common.hpp>

namespace vSMC {

/// \brief Sampler::init_type subtype
///
/// \tparam Base A subtype of InitializeSeq<T>
template <typename Base>
class InitializeBase : public Base
{
    typedef typename Base::initialize_state_type initialize_state_type;
    typedef typename Base::initialize_param_type initialize_param_type;
    typedef typename Base::pre_processor_type    pre_processor_type;
    typedef typename Base::post_processor_type   post_processor_type;

    explicit InitializeBase (
            const initialize_state_type &init,
            const initialize_param_type &param = NULL,
            const pre_processor_type    &pre = NULL,
            const post_processor_type   &post = NULL)
        initialize_state_(init), initialize_param_(param),
        pre_processor_(pre), post_processor_(post) {}

    unsigned initialize_state (SingleParticle<T> part)
    {
        return initialize_state_(part);
    }

    void initialize_param (Particle<T> &particle, void *param)
    {
        if (bool(initialize_param_))
            initialize_param_(particle, param);
    }

    void pre_processor (Particle<T> &particle)
    {
        if (bool(pre_processor_))
            pre_processor_(particle);
    }

    void post_processor (Particle<T> &particle)
    {
        if (bool(post_processor_))
            post_processor_(particle);
    }

    private :

    initialize_state_type initialize_state_;
    initialize_param_type initialize_param_;
    pre_processor_type    pre_processor_;
    post_processor_type   post_processor_;
}; // class Initialize

/// \brief Sampler::move_type subtype
///
/// \tparam Base A subtype of MoveSeq<T>
template <typename Base>
class MoveBase : public Base
{
    typedef typename Base::move_state_type     move_state_type;
    typedef typename Base::pre_processor_type  pre_processor_type;
    typedef typename Base::post_processor_type post_processor_type;

    explicit MoveBase (
            const move_state_type     &move,
            const pre_processor_type  &pre = NULL,
            const post_processor_type &post = NULL)
        move_state_(move), pre_processor_(pre), post_processor_(post) {}

    unsigned move_state (unsigned iter, SingleParticle<T> part)
    {
        return move_state_(iter, part);
    }

    void pre_processor (unsigned iter, Particle<T> &particle)
    {
        if (bool(pre_processor_))
            pre_processor_(iter, particle);
    }

    void post_processor (unsigned iter, Particle<T> &particle)
    {
        if (bool(post_processor_))
            post_processor_(iter, particle);
    }

    private :

    move_state_type     move_state_;
    pre_processor_type  pre_processor_;
    post_processor_type post_processor_;
}; // class Move

/// \brief Monitor::integral_type subtype
///
/// \tparam Base A subtype of MonitorSeq<T>
template <typename Base>
class MonitorBase : public Base
{
}; // class MonitorBase

/// \brief Path::integral_type subtype
///
/// \tparam Base A subtype of PathSeq<T>
template <typename Base>
class PathBase : public Base
{
}; // class PathBase

} // namespace vSMC

#endif // V_SMC_HELPER_ADPATER_HPP
