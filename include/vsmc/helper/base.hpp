#ifndef VSMC_HELPER_BASE_HPP
#define VSMC_HELPER_BASE_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/helper/single_particle.hpp>

namespace vsmc { namespace internal {

/// \brief Particle::value_type subtype
/// \ingroup Helper
///
/// \tparam Dim The dimension of the state parameter vector
/// \tparam T The type of the value of the state parameter vector
template <unsigned Dim, typename T>
class StateBase
{
    public :

    /// The type of the number of particles
    typedef VSMC_SIZE_TYPE size_type;

    /// The type of state parameters
    typedef T state_type;

    /// The type of the matrix of states
    typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> state_mat_type;

    /// Construct a StateBase object with given number of particles
    explicit StateBase (size_type N) : size_(N), state_(Dim, N) {}

    virtual ~StateBase () {}

    /// The dimension of the problem
    static unsigned dim ()
    {
        return Dim;
    }

    /// The number of particles
    size_type size () const
    {
        return size_;
    }

    /// \brief Read and write access to a signle particle state
    ///
    /// \param id The position of the particle
    /// \param pos The position of the parameter in the state array
    ///
    /// \return A reference to the parameter at position pos of the states
    /// array of the particle at position id
    state_type &state (size_type id, unsigned pos)
    {
        return state_(pos, id);
    }

    /// Read only access to a signle particle state
    const state_type &state (size_type id, unsigned pos) const
    {
        return state_(pos, id);
    }

    /// \brief Read and write access to the array of a single particle states
    ///
    /// \param id The position of the particle, 0 to size() - 1
    state_type *state (size_type id)
    {
        return state_.col(id).data();
    }

    /// Read only access to the array of a single particle states
    const state_type *state (size_type id) const
    {
        return state_.col(id).data();
    }

    /// \brief Read and write access to the matrix of all particle states
    ///
    /// \note The matrix is of column marjor as it's the default of Eigen.
    /// Therefore state()(pos, id) == state(id, pos). Best avoid this feature.
    state_mat_type &state ()
    {
        return state_;
    }

    /// Read only access to the matrix of all particle states
    const state_mat_type &state () const
    {
        return state_;
    }

    private :

    size_type size_;
    state_mat_type state_;
}; // class StateBase

template <typename T, typename Derived>
class InitializeBase
{

    public :

    unsigned initialize_state (SingleParticle<T> part)
    {
        return initialize_state<Derived>(part, 0);
    }

    void initialize_param (Particle<T> &particle, void *param)
    {
        initialize_param<Derived>(particle, param, 0);
    }

    void post_processor (Particle<T> &particle)
    {
        post_processor<Derived>(particle, 0);
    }

    void pre_processor (Particle<T> &particle)
    {
        pre_processor<Derived>(particle, 0);
    }

    private :

    template <typename D, unsigned (D::*)(SingleParticle<T>)>
    class initialize_state_sfinae_ {};

    template <typename D, void (D::*)(Particle<T> &, void *)>
    class initialize_param_sfinae_ {};

    template <typename D, void (D::*)(Particle<T> &)>
    class processor_sfinae_ {};

    template <typename D>
    unsigned initialize_state (SingleParticle<T> part,
            initialize_state_sfinae_<D, &D::initialize_state> *)
    {
        return static_cast<Derived *>(this)->initialize_state(part);
    }

    template <typename D>
    void initialize_param (Particle<T> &particle, void *param,
            initialize_param_sfinae_<D, &D::initialize_param> *)
    {
        static_cast<Derived *>(this)->initialize_param(particle, param);
    }

    template <typename D>
    void pre_processor (Particle<T> &particle,
            processor_sfinae_<D, &D::pre_processor> *)
    {
        static_cast<Derived *>(this)->pre_processor(particle);
    }


    template <typename D>
    void post_processor (Particle<T> &particle,
            processor_sfinae_<D, &D::post_processor> *)
    {
        static_cast<Derived *>(this)->post_processor(particle);
    }

    template <typename D>
    unsigned initialize_state (SingleParticle<T>, ...) {return 0;}

    template <typename D>
    void initialize_param (Particle<T>, void *, ...) {}

    template <typename D>
    void pre_processor (Particle<T> &, ...) {}

    template <typename D>
    void post_processor (Particle<T> &, ...) {}
}; // class InitializeBase

template <typename T>
class InitializeBase<T, internal::VBase>
{
    public :

    virtual unsigned initialize_state (SingleParticle<T>) {return 0;}
    virtual void initialize_param (Particle<T>, void *) {}
    virtual void post_processor (Particle<T> &) {}
    virtual void pre_processor (Particle<T> &) {}
}; // class InitializeBase<T, internal::VBase>

template <typename T, typename Derived>
class MoveBase
{

    public :

    unsigned move_state (unsigned iter, SingleParticle<T> part)
    {
        return move_state<Derived>(iter, part, 0);
    }

    void post_processor (unsigned iter, Particle<T> &particle)
    {
        post_processor<Derived>(iter, particle, 0);
    }

    void pre_processor (unsigned iter, Particle<T> &particle)
    {
        pre_processor<Derived>(iter, particle, 0);
    }

    private :

    template <typename D, unsigned (D::*)(unsigned, SingleParticle<T>)>
    class move_state_sfinae_ {};

    template <typename D, void (D::*)(unsigned, Particle<T> &)>
    class processor_sfinae_ {};

    template <typename D>
    unsigned move_state (unsigned iter, SingleParticle<T> part,
            move_state_sfinae_<D, &D::move_state> *)
    {
        return static_cast<Derived *>(this)->move_state(iter, part);
    }

    template <typename D>
    void pre_processor (unsigned iter, Particle<T> &particle,
            processor_sfinae_<D, &D::pre_processor> *)
    {
        static_cast<Derived *>(this)->pre_processor(iter, particle);
    }

    template <typename D>
    void post_processor (unsigned iter, Particle<T> &particle,
            processor_sfinae_<D, &D::post_processor> *)
    {
        static_cast<Derived *>(this)->post_processor(iter, particle);
    }

    template <typename D>
    unsigned move_state (unsigned, SingleParticle<T>, ...) {return 0;}

    template <typename D>
    void pre_processor (unsigned iter, Particle<T> &, ...) {}

    template <typename D>
    void post_processor (unsigned iter, Particle<T> &, ...) {}
}; // class MoveBase

template <typename T>
class MoveBase<T, internal::VBase>
{
    public :

    virtual unsigned move_state (unsigned, SingleParticle<T>) {return 0;}
    virtual void post_processor (unsigned, Particle<T> &) {}
    virtual void pre_processor (unsigned, Particle<T> &) {}
}; // class MoveBase<T, internal::VBase>

template <typename T, typename Derived>
class MonitorBase
{
    public :

    void monitor_state (unsigned iter, ConstSingleParticle<T> part,
            double *res)
    {
        monitor_state<Derived>(iter, part, res, 0);
    }

    void pre_processor (unsigned iter, const Particle<T> &particle)
    {
        pre_processor<Derived>(iter, particle);
    }

    void post_processor (unsigned iter, const Particle<T> &particle)
    {
        post_processor<Derived>(iter, particle);
    }

    private :

    template <typename D, void (D::*)(unsigned, ConstSingleParticle<T>,
            double *)>
    class monitor_state_sfinae_ {};

    template <typename D, void (D::*)(unsigned, const Particle<T> &)>
    class processor_sfinae_ {};

    template <typename D>
    void monitor_state (unsigned iter, ConstSingleParticle<T> part,
            double *res, monitor_state_sfinae_<D, &D::monitor_state> *)
    {
        static_cast<Derived *>(this)->monitor_state(iter, part, res);
    }

    template <typename D>
    void pre_processor (unsigned iter, const Particle<T> &particle,
            processor_sfinae_<D, &D::pre_processor> *)
    {
        static_cast<Derived *>(this)->pre_processor(iter, particle);
    }

    template <typename D>
    void post_processor (unsigned iter, const Particle<T> &particle,
            processor_sfinae_<D, &D::post_processor> *)
    {
        static_cast<Derived *>(this)->post_processor(iter, particle);
    }

    template <typename D>
    void monitor_state (unsigned, ConstSingleParticle<T>, double *, ...) {}

    template <typename D>
    void pre_processor (unsigned iter, const Particle<T> &, ...) {}

    template <typename D>
    void post_processor (unsigned iter, const Particle<T> &, ...) {}
}; // class MonitorBase

template <typename T>
class MonitorBase<T, internal::VBase>
{
    public :

    virtual void monitor_state (unsigned, ConstSingleParticle<T>,
            double *) {}
    virtual void post_processor (unsigned, const Particle<T> &) {}
    virtual void pre_processor (unsigned, const Particle<T> &) {}
}; // class MonitorBase<T, internal::VBase>

template <typename T, typename Derived>
class PathBase
{
    public :

    double path_state (unsigned iter, ConstSingleParticle<T> part)
    {
        return path_state<Derived>(iter, part, 0);
    }

    double path_width (unsigned iter, Particle<T> &particle)
    {
        return path_width<Derived>(iter, particle, 0);
    }

    void pre_processor (unsigned iter, const Particle<T> &particle)
    {
        pre_processor<Derived>(iter, particle);
    }

    void post_processor (unsigned iter, const Particle<T> &particle)
    {
        post_processor<Derived>(iter, particle);
    }

    private :

    template <typename D, double (D::*)(unsigned, ConstSingleParticle<T>)>
    class path_state_sfinae_ {};

    template <typename D, double (D::*)(unsigned, const Particle<T> &)>
    class path_width_sfinae_ {};

    template <typename D, void (D::*)(unsigned, const Particle<T> &)>
    class processor_sfinae_ {};

    template <typename D>
    double path_state (unsigned iter, ConstSingleParticle<T> part,
            path_state_sfinae_<D, &D::path_state> *)
    {
        static_cast<Derived *>(this)->path_state(iter, part);
    }

    template <typename D>
    double path_width (unsigned iter, const Particle<T> &particle,
            path_width_sfinae_<D, &D::path_state> *)
    {
        static_cast<Derived *>(this)->path_width(iter, particle);
    }

    template <typename D>
    void pre_processor (unsigned iter, const Particle<T> &particle,
            processor_sfinae_<D, &D::pre_processor> *)
    {
        static_cast<Derived *>(this)->pre_processor(iter, particle);
    }

    template <typename D>
    void post_processor (unsigned iter, const Particle<T> &particle,
            processor_sfinae_<D, &D::post_processor> *)
    {
        static_cast<Derived *>(this)->post_processor(iter, particle);
    }

    template <typename D>
    double path_state (unsigned, ConstSingleParticle<T>, ...) {return 0;}

    template <typename D>
    double path_width (unsigned, const Particle<T> &) {return 0;}

    template <typename D>
    void pre_processor (unsigned iter, const Particle<T> &, ...) {}

    template <typename D>
    void post_processor (unsigned iter, const Particle<T> &, ...) {}
}; // class PathBase

template <typename T>
class PathBase<T, internal::VBase>
{
    public :

    virtual double path_state (unsigned, ConstSingleParticle<T>) {return 0;}
    virtual double path_width (unsigned, const Particle<T> &) {return 0;}
    virtual void post_processor (unsigned, const Particle<T> &) {}
    virtual void pre_processor (unsigned, const Particle<T> &) {}
}; // class MonitorBase<T, internal::VBase>

} } // namespace vsmc::internal

#endif // VSMC_HELPER_BASE_HPP
