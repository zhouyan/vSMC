#ifndef VSMC_HELPER_BASE_HPP
#define VSMC_HELPER_BASE_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/helper/single_particle.hpp>

namespace vsmc {

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
        return initialize_state_dispatch(part, &Derived::initialize_state);
    }

    void initialize_param (Particle<T> &particle, void *param)
    {
        initialize_param_dispatch(particle, param, &Derived::initialize_param);
    }

    void post_processor (Particle<T> &particle)
    {
        post_processor_dispatch(particle, &Derived::pre_processor);
    }

    void pre_processor (Particle<T> &particle)
    {
        pre_processor_dispatch(particle, &Derived::post_processor);
    }

    private :

    template <typename D>
    unsigned initialize_state_dispatch (SingleParticle<T> part,
            unsigned (D::*) (SingleParticle<T>))
    {
        return static_cast<Derived *>(this)->initialize_state(part);
    }

    template <typename D>
    void initialize_param_dispatch (Particle<T> &particle, void *param,
            void (D::*) (Particle<T> &, void *))
    {
        static_cast<Derived *>(this)->initialize_param(particle, param);
    }

    template <typename D>
    void pre_processor_dispatch (Particle<T> &particle,
            void (D::*) (Particle<T> &))
    {
        static_cast<Derived *>(this)->pre_processor(particle);
    }

    template <typename D>
    void post_processor_dispatch (Particle<T> &particle,
            void (D::*) (Particle<T> &))
    {
        static_cast<Derived *>(this)->post_processor(particle);
    }

    unsigned initialize_state_dispatch (SingleParticle<T> part,
            unsigned (InitializeBase::*) (SingleParticle<T>)) {return 0;}

    void initialize_param_dispatch (Particle<T> &particle, void *param,
            void (InitializeBase::*) (Particle<T> &, void *)) {}

    void pre_processor_dispatch (Particle<T> &particle,
            void (InitializeBase::*) (Particle<T> &)) {}

    void post_processor_dispatch (Particle<T> &particle,
            void (InitializeBase::*) (Particle<T> &)) {}
}; // class InitializeBase

template <typename T>
class InitializeBase<T, internal::VirtualDerived>
{
    public :

    virtual unsigned initialize_state (SingleParticle<T>) {return 0;}
    virtual void initialize_param (Particle<T> &, void *) {}
    virtual void post_processor (Particle<T> &) {}
    virtual void pre_processor (Particle<T> &) {}
}; // class InitializeBase<T, internal::VirtualDerived>

template <typename T, typename Derived>
class MoveBase
{
    public :

    unsigned move_state (unsigned iter, SingleParticle<T> part)
    {
        return move_state_dispatch(iter, part, &Derived::move_state);
    }

    void pre_processor (unsigned iter, Particle<T> &particle)
    {
        pre_processor_dispatch(iter, particle, &Derived::pre_processor);
    }

    void post_processor (unsigned iter, Particle<T> &particle)
    {
        post_processor_dispatch(iter, particle, &Derived::post_processor);
    }

    private :

    template <typename D>
    unsigned move_state_dispatch (unsigned iter, SingleParticle<T> part,
            unsigned (D::*) (unsigned, SingleParticle<T>))
    {
        return static_cast<Derived *>(this)->move_state(iter, part);
    }

    template <typename D>
    void pre_processor_dispatch (unsigned iter, Particle<T> &particle,
            void (D::*) (unsigned, Particle<T> &))
    {
        static_cast<Derived *>(this)->pre_processor(iter, particle);
    }

    template <typename D>
    void post_processor_dispatch (unsigned iter, Particle<T> &particle,
            void (D::*) (unsigned, Particle<T> &))
    {
        static_cast<Derived *>(this)->post_processor(iter, particle);
    }

    unsigned move_state_dispatch (unsigned, SingleParticle<T>,
            unsigned (MoveBase::*) (unsigned, SingleParticle<T>)) {return 0;}

    void pre_processor_dispatch (unsigned iter, Particle<T> &,
            void (MoveBase::*) (unsigned, Particle<T> &)) {}

    void post_processor_dispatch (unsigned iter, Particle<T> &,
            void (MoveBase::*) (unsigned, Particle<T> &)) {}
}; // class MoveBase

template <typename T>
class MoveBase<T, internal::VirtualDerived>
{
    public :

    virtual unsigned move_state (unsigned, SingleParticle<T>) {return 0;}
    virtual void post_processor (unsigned, Particle<T> &) {}
    virtual void pre_processor (unsigned, Particle<T> &) {}
}; // class MoveBase<T, internal::VirtualDerived>

template <typename T, unsigned, typename Derived>
class MonitorEvalBase
{
    public :

    void monitor_state (unsigned iter, ConstSingleParticle<T> part,
            double *res)
    {
        monitor_state_dispatch(iter, part, res, &Derived::monitor_state);
    }

    void pre_processor (unsigned iter, const Particle<T> &particle)
    {
        pre_processor_dispatch(iter, particle, &Derived::pre_processor);
    }

    void post_processor (unsigned iter, const Particle<T> &particle)
    {
        post_processor_dispatch(iter, particle, &Derived::post_processor);
    }

    private :

    template <typename D>
    void monitor_state_dispatch (unsigned iter, ConstSingleParticle<T> part,
            double *res,
            void (D::*) (unsigned, ConstSingleParticle<T>, double *))
    {
        static_cast<Derived *>(this)->monitor_state(iter, part, res);
    }

    template <typename D>
    void pre_processor_dispatch (unsigned iter, const Particle<T> &particle,
            void (D::*) (unsigned, const Particle<T> &))
    {
        static_cast<Derived *>(this)->pre_processor(iter, particle);
    }

    template <typename D>
    void post_processor_dispatch (unsigned iter, const Particle<T> &particle,
            void (D::*) (unsigned, const Particle<T> &))
    {
        static_cast<Derived *>(this)->post_processor(iter, particle);
    }

    void monitor_state_dispatch (unsigned, ConstSingleParticle<T>, double *res,
            void (MonitorEvalBase::*)
            (unsigned, ConstSingleParticle<T>, double *)) {}

    void pre_processor_dispatch (unsigned iter, const Particle<T> &,
            void (MonitorEvalBase::*) (unsigned, const Particle<T> &)) {}

    void post_processor_dispatch (unsigned iter, const Particle<T> &,
            void (MonitorEvalBase::*) (unsigned, const Particle<T> &)) {}
}; // class MonitorBase

template <typename T, unsigned Dim>
class MonitorEvalBase<T, Dim, internal::VirtualDerived>
{
    public :

    virtual void monitor_state (unsigned, ConstSingleParticle<T>,
            double *) {}
    virtual void post_processor (unsigned, const Particle<T> &) {}
    virtual void pre_processor (unsigned, const Particle<T> &) {}
}; // class MonitorEvalBase<T, internal::VirtualDerived>

template <typename T, typename Derived>
class PathEvalBase
{
    public :

    double path_state (unsigned iter, ConstSingleParticle<T> part)
    {
        return path_state_dispatch(iter, part, &Derived::path_state);
    }

    double path_width (unsigned iter, Particle<T> &particle)
    {
        return path_width_dispatch(iter, particle, &Derived::path_width);
    }

    void pre_processor (unsigned iter, const Particle<T> &particle)
    {
        pre_processor_dispatch(iter, particle, &Derived::pre_processor);
    }

    void post_processor (unsigned iter, const Particle<T> &particle)
    {
        post_processor_dispatch(iter, particle, &Derived::post_processor);
    }

    private :

    template <typename D>
    double path_state_dispatch (unsigned iter, ConstSingleParticle<T> part,
            double (D::*) (unsigned, ConstSingleParticle<T>))
    {
        return static_cast<Derived *>(this)->path_state(iter, part);
    }

    template <typename D>
    double path_width_dispatch (unsigned iter, const Particle<T> &particle,
            double (D::*) (unsigned, const Particle<T> &))
    {
        return static_cast<Derived *>(this)->path_width(iter, particle);
    }

    template <typename D>
    void pre_processor_dispatch (unsigned iter, const Particle<T> &particle,
            void (D::*) (unsigned, const Particle<T> &))
    {
        static_cast<Derived *>(this)->pre_processor(iter, particle);
    }

    template <typename D>
    void post_processor_dispatch (unsigned iter, const Particle<T> &particle,
            void (D::*) (unsigned, const Particle<T> &))
    {
        static_cast<Derived *>(this)->post_processor(iter, particle);
    }

    double path_state_dispatch (unsigned, ConstSingleParticle<T>,
            double (PathEvalBase::*) (unsigned, ConstSingleParticle<T>))
    {return 0;}

    double path_width_dispatch (unsigned, const Particle<T> &,
            double (PathEvalBase::*) (unsigned, const Particle<T> &))
    {return 0;}

    void pre_processor_dispatch (unsigned iter, const Particle<T> &,
            void (PathEvalBase::*) (unsigned, const Particle<T> &)) {}

    void post_processor_dispatch (unsigned iter, const Particle<T> &,
            void (PathEvalBase::*) (unsigned, const Particle<T> &)) {}
}; // class PathEvalBase

template <typename T>
class PathEvalBase<T, internal::VirtualDerived>
{
    public :

    virtual double path_state (unsigned, ConstSingleParticle<T>) {return 0;}
    virtual double path_width (unsigned, const Particle<T> &) {return 0;}
    virtual void post_processor (unsigned, const Particle<T> &) {}
    virtual void pre_processor (unsigned, const Particle<T> &) {}
}; // class PathEval<T, internal::VirtualDerived>

} // namespace vsmc

#endif // VSMC_HELPER_BASE_HPP
