#ifndef VSMC_HELPER_PARALLEL_TBB_HPP
#define VSMC_HELPER_PARALLEL_TBB_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/helper/sequential.hpp>
#include <tbb/tbb.h>

/// \defgroup TBB Intel Threading Buidling Block
/// \ingroup Helper
/// \brief Parallelized samplers with Intel TBB

namespace vsmc {

/// \brief Particle::value_type subtype
/// \ingroup TBB
///
/// \tparam Dim The dimension of the state parameter vector
/// \tparam T The type of the value of the state parameter vector
/// \tparam Profiler class The profiler used for profiling run_parallel().
/// The default is NullProfiler, which does nothing but provide the compatible
/// inteferce. Profiler::start() and Profiler::stop() are called automatically
/// when entering and exiting run_parallel(). This shall provide how much
/// time are spent on the parallel code (plus a small overhead of scheduling).
template <unsigned Dim, typename T, typename Profiler>
class StateTBB : public internal::StateBase<Dim, T>
#if !VSMC_HAS_CXX11_DECLTYPE || !VSMC_HAS_CXX11_AUTO_TYPE
    , public internal::ParallelTag
#endif
{
    public :

    typedef VSMC_SIZE_TYPE size_type;
    typedef T state_type;
    typedef Profiler profiler_type;

    explicit StateTBB (size_type N) :
        internal::StateBase<Dim, T>(N), size_(N), copy_(N) {}

    virtual ~StateTBB () {}

    /// \brief Run a worker in parallel with tbb::parallel_for
    ///
    /// \param work The worker object
    ///
    /// \note The kernel shall implement
    /// \code
    /// void operator () (const tbb::blocked_range<size_type> &range) const
    /// \endcode
    /// where \c size_type is StateTBB::size_type. There are equivalent
    /// typedefs in InitializeTBB etc.
    ///
    /// \note The range form of tbb::parallel_for is used by run_parallel.
    /// This is the most commonly used in the context of SMC. The range is
    /// constructed as <tt>tbb::blocked_range<size_type>(0, size())</tt>. For
    /// more complex parallel patterns, users need to call Intel TBB API
    /// themselves.
    template <typename Work>
    void run_parallel (const Work &work) const
    {
        profiler_.start();
        tbb::parallel_for(tbb::blocked_range<size_type>(0, size_), work);
        profiler_.stop();
    }

    /// \brief Run a worker in parallel with tbb::parallel_for
    ///
    /// \param work The worker object
    /// \param part The affinity partitioner
    template <typename Work>
    void run_parallel (const Work &work, tbb::affinity_partitioner &part) const
    {
        profiler_.start();
        tbb::parallel_for(tbb::blocked_range<size_type>(0, size_), work, part);
        profiler_.stop();
    }

    Profiler &profiler () const
    {
        return profiler_;
    }

    void copy (size_type from, size_type to)
    {
        copy_[to] = from;
    }

    void pre_resampling ()
    {
        for (size_type i = 0; i != this->size(); ++i)
            copy_[i] = i;
    }

    void post_resampling ()
    {
        run_parallel(copy_work_(this, copy_.data()));
    }

    private :

    size_type size_;
    std::vector<size_type> copy_;
    profiler_type profiler_;

    class copy_work_
    {
        public :

        copy_work_ (StateTBB<Dim, T> *state, const size_type *from) :
            state_(state), from_(from) {}

        void operator () (const tbb::blocked_range<size_type> &range) const
        {
            for (size_type i = range.begin(); i != range.end(); ++i) {
                size_type from = from_[i];
                if (from != i)
                    state_->state().col(i) = state_->state().col(from);
            }
        }

        private :

        StateTBB<Dim, T> *const state_;
        const size_type *const from_;
    }; // class work_
}; // class StateTBB

/// \brief Sampler<T>::init_type subtype
/// \ingroup TBB
///
/// \tparam T A subtype of StateBase
template <typename T>
class InitializeTBB
{
    public :

    typedef VSMC_SIZE_TYPE size_type;
    typedef T value_type;

    virtual ~InitializeTBB () {}

    unsigned operator() (Particle<T> &particle, void *param)
    {
        this->initialize_param(particle, param);
        this->pre_processor(particle);
        accept_.resize(particle.size());
        particle.value().run_parallel(work_(this, &particle, accept_.data()));
        this->post_processor(particle);

        return accept_.sum();
    }

    virtual unsigned initialize_state (SingleParticle<T> part) = 0;
    virtual void initialize_param (Particle<T> &particle, void *param) {}
    virtual void pre_processor (Particle<T> &particle) {}
    virtual void post_processor (Particle<T> &particle) {}

    private :

    Eigen::Matrix<unsigned, Eigen::Dynamic, 1> accept_;

    class work_
    {
        public :

        work_ (InitializeTBB<T> *init,
                Particle<T> *particle, unsigned *accept) :
            init_(init), particle_(particle), accept_(accept) {}

        void operator() (const tbb::blocked_range<size_type> &range) const
        {
            for (size_type i = range.begin(); i != range.end(); ++i) {
                unsigned *const acc = accept_;
                Particle<T> *const part = particle_;
                acc[i] = init_->initialize_state(SingleParticle<T>(i, part));
            }
        }

        private :

        InitializeTBB<T> *const init_;
        Particle<T> *const particle_;
        unsigned *const accept_;
    }; // class work_
}; // class InitializeTBB

/// \brief Sampler<T>::move_type subtype
/// \ingroup TBB
///
/// \tparam T A subtype of StateBase
template <typename T>
class MoveTBB
{
    public :

    typedef VSMC_SIZE_TYPE size_type;
    typedef T value_type;

    virtual ~MoveTBB () {}

    unsigned operator() (unsigned iter, Particle<T> &particle)
    {
        this->pre_processor(iter, particle);
        accept_.resize(particle.size());
        particle.value().run_parallel(
                work_(this, iter, &particle, accept_.data()));
        this->post_processor(iter, particle);

        return accept_.sum();
    }

    virtual unsigned move_state (unsigned iter, SingleParticle<T> part) = 0;
    virtual void pre_processor (unsigned iter, Particle<T> &particle) {}
    virtual void post_processor (unsigned iter, Particle<T> &particle) {}

    private :

    Eigen::Matrix<unsigned, Eigen::Dynamic, 1> accept_;

    class work_
    {
        public :

        work_ (MoveTBB<T> *move, unsigned iter,
                Particle<T> *particle, unsigned *accept) :
            move_(move), iter_(iter), particle_(particle), accept_(accept) {}

        void operator() (const tbb::blocked_range<size_type> &range) const
        {
            for (size_type i = range.begin(); i != range.end(); ++i) {
                unsigned *const acc = accept_;
                Particle<T> *const part = particle_;
                acc[i] = move_->move_state(iter_, SingleParticle<T>(i, part));
            }
        }

        private :

        MoveTBB<T> *const move_;
        const unsigned iter_;
        Particle<T> *const particle_;
        unsigned *const accept_;
    }; // class work_
}; // class MoveTBB

/// \brief Monitor<T>::eval_type subtype
/// \ingroup TBB
///
/// \tparam T A subtype of StateBase
/// \tparam Dim The dimension of the monitor
template <typename T, unsigned Dim>
class MonitorTBB
{
    public :

    typedef VSMC_SIZE_TYPE size_type;
    typedef T value_type;

    virtual ~MonitorTBB () {}

    void operator() (unsigned iter, const Particle<T> &particle, double *res)
    {
        this->pre_processor(iter, particle);
        particle.value().run_parallel(work_(this, iter, &particle, res));
        this->post_processor(iter, particle);
    }

    virtual void monitor_state (unsigned iter, ConstSingleParticle<T> part,
            double *res) = 0;
    virtual void pre_processor (unsigned iter, const Particle<T> &particle) {}
    virtual void post_processor (unsigned iter, const Particle<T> &particle) {}

    static unsigned dim ()
    {
        return Dim;
    }

    private :

    class work_
    {
        public :

        work_ (MonitorTBB<T, Dim> *monitor, unsigned iter,
                const Particle<T> *particle, double *res) :
            monitor_(monitor), iter_(iter), particle_(particle), res_(res) {}

        void operator() (const tbb::blocked_range<size_type> &range) const
        {
            for (size_type i = range.begin(); i != range.end(); ++i) {
                double *const r = res_ + i * Dim;
                const Particle<T> *const part = particle_;
                monitor_->monitor_state(iter_,
                        ConstSingleParticle<T>(i, part), r);
            }
        }

        private :

        MonitorTBB<T, Dim> *const monitor_;
        const unsigned iter_;
        const Particle<T> *const particle_;
        double *const res_;
    }; // class work_
}; // class MonitorTBB

/// \brief Path<T>::eval_type subtype
/// \ingroup TBB
///
/// \tparam T A subtype of StateBase
template <typename T>
class PathTBB
{
    public :

    typedef VSMC_SIZE_TYPE size_type;
    typedef T value_type;

    virtual ~PathTBB () {}

    double operator() (unsigned iter, const Particle<T> &particle, double *res)
    {
        this->pre_processor(iter, particle);
        particle.value().run_parallel(work_(this, iter, &particle, res));
        this->post_processor(iter, particle);

        return this->width_state(iter, particle);
    }

    virtual double path_state (unsigned iter,
            ConstSingleParticle<T> part) = 0;
    virtual double width_state (unsigned iter,
            const Particle<T> &particle) = 0;
    virtual void pre_processor (unsigned iter, const Particle<T> &particle) {}
    virtual void post_processor (unsigned iter, const Particle<T> &particle) {}

    private :

    class work_
    {
        public :

        work_ (PathTBB<T> *path, unsigned iter,
                const Particle<T> *particle, double *res) :
            path_(path), iter_(iter), particle_(particle), res_(res) {}

        void operator() (const tbb::blocked_range<size_type> &range) const
        {
            for (size_type i = range.begin(); i != range.end(); ++i) {
                const Particle<T> *const part = particle_;
                res_[i] = path_->path_state(iter_,
                        ConstSingleParticle<T>(i, part));
            }
        }

        private :

        PathTBB<T> *const path_;
        const unsigned iter_;
        const Particle<T> *const particle_;
        double *const res_;
    }; // class work_
}; // PathTBB

} // namespace vsmc

#endif // VSMC_HELPER_PARALLEL_TBB_HPP
