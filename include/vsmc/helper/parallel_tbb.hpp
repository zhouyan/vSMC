#ifndef VSMC_HELPER_PARALLEL_TBB_HPP
#define VSMC_HELPER_PARALLEL_TBB_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/helper/base.hpp>
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
/// \tparam Timer class The timer used for profiling run_parallel().
/// The default is NullTimer, which does nothing but provide the compatible
/// inteferce. Timer::start() and Timer::stop() are called automatically
/// when entering and exiting run_parallel(). This shall provide how much
/// time are spent on the parallel code (plus a small overhead of scheduling).
template <unsigned Dim, typename T, typename Timer>
class StateTBB : public StateBase<Dim, T>
#if !VSMC_HAS_CXX11_DECLTYPE || !VSMC_HAS_CXX11_AUTO_TYPE
    , public internal::PreResamplingTag, public internal::PostResamplingTag
#endif
{
    public :

    typedef VSMC_SIZE_TYPE size_type;
    typedef T state_type;
    typedef Timer timer_type;

    explicit StateTBB (size_type N) :
        StateBase<Dim, T>(N), size_(N), copy_(N) {}

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
        timer_.start();
        tbb::parallel_for(tbb::blocked_range<size_type>(0, size_), work);
        timer_.stop();
    }

    /// \brief Run a worker in parallel with tbb::parallel_for
    ///
    /// \param work The worker object
    /// \param ap The affinity partitioner
    ///
    /// \note This is only useful when the all the data can fit into system
    /// cache and the computation to memeory access ratio is low. This is
    /// usually not common in an SMC sampler. There can be situations that
    /// such operations appears, for example, a simple Monitor. But usually
    /// these operations cost such a small fraction of time of the whole
    /// sampler that we don't find they worth the time of optimization.
    template <typename Work>
    void run_parallel (const Work &work, tbb::affinity_partitioner &ap) const
    {
        timer_.start();
        tbb::parallel_for(tbb::blocked_range<size_type>(0, size_), work, ap);
        timer_.stop();
    }

    const Timer &timer () const
    {
        return timer_;
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
    timer_type timer_;

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
template <typename T, typename Derived>
class InitializeTBB : public InitializeBase<T, Derived>
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

    private :

    Eigen::Matrix<unsigned, Eigen::Dynamic, 1> accept_;

    class work_
    {
        public :

        work_ (InitializeTBB<T, Derived> *init,
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

        InitializeTBB<T, Derived> *const init_;
        Particle<T> *const particle_;
        unsigned *const accept_;
    }; // class work_
}; // class InitializeTBB

/// \brief Sampler<T>::move_type subtype
/// \ingroup TBB
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class MoveTBB : public MoveBase<T, Derived>
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

    private :

    Eigen::Matrix<unsigned, Eigen::Dynamic, 1> accept_;

    class work_
    {
        public :

        work_ (MoveTBB<T, Derived> *move, unsigned iter,
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

        MoveTBB<T, Derived> *const move_;
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
template <typename T, unsigned Dim, typename Derived>
class MonitorEvalTBB : public MonitorEvalBase<T, Dim, Derived>
{
    public :

    typedef VSMC_SIZE_TYPE size_type;
    typedef T value_type;

    virtual ~MonitorEvalTBB () {}

    void operator() (unsigned iter, const Particle<T> &particle, double *res)
    {
        this->pre_processor(iter, particle);
        particle.value().run_parallel(work_(this, iter, &particle, res));
        this->post_processor(iter, particle);
    }

    static unsigned dim ()
    {
        return Dim;
    }

    private :

    class work_
    {
        public :

        work_ (MonitorEvalTBB<T, Dim, Derived> *monitor, unsigned iter,
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

        MonitorEvalTBB<T, Dim, Derived> *const monitor_;
        const unsigned iter_;
        const Particle<T> *const particle_;
        double *const res_;
    }; // class work_
}; // class MonitorEvalTBB

/// \brief Path<T>::eval_type subtype
/// \ingroup TBB
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class PathEvalTBB : public PathEvalBase<T, Derived>
{
    public :

    typedef VSMC_SIZE_TYPE size_type;
    typedef T value_type;

    virtual ~PathEvalTBB () {}

    double operator() (unsigned iter, const Particle<T> &particle, double *res)
    {
        this->pre_processor(iter, particle);
        particle.value().run_parallel(work_(this, iter, &particle, res));
        this->post_processor(iter, particle);

        return this->path_width(iter, particle);
    }

    private :

    class work_
    {
        public :

        work_ (PathEvalTBB<T, Derived> *path, unsigned iter,
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

        PathEvalTBB<T, Derived> *const path_;
        const unsigned iter_;
        const Particle<T> *const particle_;
        double *const res_;
    }; // class work_
}; // PathEvalTBB

} // namespace vsmc

#endif // VSMC_HELPER_PARALLEL_TBB_HPP
