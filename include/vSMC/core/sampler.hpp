#ifndef V_SMC_CORE_SAMPLER_HPP
#define V_SMC_CORE_SAMPLER_HPP

#include <vSMC/internal/common.hpp>
#include <iostream>
#include <map>
#include <set>
#include <string>

namespace vSMC {

/// \brief SMC Sampler
///
/// \tparam T State state type. Requiment:
/// \li Consturctor: T (IntType N)
/// \li Method: copy (IntType from, IntType to)
template <typename T>
class Sampler
{
    public :

    /// The type of initialization functor
    typedef internal::function<unsigned (Particle<T> &, void *)>
        initialize_type;

    /// The type of move and mcmc functor
    typedef internal::function<unsigned (unsigned, Particle<T> &)>
        move_type;

    /// The type of ESS history vector
    typedef std::deque<double> ess_type;

    /// The type of resampling history vector
    typedef std::deque<bool> resampled_type;

    /// The type of accept count history vector
    typedef std::deque<std::deque<unsigned> > accept_type;

    /// The type of Monitor map
    typedef std::map<std::string, Monitor<T> > monitor_map;

    /// \brief Construct a sampler with given number of particles
    ///
    /// \param N The number of particles
    /// \param init The functor used to initialize the particles
    /// \param move The functor used to move the particles and weights
    /// \param scheme The resampling scheme. See ResampleScheme
    /// \param threshold The threshold for performing resampling
    /// \param seed The seed to the parallel RNG system
    explicit Sampler (
            typename Particle<T>::size_type N,
            const initialize_type &init = NULL,
            const move_type &move = NULL,
            ResampleScheme scheme = STRATIFIED,
            double threshold = 0.5,
            typename Particle<T>::seed_type seed = V_SMC_CRNG_SEED) :
        initialized_(false), init_(init), move_(move),
        scheme_(scheme), threshold_(threshold),
        particle_(N, seed), iter_num_(0) {}

    /// \brief Size of the particle set
    ///
    /// \return The number of particles
    typename Particle<T>::size_type size () const
    {
        return particle_.size();
    }

    /// \brief The number of iterations recorded
    ///
    /// \return The number of iterations recorded so far (including the
    /// initialization)
    unsigned iter_size () const
    {
        return ess_.size();
    }

    /// \brief The number of iterations
    ///
    /// \return The number of iterations performed by iterate() so far (not
    /// including the initialization)
    unsigned iter_num () const
    {
        return iter_num_;
    }

    /// \brief Get the current resampling scheme
    ///
    /// \return The current resampling scheme
    ResampleScheme resample_scheme () const
    {
        return scheme_;
    }

    /// \brief Set new resampling scheme
    ///
    /// \param scheme The new scheme for resampling
    void resample_scheme (ResampleScheme scheme)
    {
        scheme_ = scheme;
    }

    /// \brief Get the current threshold
    ///
    /// \return The current threshold for resmapling
    double resample_threshold () const
    {
        return threshold_;
    }

    /// \brief Set new resampling threshold
    ///
    /// \param threshold The new threshold for resampling
    void resample_threshold (double threshold)
    {
        threshold_ = threshold;
    }

    /// \brief ESS history
    ///
    /// \return A const reference to the history of ESS
    const ess_type &ess () const
    {
        return ess_;
    }

    /// \brief Resampling history
    ///
    /// \return A const reference to the history of resampling
    const resampled_type &resampled () const
    {
        return resampled_;
    }

    /// \brief Accept count history
    ///
    /// \return A const reference to the history of accept count
    const accept_type &accept () const
    {
        return accept_;
    }

    /// \brief Read and write access to the particle set
    ///
    /// \return A reference to the latest particle set
    Particle<T> &particle ()
    {
        return particle_;
    }

    /// \brief Read only access to the particle set
    ///
    /// \return A const reference to the latest particle set.
    const Particle<T> &particle () const
    {
        return particle_;
    }

    /// \brief Replace initialization functor
    ///
    /// \param new_init New Initialization functor
    void init (const initialize_type &new_init)
    {
        init_ = new_init;
    }

    /// \brief Replace iteration functor
    ///
    /// \param new_move New Move functor
    void move (const move_type &new_move)
    {
        move_ = new_move;
    }

    /// \brief Replace iteration functor
    ///
    /// \param new_mcmc New MCMC Move functor
    void mcmc (const move_type &new_mcmc)
    {
        mcmc_.push_back(new_mcmc);
    }

    /// \brief Clear all MCMC moves
    void clear_mcmc ()
    {
        mcmc_.clear();
    }

    /// \brief Initialize the particle set
    ///
    /// \param param Additional parameters passed to the initialization
    /// functor
    void initialize (void *param = NULL)
    {
        assert(bool(init_));

        ess_.clear();
        resampled_.clear();
        accept_.clear();
        path_.clear();

        for (typename monitor_map::iterator miter = monitor_.begin();
                miter != monitor_.end(); ++miter)
            miter->second.clear();

        iter_num_ = 0;
        accept_.push_back(std::deque<unsigned>(1, init_(particle_, param)));
        post_move();
        particle_.reset_zconst();

        initialized_ = true;
    }

    /// \brief Perform iteration
    void iterate ()
    {
        ++iter_num_;
        std::deque<unsigned> acc;
        if (bool(move_))
            acc.push_back(move_(iter_num_, particle_));
        for (typename std::deque<move_type>::iterator miter = mcmc_.begin();
                miter != mcmc_.end(); ++miter)
            acc.push_back((*miter)(iter_num_, particle_));
        accept_.push_back(acc);
        post_move();
    }

    /// \brief Perform iteration
    ///
    /// \param n The number of iterations to be performed
    void iterate (unsigned n)
    {
        for (unsigned i = 0; i != n; ++i)
            iterate();
    }

    /// \brief Perform importance sampling integration
    ///
    /// \param dim The dimension of the parameter
    /// \param integral The functor used to compute the integrands
    /// \param res The result, an array of length dim
    void integrate (unsigned dim,
            const typename Monitor<T>::integral_type &integral, double *res)
    {
        assert(bool(integral));
        assert(dim > 0);

        Monitor<T> m(dim, integral);
        m.eval(iter_num_, particle_);
        for (unsigned d = 0; d += dim; ++d)
            res[d] = m.record()[d].back();
    }

    /// \brief Add a monitor, similar to \b monitor in \b BUGS
    ///
    /// \param name The name of the monitor
    /// \param integral The functor used to compute the integrands
    template<typename MonitorIntegralType>
    void monitor (const std::string &name, const MonitorIntegralType &integral)
    {
        monitor_.insert(std::make_pair(
                    name, Monitor<T>(integral.dim(), integral)));
        monitor_name_.insert(name);
    }

    /// \brief Add a monitor, similar to \b monitor in \b BUGS
    ///
    /// \param name The name of the monitor
    /// \param dim The dimension of the monitor
    /// \param integral The functor used to compute the integrands
    void monitor (const std::string &name, unsigned dim,
            const typename Monitor<T>::integral_type &integral)
    {
        monitor_.insert(std::make_pair(name, Monitor<T>(dim, integral)));
        monitor_name_.insert(name);
    }

    /// \brief Read only access to a named monitor through iterator
    ///
    /// \param name The name of the monitor
    ///
    /// \return An const_iterator point to the monitor for the given name
    typename monitor_map::const_iterator monitor (
            const std::string &name) const
    {
        return monitor_.find(name);
    }

    /// \brief Read and write access to a named monitor through iterator
    ///
    /// \param name The name of the monitor
    ///
    /// \return An iterator point to the monitor for the given name
    typename monitor_map::iterator monitor (const std::string &name)
    {
        return monitor_.find(name);
    }

    /// \brief Read only access to all monitors
    ///
    /// \return A const reference to monitors
    const monitor_map &monitor () const
    {
        return monitor_;
    }

    /// \brief Read and write access to all monitors
    ///
    /// \return A reference to monitors
    monitor_map &monitor ()
    {
        return monitor_;
    }

    /// \brief Erase a named monitor
    ///
    /// \param name The name of the monitor
    void clear_monitor (const std::string &name)
    {
        monitor_.erase(name);
        monitor_name_.erase(name);
    }

    /// \brief Erase all monitors
    void clear_monitor ()
    {
        monitor_.clear();
        monitor_name_.clear();
    }

    /// \brief Read only access to the Path sampling monitor
    ///
    /// \return A const reference to the Path sampling monitor
    const Path<T> &path () const
    {
        return path_;
    }

    /// \brief Set the path sampling integral
    ///
    /// \param integral The functor used to compute the integrands
    void path_sampling (const typename Path<T>::integral_type &integral)
    {
        path_.integral(integral);
    }

    /// \brief Path sampling estimate of normalizing constant
    ///
    /// \return The log ratio of normalizing constants
    double path_sampling () const
    {
        return path_.zconst();
    }

    /// \brief SMC estimate of normalizing constant
    ///
    /// \return The log of SMC normalizng constant estimate
    double zconst () const
    {
        return particle_.zconst();
    }

    /// \brief Print the history of the sampler
    ///
    /// \param os The ostream to which the contents are printed
    /// \param print_header Print header if \b true
    void print (std::ostream &os = std::cout, bool print_header = true) const
    {
        // Accept count
        unsigned accept_dim = 0;
        for (accept_type::const_iterator aiter = accept_.begin();
                aiter != accept_.end(); ++aiter)
            if (aiter->size() > accept_dim)
                accept_dim = aiter->size();
        Eigen::MatrixXd accept_data(iter_num_ + 1, accept_dim);
        accept_data.setConstant(0);
        double anorm = static_cast<double>(size());
        for (unsigned r = 0; r != iter_num_ + 1; ++r)
            for (unsigned c = 0; c != accept_[r].size(); ++c)
                accept_data(r, c) = accept_[r][c] / anorm;

        // Path sampling
        Eigen::MatrixXd path_data(iter_num_ + 1, 3);
        Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>
            path_mask(iter_num_ + 1, 1);
        path_mask.setConstant(false);
        for (unsigned d = 0; d != path_.iter_size(); ++d) {
            unsigned prow = path_.index()[d];
            path_data(prow, 0) = path_.integrand()[d];
            path_data(prow, 1) = path_.width()[d];
            path_data(prow, 2) = path_.grid()[d];
            path_mask(prow) = true;
        }

        // Monitors
        unsigned monitor_dim = 0;
        for (typename monitor_map::const_iterator miter = monitor_.begin();
                miter != monitor_.end(); ++miter)
            monitor_dim += miter->second.dim();
        Eigen::MatrixXd monitor_data(iter_num_ + 1, monitor_dim);
        Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>
            monitor_mask(iter_num_ + 1, monitor_.size());
        unsigned mcol = 0;
        unsigned mmask = 0;
        for (typename monitor_map::const_iterator miter = monitor_.begin();
                miter != monitor_.end(); ++miter) {
            unsigned mdim = miter->second.dim();
            for (unsigned d = 0; d != miter->second.iter_size(); ++d) {
                unsigned mrow = miter->second.index()[d];
                for (unsigned c = 0; c != mdim; ++c) {
                    monitor_data(mrow, c + mcol) =
                        miter->second.record()[c][d];
                }
                monitor_mask(mrow, mmask) = true;
            }
            mcol += miter->second.dim();
            ++mmask;
        }

        // Print header
        if (print_header) {
            os << "iter\tESS\tresample\t";
            if (accept_dim == 1) {
                os << "accept\t";
            }
            else {
                for (unsigned d = 0; d != accept_dim; ++d)
                    os << "accept" << d + 1 << '\t';
            }
            os << "path.integrand\tpath.width\tpath.grid\t";
            for (typename monitor_map::const_iterator miter = monitor_.begin();
                    miter != monitor_.end(); ++miter) {
                if (miter->second.dim() == 1) {
                    os << miter->first << '\t';
                } else {
                    for (unsigned d = 0; d != miter->second.dim(); ++d)
                        os << miter->first << d + 1 << '\t';
                }
            }
            os << '\n';
        }

        // Print data
        for (unsigned iter = 0; iter != iter_num_ + 1; ++iter) {
            os << iter << '\t';
            os << ess_[iter] / size() << '\t';
            os << resampled_[iter] << '\t';
            os << accept_data.row(iter) << '\t';
            if (path_mask(iter))
                os << path_data.row(iter) << '\t';
            else
                os << ".\t.\t.\t";
            unsigned mcol = 0;
            unsigned mmask = 0;
            for (typename monitor_map::const_iterator miter = monitor_.begin();
                    miter != monitor_.end(); ++miter) {
                unsigned mdim = miter->second.dim();
                if (monitor_mask(iter, mmask)) {
                    os << monitor_data.block(iter, mcol, 1, mdim) << '\t';
                } else {
                    for (unsigned m = 0; m != mdim; ++m)
                        os << ".\t";
                }
                mcol += miter->second.dim();
                ++mmask;
            }
            os << '\n';
        }
    }

    private :

    /// Initialization indicator
    bool initialized_;

    /// Initialization and movement
    initialize_type init_;
    move_type move_;
    std::deque<move_type> mcmc_;

    /// Resampling
    ResampleScheme scheme_;
    double threshold_;

    /// Particle sets
    Particle<T> particle_;
    unsigned iter_num_;
    ess_type ess_;
    resampled_type resampled_;
    accept_type accept_;

    /// Monte Carlo estimation by integration
    monitor_map monitor_;
    std::set<std::string> monitor_name_;

    /// Path sampling
    Path<T> path_;

    void post_move ()
    {
        bool do_resample = particle_.ess() < threshold_ * size();
        if (do_resample)
            particle_.resample(scheme_);

        if (!path_.empty())
            path_.eval(iter_num_, particle_);

        for (typename std::map<std::string, Monitor<T> >::iterator
                imap = monitor_.begin(); imap != monitor_.end(); ++imap) {
            if (!imap->second.empty())
                imap->second.eval(iter_num_, particle_);
        }

        ess_.push_back(particle_.ess());
        resampled_.push_back(do_resample);
        particle_.resampled(resampled_.back());
        particle_.accept(accept_.back().back());
    }
}; // class Sampler

} // namespace vSMC

namespace std {

/// \brief Print the sampler
///
/// \param os The ostream to which the contents are printed
/// \param sampler The sampler to be printed
///
/// \note This is the same as <tt>sampler.print(os)</tt>
template<typename T>
std::ostream & operator<< (std::ostream &os, const vSMC::Sampler<T> &sampler)
{
    sampler.print(os);

    return os;
}

} // namespace std

#endif // V_SMC_CORE_SAMPLER_HPP
