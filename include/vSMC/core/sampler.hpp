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
    typedef internal::function<std::size_t (Particle<T> &, void *)>
        initialize_type;

    /// The type of move functor
    typedef internal::function<std::size_t (std::size_t, Particle<T> &)>
        move_type;

    /// The type ESS history vector
    typedef std::vector<double> ess_type;

    /// The type resampling history vector
    typedef std::vector<bool> resampled_type;

    /// The type accept count history vector
    typedef std::vector<std::size_t> accept_type;

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
        scheme_(scheme), threshold_(threshold * N),
        particle_(N, seed), iter_num_(0)
    {
        particle_.sampler(this);
    }

    /// \brief Size of the particle set
    ///
    /// \return The number of particles
    typename Particle<T>::size_type size () const
    {
        return particle_.size();
    }

    /// \brief Size of records
    ///
    /// \return The number of iterations recorded (including the
    /// initialization step)
    std::size_t iter_size () const
    {
        return ess_.size();
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
        threshold_ = threshold * particle_.size();
    }

    /// \brief ESS
    ///
    /// \return The ESS value of the latest iteration
    ess_type::value_type ess () const
    {
        return ess_.back();
    }

    /// \brief ESS history
    ///
    /// \return A const reference to the history of ESS
    const ess_type &ess_history () const
    {
        return ess_;
    }

    /// \brief Indicator of resampling
    ///
    /// \return A bool value, \b true if the latest iteration was resampled
    resampled_type::value_type resampled () const
    {
        return resampled_.back();
    }

    /// \brief Resampling history
    ///
    /// \return A const reference to the history of resampling
    const resampled_type &resampled_history () const
    {
        return resampled_;
    }

    /// \brief Accept count
    ///
    /// \return The accept count of the latest iteration
    accept_type::value_type accept () const
    {
        return accept_.back();
    }

    /// \brief Accept count history
    ///
    /// \return A const reference to the history of accept count
    const accept_type &accept_history () const
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
    /// \param init New Initialization functor
    void initialize (const initialize_type &init)
    {
        init_ = init;
    }

    /// \brief Initialize the particle set
    ///
    /// \param param Additional parameters passed to the initialization
    /// functor
    void initialize (void *param = NULL)
    {
        ess_.clear();
        resampled_.clear();
        accept_.clear();
        path_.clear();

        for (typename std::map<std::string, Monitor<T> >::iterator
                imap = monitor_.begin(); imap != monitor_.end(); ++imap)
            imap->second.clear();

        iter_num_ = 0;
        accept_.push_back(init_(particle_, param));
        post_move();
        particle_.reset_zconst();

        initialized_ = true;
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

    /// \brief Perform iteration
    void iterate ()
    {
        assert(initialized_);

        ++iter_num_;
        if (bool(move_))
            accept_.push_back(move_(iter_num_, particle_));
        for (typename std::vector<move_type>::iterator miter = mcmc_.begin();
                miter != mcmc_.end(); ++miter)
            accept_.back() = (*miter)(iter_num_, particle_);
        post_move();
    }

    /// \brief Perform iteration
    ///
    /// \param n The number of iterations to be performed
    void iterate (std::size_t n)
    {
        for (std::size_t i = 0; i != n; ++i)
            iterate();
    }

    /// \brief Perform importance sampling integration
    ///
    /// \param integral The functor used to compute the integrands
    /// \param res The result, an array of length dim
    template<typename MonitorType>
    void integrate (const MonitorType &integral, double *res)
    {
        Monitor<T> m(integral.dim(), integral);
        m.eval(iter_num_, particle_);
        Eigen::Map<Eigen::VectorXd> r(res, m.dim());
        r = m.record().back();
    }

    /// \brief Perform importance sampling integration
    ///
    /// \param dim The dimension of the parameter
    /// \param integral The functor used to compute the integrands
    /// \param res The result, an array of length dim
    void integrate (unsigned dim,
            const typename Monitor<T>::integral_type &integral, double *res)
    {
        Monitor<T> m(dim, integral);
        m.eval(iter_num_, particle_);
        Eigen::Map<Eigen::VectorXd> r(res, m.dim());
        r = m.record().back();
    }

    /// \brief Add a monitor, similar to \b monitor in \b BUGS
    ///
    /// \param name The name of the monitor
    /// \param integral The functor used to compute the integrands
    template<typename MonitorType>
    void monitor (const std::string &name, const MonitorType &integral)
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
    typename std::map<std::string, Monitor<T> >::const_iterator
        monitor (const std::string &name) const
    {
        return monitor_.find(name);
    }

    /// \brief Read only access to all monitors
    ///
    /// \return A const reference to monitors
    const std::map<std::string, Monitor<T> > &monitor () const
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
        print(os, print_header, !path_.index().empty(), monitor_name_);
    }

    /// \brief Print the history of the sampler
    ///
    /// \param os The ostream to which the contents are printed
    /// \param print_path Print path sampling history if \b true
    /// \param print_monitor A set of monitor names to be printed
    /// \param print_header Print header if \b true
    void print (std::ostream &os, bool print_header, bool print_path,
            const std::set<std::string> &print_monitor) const
    {
        if (print_header) {
            os << "iter\tESS\tresample\taccept\t";
            if (print_path)
                os << "path.integrand\tpath.width\tpath.grid\t";
        }

        typename Path<T>::index_type::const_iterator iter_path_index
            = path_.index().begin();
        typename Path<T>::integrand_type::const_iterator iter_path_integrand
            = path_.integrand().begin();
        typename Path<T>::width_type::const_iterator iter_path_width
            = path_.width().begin();
        typename Path<T>::grid_type::const_iterator iter_path_grid
            = path_.grid().begin();

        std::vector<bool> monitor_index_empty;
        std::vector<unsigned> monitor_dim;
        std::vector<typename Monitor<T>::index_type::const_iterator>
            iter_monitor_index;
        std::vector<typename Monitor<T>::record_type::const_iterator>
            iter_monitor_record;
        for (typename std::map<std::string, Monitor<T> >::const_iterator
                imap = monitor_.begin(); imap != monitor_.end(); ++imap) {
            if (print_monitor.count(imap->first)) {
                monitor_index_empty.push_back(imap->second.index().empty());
                monitor_dim.push_back(imap->second.dim());
                iter_monitor_index.push_back(imap->second.index().begin());
                iter_monitor_record.push_back(imap->second.record().begin());
                if (print_header) {
                    if (monitor_dim.back() > 1) {
                        for (unsigned d = 0; d != monitor_dim.back(); ++d)
                            os << imap->first << d + 1 << '\t';
                    } else {
                        os << imap->first << '\t';
                    }
                }
            }
        }

        if (print_header)
            os << '\n';

        for (std::size_t i = 0; i != iter_size(); ++i) {
            os
                << i << '\t' << ess_[i] / size()
                << '\t' << resampled_[i]
                << '\t' << static_cast<double>(accept_[i]) / size();

            if (print_path) {
                if (!path_.index().empty() && *iter_path_index == i) {
                    os
                        << '\t' << *iter_path_integrand++
                        << '\t' << *iter_path_width++
                        << '\t' << *iter_path_grid++;
                    ++iter_path_index;
                } else {
                    os << '\t' << '.' << '\t' << '.' << '\t' << '.';
                }
            }

            for (std::size_t m = 0; m != monitor_index_empty.size(); ++m) {
                if (!monitor_index_empty[m] && *iter_monitor_index[m] == i) {
                    for (unsigned d = 0; d != monitor_dim[m]; ++d)
                        os << '\t' << (*iter_monitor_record[m])[d];
                    ++iter_monitor_index[m];
                    ++iter_monitor_record[m];
                } else {
                    for (unsigned d = 0; d != monitor_dim[m]; ++d)
                        os << '\t' << '.';
                }
            }

            if (i != iter_size() - 1)
                os << '\n';
        }
    }

    private :

    /// Initialization indicator
    bool initialized_;

    /// Initialization and movement
    initialize_type init_;
    move_type move_;
    std::vector<move_type> mcmc_;

    /// Resampling
    ResampleScheme scheme_;
    double threshold_;

    /// Particle sets
    Particle<T> particle_;
    std::size_t iter_num_;
    ess_type ess_;
    resampled_type resampled_;
    accept_type accept_;

    /// Monte Carlo estimation by integration
    std::map<std::string, Monitor<T> > monitor_;
    std::set<std::string> monitor_name_;

    /// Path sampling
    Path<T> path_;

    void post_move ()
    {
        ess_.push_back(particle_.ess());
        particle_.resampled(ess_.back() < threshold_);
        resampled_.push_back(particle_.resampled());
        if (particle_.resampled()) {
            particle_.resample(scheme_);
            ess_.back() = particle_.size();
        }

        if (!path_.empty())
            path_.eval(iter_num_, particle_);

        for (typename std::map<std::string, Monitor<T> >::iterator
                imap = monitor_.begin(); imap != monitor_.end(); ++imap) {
            if (!imap->second.empty())
                imap->second.eval(iter_num_, particle_);
        }
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
