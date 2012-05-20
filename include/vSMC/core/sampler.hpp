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
    typedef std::vector<double> ess_type;

    /// The type of resampling history vector
    typedef std::vector<bool> resampled_type;

    /// The type of accept count history vector
    typedef std::vector<unsigned> accept_type;

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

    /// \brief Reserve space for histories
    ///
    /// \param num Expected iteration number
    void reserve (unsigned num)
    {
        ess_.reserve(num);
        resampled_.reserve(num);
        accept_.reserve(num);

        for (typename std::map<std::string, Monitor<T> >::iterator
                imap = monitor_.begin(); imap != monitor_.end(); ++imap) {
            if (!imap->second.empty())
                imap->second.reserve(num);
        }

        if (!path_.empty())
            path_.reserve(num);
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

        for (typename std::map<std::string, Monitor<T> >::iterator
                imap = monitor_.begin(); imap != monitor_.end(); ++imap)
            imap->second.clear();

        iter_num_ = 0;
        accept_.push_back(init_(particle_, param));
        post_move();
        particle_.reset_zconst();

        initialized_ = true;
    }

    /// \brief Perform iteration
    void iterate ()
    {
        assert(initialized_);
        assert(bool(move_));

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
    void iterate (unsigned n)
    {
        for (unsigned i = 0; i != n; ++i)
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
        assert(bool(integral));

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

        for (unsigned i = 0; i != iter_num_ + 1; ++i) {
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

            for (unsigned m = 0; m != monitor_index_empty.size(); ++m) {
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
    unsigned iter_num_;
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
        particle_.accept(accept_.back());
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
