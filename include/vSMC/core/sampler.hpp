#ifndef V_SMC_CORE_SAMPLER_HPP
#define V_SMC_CORE_SAMPLER_HPP

#include <vSMC/internal/common.hpp>
#include <ostream>
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
    typedef std::vector<std::vector<unsigned> > accept_type;

    /// The type of Monitor map
    typedef std::map<std::string, Monitor<T> > monitor_map_type;

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
        init_(init), move_(move),
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
        ess_.clear();
        resampled_.clear();
        accept_.clear();
        path_.clear();

        for (typename monitor_map_type::iterator
                m = monitor_.begin(); m != monitor_.end(); ++m)
            m->second.clear();

        iter_num_ = 0;
        if (bool(init_)) {
            accept_.push_back(
                    std::vector<unsigned>(1, init_(particle_, param)));
        }
#ifndef NDEBUG
        else {
            std::cerr << "vSMC Warning:" << std::endl;
            std::cerr << "\tSampler::initiliaize" << std::endl;
            std::cerr
                << "\tAttempt Initialization without a callable object"
                << std::endl;
        }
#endif
        post_move();
        particle_.reset_zconst();
    }

    /// \brief Perform iteration
    void iterate ()
    {
        ++iter_num_;
        std::vector<unsigned> acc;
        if (bool(move_)) {
            acc.push_back(move_(iter_num_, particle_));
        }
#ifndef V_SMC_NDEBUG
        else {
            std::cerr << "vSMC Warning:" << std::endl;
            std::cerr << "\tSampler::iterate" << std::endl;
            std::cerr
                << "\tAttempt Move without a callable object"
                << std::endl;
        }
#endif // V_SMC_NDEBUG
        for (typename std::vector<move_type>::iterator
                m = mcmc_.begin(); m != mcmc_.end(); ++m) {
            if (bool(*m)) {
                acc.push_back((*m)(iter_num_, particle_));
            }
#ifndef V_SMC_NDEBUG
            else {
                std::cerr << "vSMC Warning:" << std::endl;
                std::cerr << "\tSampler::iterate" << std::endl;
                std::cerr
                    << "\tAttempt MCMC without a callable object"
                    << std::endl;
            }
#endif // V_SMC_NDEBUG
        }
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

    /// \brief Add a monitor with an evaluation functor
    ///
    /// \param name The name of the monitor
    /// \param dim The dimension of the monitor, i.e., the number of variables
    /// \param eval The functor used to directly evaluate the results
    /// \param direct Whether or not eval return the integrands or the final
    void monitor (const std::string &name, unsigned dim,
            const typename Monitor<T>::eval_type &eval, bool direct = false)
    {
        monitor_.insert(std::make_pair(name, Monitor<T>(dim, eval, direct)));
        monitor_name_.insert(name);
    }

    /// \brief Read only access to a named monitor through iterator
    ///
    /// \param name The name of the monitor
    ///
    /// \return An const_iterator point to the monitor for the given name
    typename monitor_map_type::const_iterator monitor (
            const std::string &name) const
    {
        return monitor_.find(name);
    }

    /// \brief Read and write access to a named monitor through iterator
    ///
    /// \param name The name of the monitor
    ///
    /// \return An iterator point to the monitor for the given name
    typename monitor_map_type::iterator monitor (const std::string &name)
    {
        return monitor_.find(name);
    }

    /// \brief Read only access to all monitors
    ///
    /// \return A const reference to monitors
    const monitor_map_type &monitor () const
    {
        return monitor_;
    }

    /// \brief Read and write access to all monitors
    ///
    /// \return A reference to monitors
    monitor_map_type &monitor ()
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

    /// \brief Set the path sampling evaluation functor
    ///
    /// \param eval The functor used to compute the integrands or results
    /// \param direct Whether or not eval return the integrands or the final
    /// results
    void path_sampling (const typename Path<T>::eval_type &eval,
            bool direct = false)
    {
        path_.eval(eval, direct);
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
    template<typename CharT, typename Traits>
    void print (std::basic_ostream<CharT, Traits> &os = std::cout,
            bool print_header = true) const
    {
        const char sep = '\t';

        // Accept count
        Eigen::MatrixXd acc;
        unsigned accd = 0;
        for (accept_type::const_iterator
                a = accept_.begin(); a != accept_.end(); ++a) {
            if (a->size() > accd)
                accd = a->size();
        }
        bool print_accept = accd > 0 && iter_size() > 0;
        if (print_accept) {
            acc.resize(iter_size(), accd);
            acc.setConstant(0);
            double accdnorm = static_cast<double>(size());
            for (unsigned r = 0; r != iter_size(); ++r)
                for (unsigned c = 0; c != accept_[r].size(); ++c)
                    acc(r, c) = accept_[r][c] / accdnorm;
        }

        // Path sampling
        Eigen::MatrixXd pa;
        Eigen::MatrixXi pmask;
        bool print_path = path_.iter_size() > 0 && iter_size() > 0;
        if (print_path) {
            pa.resize(iter_size(), 3);
            pmask.resize(iter_size(), 1);
            pmask.setConstant(0);
            for (unsigned d = 0; d != path_.iter_size(); ++d) {
                unsigned pr = path_.index()[d];
                pa(pr, 0) = path_.integrand()[d];
                pa(pr, 1) = path_.width()[d];
                pa(pr, 2) = path_.grid()[d];
                pmask(pr) = 1;
            }
        }

        // Monitors
        Eigen::MatrixXd mon;
        Eigen::MatrixXi mmask;
        unsigned mond = 0;
        for (typename monitor_map_type::const_iterator
                m = monitor_.begin(); m != monitor_.end(); ++m) {
            mond += m->second.dim();
        }
        bool print_monitor = mond > 0 && iter_size() > 0;
        if (print_monitor) {
            mon.resize(iter_size(), mond);
            mmask.resize(iter_size(), monitor_.size());
            mmask.setConstant(0);
            unsigned mc = 0;
            unsigned mn = 0;
            for (typename monitor_map_type::const_iterator
                    m = monitor_.begin(); m != monitor_.end(); ++m) {
                unsigned md = m->second.dim();
                for (unsigned d = 0; d != m->second.iter_size(); ++d) {
                    unsigned mr = m->second.index()[d];
                    for (unsigned c = 0; c != md; ++c)
                        mon(mr, c + mc) = m->second.record(c)[d];
                    mmask(mr, mn) = 1;
                }
                mc += md;
                ++mn;
            }
        }

        // Print header
        if (print_header) {
            os << "Iter" << sep << "ESS" << sep << "ResSam" << sep;
            if (print_accept) {
                if (accd == 1) {
                    os << "Accept" << sep;
                } else {
                    for (unsigned d = 0; d != accd; ++d)
                        os << "Accept." << d + 1 << sep;
                }
            }
            if (print_path) {
                os
                    << "Path.Integrand" << sep
                    << "Path.Width" << sep
                    << "Path.Grid" << sep << "";
            }
            if (print_monitor) {
                for (typename monitor_map_type::const_iterator
                        m = monitor_.begin(); m != monitor_.end(); ++m) {
                    if (m->second.dim() == 1) {
                        os << m->first << sep;
                    } else {
                        for (unsigned d = 0; d != m->second.dim(); ++d)
                            os << m->first << '.' << d + 1 << sep;
                    }
                }
            }
            os << '\n';
        }

        // Print data
        for (unsigned iter = 0; iter != iter_size(); ++iter) {
            os << iter << sep;
            os << ess_[iter] / size() << sep;
            os << resampled_[iter] << sep;
            if (print_accept)
                os << acc.row(iter) << sep;
            if (print_path) {
                if (pmask(iter))
                    os << pa.row(iter) << sep;
                else
                    os << '.' << sep << '.' << sep << '.' << sep;
            }
            if (print_monitor) {
                unsigned mc = 0;
                unsigned mn = 0;
                for (typename monitor_map_type::const_iterator
                        m = monitor_.begin(); m != monitor_.end(); ++m) {
                    unsigned md = m->second.dim();
                    if (mmask(iter, mn)) {
                        os << mon.block(iter, mc, 1, md) << sep;
                    } else {
                        for (unsigned m = 0; m != md; ++m)
                            os << '.' << sep;
                    }
                    mc += md;
                    ++mn;
                }
            }
            os << '\n';
        }
    }

    private :

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
    monitor_map_type monitor_;
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

        for (typename monitor_map_type::iterator
                m = monitor_.begin(); m != monitor_.end(); ++m) {
            if (!m->second.empty())
                m->second.eval(iter_num_, particle_);
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
template<typename T, typename CharT, typename Traits>
std::basic_ostream<CharT, Traits> &operator<< (
        std::basic_ostream<CharT, Traits> &os, const vSMC::Sampler<T> &sampler)
{
    sampler.print(os, true);
    return os;
}

} // namespace std

#endif // V_SMC_CORE_SAMPLER_HPP
