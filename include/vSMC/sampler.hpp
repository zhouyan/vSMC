#ifndef V_SMC_SAMPLER_HPP
#define V_SMC_SAMPLER_HPP

#include <map>
#include <stdexcept>
#include <string>
#include <vector>
#include <gsl/gsl_cblas.h>
#include <boost/function.hpp>
#include <vDist/rng/gsl.hpp>
#include <vDist/tool/buffer.hpp>
#include <vSMC/history.hpp>
#include <vSMC/monitor.hpp>
#include <vSMC/particle.hpp>

namespace vSMC {

template <typename T>
class Sampler
{
    public :

    /// The type of particle values
    typedef T value_type;
    /// The type of partiles
    typedef Particle<T> particle_type;
    /// The type of initialize callable objects
    typedef boost::function<std::size_t
        (Particle<T> &, void *)> init_type;
    /// The type of move callable objects
    typedef boost::function<std::size_t
        (std::size_t, Particle<T> &)> move_type;
    /// The type of importance sampling integral
    typedef boost::function<void
        (std::size_t, const Particle<T> &, double *, void *)> integral_type;
    /// The type of path sampling integration
    typedef boost::function<double
        (std::size_t, const Particle<T> &, double *)> path_type;

    /// \brief Sampler does not have a default constructor
    ///
    /// \param N The number of particles
    /// \param init The functor used to initialize the particles
    /// \param move The functor used to move the particles and weights
    /// \param mode The history storage mode. See HistoryMode
    /// \param scheme The resampling scheme. See ResampleScheme
    /// \param seed The seed for the reampling RNG. See documentation of vDist
    /// \param brng The basic RNG for resampling RNG. See documentation of GSL
    Sampler (
            std::size_t N,
            const init_type &init, const move_type &move,
            const typename Particle<T>::copy_type &copy,
            HistoryMode mode = HISTORY_RAM,
            ResampleScheme scheme = RESIDUAL,
            double threshold = 0.5,
            const int seed = V_DIST_SEED,
            const gsl_rng_type *brng = V_DIST_GSL_BRNG) :
        initialized_(false), init_(init), move_(move),
        rng_(seed, brng), scheme_(scheme), threshold_(threshold * N),
        particle_(N, copy), iter_num_(0), history_(mode),
        buffer_(N), path_integral_(NULL) {}

    /// \brief Get ESS
    ///
    /// \return The ESS value of the latest iteration
    double ESS () const
    {
        return ess_.back();
    }

    /// \brief Get all ESS
    ///
    /// \return History of ESS for all iterations
    std::vector<double> ESS_history () const
    {
        return ess_;
    }

    /// \brief Get indicator of resampling
    ///
    /// \return A bool value, \b true if the latest iteration was resampled
    bool was_resampled () const
    {
        return resample_.back();
    }

    /// \brief Get history of resampling
    ///
    /// \return History of resampling for all iterations
    std::vector<bool> was_resampled_history () const
    {
        return resample_;
    }

    /// \brief Get accept count
    ///
    /// \return The accept count of the latest iteration
    std::size_t accept_count () const
    {
        return accept_.back();
    }

    /// \brief Get history of accept count
    ///
    /// \return History of accept count for all iterations
    std::vector<std::size_t> accept_count_history () const
    {
        return accept_;
    }

    /// \brief Read only access to the particle set
    ///
    /// \return A const reference to the latest particle set.
    /// \note Any operations that change the state of the sampler (e.g., an
    /// iteration) may invalidate the reference.
    const Particle<T> &getParticle () const
    {
        return particle_;
    }

    /// \brief (Re)initialize the particle set
    ///
    /// \param param Additional parameters passed to initialization functor,
    /// the default is NULL
    void initialize (void *param = NULL)
    {
        history_.clear();
        ess_.clear();
        resample_.clear();
        accept_.clear();

        for (typename std::map<std::string, Monitor<T> >::iterator
                imap = monitor_.begin(); imap != monitor_.end(); ++imap)
            imap->second.clear();

        path_sample_.clear();
        path_width_.clear();

        iter_num_ = 0;
        accept_.push_back(init_(particle_, param));
        post_move();

        initialized_ = true;
    }

    /// \brief Perform iteration
    void iterate ()
    {
        if (!initialized_)
            throw std::runtime_error(
                    "ERROR: vSMC::Sampler::iterate: "
                    "Sampler has not been initialized yet");

        ++iter_num_;
        accept_.push_back(move_(iter_num_, particle_));
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
    /// \param intgral The functor used to compute the integrands
    double integrate (typename Monitor<T>::integral_type integral) const
    {
        std::size_t n = particle_.size();
        integral(iter_num_, particle_, buffer_);

        return cblas_ddot(n, particle_.get_weight_ptr(), 1, buffer_, 1);
    }

    /// \brief Perform importance sampling integration
    ///
    /// \param intgral The functor used to compute the integrands
    /// \param param Additional parameters to be passed to integral
    double integrate (integral_type integral, void *param) const
    {
        std::size_t n = particle_.size();
        integral(iter_num_, particle_, buffer_, param);

        return cblas_ddot(n, particle_.get_weight_ptr(), 1, buffer_, 1);
    }

    /// \brief Add a monitor, similar to \b monitor in \b BUGS
    ///
    /// \param name The name of the monitor
    /// \param integral The functor used to compute the integrands
    void add_monitor (const std::string &name,
            const typename Monitor<T>::integral_type &integral)
    {
        monitor_.insert(
                typename std::map<std::string, Monitor<T> >::value_type(
                    name, Monitor<T>(particle_.size(), integral)));
    }

    /// \brief Get the iteration index of a monitor
    ///
    /// \param The name of the monitor
    /// \return A vector of the monitor index
    typename Monitor<T>::index_type get_monitor_index (
            const std::string &name) const
    {
        return monitor_.find(name)->second.get_index();
    }

    /// \brief Get the record of Monite Carlo integration of a monitor
    ///
    /// \param name The name of the monitor
    /// \return A vector of the monitor record
    typename Monitor<T>::record_type get_monitor_record (
            const std::string &name) const
    {
        return monitor_.find(name)->second.get_record();
    }

    /// \brief Get both the iteration index and record of a monitor
    typename Monitor<T>::value_type get_monitor_value (
            const std::string &name) const
    {
        return monitor_.find(name)->second.get();
    }

    /// \brief Erase a monitor by name 
    ///
    /// \param name The name of the monitor
    void erase_monitor (const std::string &name)
    {
        monitor_.erase(name);
    }

    /// \brief Clear all monitors
    void clear_monitor ()
    {
        monitor_.clear();
    }

    /// \brief Set the path sampling integral
    ///
    /// \param integral The functor used to compute the integrands
    void set_path_sampling (path_type integral)
    {
        path_integral_ = integral;
    }

    /// \brief Stop path sampling
    void clear_path_sampling ()
    {
        path_integral_ = NULL;
    }

    /// \brief Get the results of path sampling
    double get_path_sampling () const
    {
	std::size_t num = path_sample_.size();
	double sum = 0;
	for (std::size_t i = 1; i != num; ++i)
            sum += (path_sample_[i-1] + path_sample_[i])
                * path_width_[i] * 0.5;
        return sum;
    }

    private :

    /// Initialization indicator
    bool initialized_;

    /// Initialization and movement
    init_type init_;
    move_type move_;

    /// Resampling
    vDist::RngGSL rng_;
    ResampleScheme scheme_;
    double threshold_;

    /// Particle sets
    Particle<T> particle_;
    std::size_t iter_num_;
    std::vector<double> ess_;
    std::vector<bool> resample_;
    std::vector<std::size_t> accept_;

    /// History
    History<T> history_;

    /// Monte Carlo estimation by integration
    mutable vDist::tool::Buffer<double> buffer_;
    std::map<std::string, Monitor<T> > monitor_;

    /// Path sampling
    path_type path_integral_;
    std::vector<double> path_sample_;
    std::vector<double> path_width_;

    void post_move ()
    {
        bool res_indicator = false;
        if (particle_.ESS() < threshold_) {
            res_indicator = true;
            particle_.resample(scheme_, rng_.get_rng());
        }
        ess_.push_back(particle_.ESS());
        resample_.push_back(res_indicator);

        if (history_.mode() != HISTORY_NONE)
            history_.push_back(particle_);

        for (typename std::map<std::string, Monitor<T> >::iterator
                imap = monitor_.begin(); imap != monitor_.end(); ++imap) {
            if (!imap->second.empty())
                imap->second.eval(iter_num_, particle_);
        }

        if (!path_integral_.empty()) {
            double width; 
            path_sample_.push_back(eval_path(width));
            path_width_.push_back(width);
        }
    }

    double eval_path (double &width)
    {
        width = path_integral_(iter_num_, particle_, buffer_);
        return cblas_ddot(particle_.size(),
                particle_.get_weight_ptr(), 1, buffer_, 1);
    }
}; // class Sampler

} // namespace vSMC

#endif // V_SMC_SAMPLER_HPP
