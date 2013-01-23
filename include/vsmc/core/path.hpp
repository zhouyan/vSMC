#ifndef VSMC_CORE_PATH_HPP
#define VSMC_CORE_PATH_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/utility/cxxblas.hpp>

namespace vsmc {

/// \brief Monitor for Path sampling
/// \ingroup Core
template <typename T>
class Path
{
    public :

    typedef T value_type;
    typedef typename traits::DDotTypeTrait<T>::type ddot_type;
    typedef cxx11::function<double (
            std::size_t, const Particle<T> &, double *)> eval_type;

    /// \brief Construct a Path with an evaluation object
    ///
    /// \param eval The evaluation object of type Path::eval_type
    ///
    /// A Path object is very similar to a Monitor object. It is a special case
    /// for Path sampling Monitor. The dimension of the Monitor is always one.
    /// In addition, the evaluation object returns the width of the Path
    /// sampling.
    ///
    /// The evaluation object has the signature
    /// \code
    /// double eval (std::size_t iter, const Particle<T> &particle, double *integrand)
    /// \endcode
    /// where the first two arguments are passed in by the Sampler at the end
    /// of each iteration. The evaluation occurs after the possible MCMC moves.
    /// The output parameter `integrand` shall contains the results of the
    /// Path sampling integrands. The return value shall be the Path sampling
    /// width.
    ///
    /// For example, say the Path sampling is computed through integral of
    /// \f$\lambda = \int_0^1 E[g_\alpha(X)]\,\mathrm{d}\alpha\f$. The integral
    /// is approximated with numerical integration at point
    /// \f$\alpha_0 = 0, \alpha_1, \dots, \alpha_T = 1\f$, then at iteration
    /// \f$t\f$, the output parameter `integrand` contains
    /// \f$(g_{\alpha_t}(X_0),\dots)\f$ and the return value is
    /// \f$\alpha_t - \alpha_{t-1}\f$.
    explicit Path (const eval_type &eval) : eval_(eval), recording_(true) {}

    Path (const Path<T> &other) :
        eval_(other.eval_), recording_(other.recording_),
        index_(other.index_), integrand_(other.integrand_),
        width_(other.width_), grid_(other.grid_)
    {}

    Path<T> &operator= (const Path<T> &other)
    {
        if (&other != this) {
            eval_      = other.eval_;
            recording_ = other.recording_;
            index_     = other.index_;
            integrand_ = other.integrand_;
            width_     = other.width_;
            grid_      = other.grid_;
        }

        return *this;
    }

    /// \brief The number of iterations has been recorded
    ///
    /// This is not necessarily the same as Sampler<T>::iter_size. For example,
    /// a Path sampling monitor can be set only after a certain time point of
    /// the sampler's iterations.
    std::size_t iter_size () const
    {
        return index_.size();
    }

    /// \brief Whether the evaluation object is valid
    VSMC_EXPLICIT_OPERATOR operator bool () const
    {
        return bool(eval_);
    }

    /// \brief Get the iteration index of the sampler of a given monitor
    /// iteration
    ///
    /// \details
    /// For example, if a Path sampling monitor is only set at the sampler's
    /// iteration `siter`. Then index(0) will be `siter` and so on. If the Path
    /// sampling monitor is set before the sampler's initialization and
    /// continued to be evaluated during the iterations, then iter(iter) shall
    /// just be `iter`.
    std::size_t index (std::size_t iter) const
    {
        VSMC_RUNTIME_ASSERT((iter >= 0 && iter < iter_size()),
                ("CALL **Path::index** WITH AN INVALID "
                 "ITERATION NUMBER"));

        return index_[iter];
    }

    /// \brief Get the Path sampling integrand of a given Path iteration
    double integrand (std::size_t iter) const
    {
        VSMC_RUNTIME_ASSERT((iter >= 0 && iter < iter_size()),
                ("CALL **Path::integrand** WITH AN INVALID "
                 "ITERATION NUMBER"));

        return integrand_[iter];
    }

    /// \brief Get the Path sampling width of a given Path iteration
    double width (std::size_t iter) const
    {
        VSMC_RUNTIME_ASSERT((iter >= 0 && iter < iter_size()),
                ("CALL **Path::width** WITH AN INVALID "
                 "ITERATION NUMBER"));

        return width_[iter];
    }

    /// \brief Get the Path sampling grid value of a given Path iteration
    ///
    /// \details
    /// This shall be sum of width's from zero up to the given iteration
    double grid (std::size_t iter) const
    {
        VSMC_RUNTIME_ASSERT((iter >= 0 && iter < iter_size()),
                ("CALL **Path::grid** WITH AN INVALID "
                 "ITERATION NUMBER"));

        return grid_[iter];
    }

    /// \brief Read the index history through an output iterator
    template <typename OutputIter>
    OutputIter read_index (OutputIter first) const
    {
        return std::copy(index_.begin(), index_.end(), first);
    }

    /// \brief Read the integrand history through an output iterator
    template <typename OutputIter>
    OutputIter read_integrand (OutputIter first) const
    {
        return std::copy(integrand_.begin(), integrand_.end(), first);
    }

    /// \brief Read the width history through an output iterator
    template <typename OutputIter>
    OutputIter read_width (OutputIter first) const
    {
        return std::copy(width_.begin(), width_.end(), first);
    }

    /// \brief Read the grid history through an output iterator
    template <typename OutputIter>
    OutputIter read_grid (OutputIter first) const
    {
        return std::copy(grid_.begin(), grid_.end(), first);
    }

    /// \brief Set a new evaluation object of type eval_type
    void set_eval (const eval_type &new_eval)
    {
        eval_ = new_eval;
    }

    /// Perform the evaluation for a given iteration and a Particle<T> object
    void eval (std::size_t iter, const Particle<T> &particle)
    {
        VSMC_RUNTIME_ASSERT((bool(eval_)),
                ("CALL **Path::eval** WITH AN INVALID "
                 "EVALUATION FUNCTOR"));

        buffer_.resize(particle.size());
        weight_.resize(particle.size());
        particle.read_weight(&weight_[0]);

        index_.push_back(iter);
        width_.push_back(eval_(iter, particle, &buffer_[0]));
        grid_.push_back(grid_.size() ?
                grid_.back() + width_.back() : width_.back());
        integrand_.push_back(ddot_(static_cast<
                    typename traits::SizeTypeTrait<ddot_type>::type>(
                        weight_.size()), &weight_[0], 1, &buffer_[0], 1));
    }

    /// \brief Get the logarithm nomralizing constants ratio estimates
    double zconst () const
    {
        double sum = 0;
        for (std::size_t i = 1; i != iter_size(); ++i)
            sum += 0.5 * width_[i] * (integrand_[i-1] + integrand_[i]);

        return sum;
    }

    /// \brief Clear all records of the index and integrations
    void clear ()
    {
        index_.clear();
        integrand_.clear();
        width_.clear();
        grid_.clear();
    }

    /// \brief Whether the Path is actively recording restuls
    bool recording () const
    {
        return recording_;
    }

    /// \brief Turn on the recording
    void turnon ()
    {
        recording_ = true;
    }

    /// \brief Turn off the recording
    void turnoff ()
    {
        recording_ = false;
    }

    private :

    eval_type eval_;
    bool recording_;
    std::vector<std::size_t> index_;
    std::vector<double> integrand_;
    std::vector<double> width_;
    std::vector<double> grid_;
    std::vector<double> weight_;
    std::vector<double> buffer_;
    ddot_type ddot_;
}; // class PathSampling

} // namespace vsmc

#endif // VSMC_CORE_PATH_HPP
