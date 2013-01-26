#ifndef VSMC_CORE_PATH_HPP
#define VSMC_CORE_PATH_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/utility/integrate.hpp>

namespace vsmc {

/// \brief Monitor for Path sampling
/// \ingroup Core
template <typename T>
class Path
{
    public :

    typedef T value_type;
    typedef cxx11::function<double (
            std::size_t, const Particle<T> &, double *)> eval_type;

    /// \brief Construct a Path with an evaluation object
    ///
    /// \param eval The evaluation object of type Path::eval_type
    ///
    /// A Path object is very similar to a Monitor object. It is a special case
    /// for Path sampling Monitor. The dimension of the Monitor is always one.
    /// In addition, the evaluation object returns the integration grid of the
    /// Path sampling.
    ///
    /// The evaluation object has the signature
    /// \code
    /// double eval (std::size_t iter, const Particle<T> &particle, double *integrand)
    /// \endcode
    /// where the first two arguments are passed in by the Sampler at the end
    /// of each iteration. The evaluation occurs after the possible MCMC moves.
    /// The output parameter `integrand` shall contains the results of the
    /// Path sampling integrands. The return value shall be the Path sampling
    /// integration grid.
    ///
    /// For example, say the Path sampling is computed through integration of
    /// \f$\lambda = \int_0^1 E[g_\alpha(X)]\,\mathrm{d}\alpha\f$. The integral
    /// is approximated with numerical integration at point
    /// \f$\alpha_0 = 0, \alpha_1, \dots, \alpha_T = 1\f$, then at iteration
    /// \f$t\f$, the output parameter `integrand` contains
    /// \f$(g_{\alpha_t}(X_0),\dots)\f$ and the return value is \f$\alpha_t\f$.
    explicit Path (const eval_type &eval) : eval_(eval), recording_(true) {}

    Path (const Path<T> &other) :
        eval_(other.eval_), recording_(other.recording_),
        index_(other.index_), integrand_(other.integrand_), grid_(other.grid_)
    {}

    Path<T> &operator= (const Path<T> &other)
    {
        if (&other != this) {
            eval_      = other.eval_;
            recording_ = other.recording_;
            index_     = other.index_;
            integrand_ = other.integrand_;
            grid_      = other.grid_;
        }

        return *this;
    }

    /// \brief The number of iterations has been recorded
    ///
    /// \sa Monitor::iter_size()
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
    /// \sa Monitor::index()
    std::size_t index (std::size_t iter) const
    {
        VSMC_RUNTIME_ASSERT_ITERATION_NUMBER(Path::index);

        return index_[iter];
    }

    /// \brief Get the Path sampling integrand of a given Path iteration
    double integrand (std::size_t iter) const
    {
        VSMC_RUNTIME_ASSERT_ITERATION_NUMBER(Path::integrand);

        return integrand_[iter];
    }

    /// \brief Get the Path sampling grid value of a given Path iteration
    double grid (std::size_t iter) const
    {
        VSMC_RUNTIME_ASSERT_ITERATION_NUMBER(Path::grid);

        return grid_[iter];
    }

    /// \brief Read the index history through an output iterator
    ///
    /// \sa Monitor::read_index()
    template <typename OutputIter>
    OutputIter read_index (OutputIter first) const
    {
        for (std::size_t i = 0; i != index_.size(); ++i, ++first)
            *first = index_[i];

        return first;
    }

    /// \brief Read the integrand history through an output iterator
    template <typename OutputIter>
    OutputIter read_integrand (OutputIter first) const
    {
        for (std::size_t i = 0; i != integrand_.size(); ++i, ++first)
            *first = integrand_[i];

        return first;
    }

    /// \brief Read the grid history through an output iterator
    template <typename OutputIter>
    OutputIter read_grid (OutputIter first) const
    {
        for (std::size_t i = 0; i != grid_.size(); ++i, ++first)
            *first = grid_[i];

        return first;
    }

    /// \brief Set a new evaluation object of type eval_type
    void set_eval (const eval_type &new_eval)
    {
        eval_ = new_eval;
    }

    /// Perform the evaluation for a given iteration and a Particle<T> object
    ///
    /// \sa Monitor::eval()
    void eval (std::size_t iter, const Particle<T> &particle)
    {
        if (!recording_)
            return;

        VSMC_RUNTIME_ASSERT_FUNCTOR(eval_, Path::eval, EVALUATION);

        buffer_.resize(particle.size());
        weight_.resize(particle.size());
        particle.read_weight(&weight_[0]);

        index_.push_back(iter);
        grid_.push_back(eval_(iter, particle, &buffer_[0]));
        integrand_.push_back(is_int_1_(static_cast<typename
                    traits::SizeTypeTrait<
                    typename traits::ImportanceSampling1TypeTrait<T>::type
                    >::type>(weight_.size()), &buffer_[0], &weight_[0]));
    }

    /// \brief Get the logarithm nomralizing constants ratio estimates
    double zconst () const
    {
        if (iter_size() < 2)
            return 0;

        double integral = 0;
        for (std::size_t i = 1; i != iter_size(); ++i)
            integral += 0.5 * (grid_[i] - grid_[i - 1]) *
                (integrand_[i] + integrand_[i - 1]);

        return integral;
    }

    /// \brief Clear all records of the index and integrations
    void clear ()
    {
        index_.clear();
        integrand_.clear();
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
    std::vector<double> grid_;
    std::vector<double> weight_;
    std::vector<double> buffer_;
    typename traits::ImportanceSampling1TypeTrait<T>::type is_int_1_;
}; // class PathSampling

} // namespace vsmc

#endif // VSMC_CORE_PATH_HPP
