#ifndef VSMC_CORE_PATH_HPP
#define VSMC_CORE_PATH_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/utility/integrate.hpp>

#if VSMC_USE_MKL
#include <mkl_vml.h>
#endif

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

    /// \brief Reserve space for a specified number of iterations
    void reserve (std::size_t num)
    {
        index_.reserve(num);
        integrand_.reserve(num);
        grid_.reserve(num);
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

        double *buffer = malloc_eval_integrand(particle.size());
        double *weight = malloc_weight(particle.size());
        particle.read_weight(weight);

        index_.push_back(iter);
        grid_.push_back(eval_(iter, particle, buffer));
        integrand_.push_back(is_int_1_(static_cast<
                    typename traits::SizeTypeTrait<
                    typename traits::ImportanceSampling1TypeTrait<T>::type
                    >::type>(particle.size()), buffer, weight));
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

    protected :

    virtual double *malloc_weight (std::size_t N)
    {
        weight_.resize(N);

        return &weight_[0];
    }

    virtual double *malloc_eval_integrand (std::size_t N)
    {
        buffer_.resize(N);

        return &buffer_[0];
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

/// \brief Monitor for path sampling for SMC with geometry path
/// \ingroup Core
template <typename T>
class PathGeometry : public Path<T>
{
    public :

    typedef cxx11::function<double (
            std::size_t, const Particle<T> &, double *)> eval_type;

    PathGeometry (const eval_type &eval) : Path<T>(eval) {}

    PathGeometry (const PathGeometry<T> &other) :
        weight_history_(other.weight_history_),
        integrand_history_(other.integrand_history_) {}

    PathGeometry<T> &operator= (const PathGeometry<T> &other)
    {
        if (&other != this) {
            weight_history_    = other.weight_history_;
            integrand_history_ = other.integrand_history_;
        }

        return *this;
    }

    template <unsigned Degree>
    double zconst_newton_cotes (unsigned insert_points = 0) const
    {
        if (this->iter_size() < 2)
            return 0;

        integrate::NumericNewtonCotes<Degree> numeric_int;

        if (insert_points == 0) {
            std::vector<double> base_grid(this->iter_size());
            for (std::size_t i = 0; i != this->iter_size(); ++i)
                base_grid[i] = this->grid(i);
            return numeric_int(static_cast<
                    typename integrate::NumericNewtonCotes<Degree>::size_type>(
                        base_grid.size()), &base_grid[0],
                    f_alpha_(*this, weight_history_, integrand_history_));
        }

        std::vector<double> super_grid(insert_points * this->iter_size() -
                insert_points + this->iter_size());
        std::size_t offset = 0;
        for (std::size_t i = 0; i != this->iter_size() - 1; ++i) {
            double a = this->grid(i);
            double b = this->grid(i + 1);
            double h = (b - a) / (insert_points + 1);
            super_grid[offset++] = a;
            for (unsigned j = 0; j != insert_points; ++j)
                super_grid[offset++] = a + h * (j + 1);
        }
        super_grid.back() = this->grid(this->iter_size() - 1);

        return numeric_int(static_cast<
                typename integrate::NumericNewtonCotes<Degree>::size_type>(
                    super_grid.size()), &super_grid[0],
                    f_alpha_(*this, weight_history_, integrand_history_));
    }

    void clear ()
    {
        Path<T>::clear();
        weight_history_.clear();
        integrand_history_.clear();
    }

    protected :

    double *malloc_weight (std::size_t N)
    {
        weight_history_.push_back(std::vector<double>(N));

        return &weight_history_.back()[0];
    }

    double *malloc_eval_integrand (std::size_t N)
    {
        integrand_history_.push_back(std::vector<double>(N));

        return &integrand_history_.back()[0];
    }

    private :

    std::vector<std::vector<double> > weight_history_;
    std::vector<std::vector<double> > integrand_history_;

    class f_alpha_
    {
        public :

        f_alpha_(const PathGeometry<T> &path,
                const std::vector<std::vector<double> > &weight_history,
                const std::vector<std::vector<double> > &integrand_history) :
            path_(path), weight_history_(weight_history),
            integrand_history_(integrand_history) {}

        double operator() (double alpha)
        {
            using std::exp;

            if (path_.iter_size() == 0)
                return 0;

            std::size_t iter = 0;
            while (iter != path_.iter_size() && path_.grid(iter) < alpha)
                ++iter;

            const double tol = alpha > 0 ? 1e-6 * alpha : 1e-10;

            if (iter != path_.iter_size() && path_.grid(iter) - alpha < tol)
                return path_.integrand(iter);

            if (iter == 0)
                return 0;

            --iter;
            if (alpha - path_.grid(iter) < tol)
                return path_.integrand(iter);

            double alpha_inc = alpha - path_.grid(iter);
            std::size_t size = weight_history_[iter].size();
            weight_.resize(size);
            for (std::size_t i = 0; i != size; ++i)
                weight_[i] = alpha_inc * integrand_history_[iter][i];
#if VSMC_USE_MKL
            ::vdExp(static_cast<MKL_INT>(size), &weight_[0], &weight_[0]);
#else
            for (std::size_t i = 0; i != size; ++i)
                weight_[i] = exp(weight_[i]);
#endif
            for (std::size_t i = 0; i != size; ++i)
                weight_[i] *= weight_history_[iter][i];
            double coeff = 0;
            for (std::size_t i = 0; i != size; ++i)
                coeff += weight_[i];
            coeff = 1 / coeff;
            for (std::size_t i = 0; i != size; ++i)
                weight_[i] *= coeff;

            return (is_int_1_(static_cast<
                        typename traits::SizeTypeTrait<
                        typename traits::ImportanceSampling1TypeTrait<T>::type
                        >::type>(size), &integrand_history_[iter][0],
                        &weight_[0]));
        }

        private :

        const PathGeometry<T> &path_;
        const std::vector<std::vector<double> > &weight_history_;
        const std::vector<std::vector<double> > &integrand_history_;
        std::vector<double> weight_;
        typename traits::ImportanceSampling1TypeTrait<T>::type is_int_1_;
    }; // class f_alpha_
}; // class PathGeometry

} // namespace vsmc

#endif // VSMC_CORE_PATH_HPP
