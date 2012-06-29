#ifndef VSMC_CORE_PATH_HPP
#define VSMC_CORE_PATH_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

/// \brief Monitor for path sampling
/// \ingroup Core
///
/// \tparam T Particle<T>::value_type
template <typename T>
class Path
{
    public :

    /// The type of the particle values
    typedef T value_type;

    /// The type of path sampling evaluation functor
    typedef internal::function<double (
            unsigned, const Particle<T> &, double *)> eval_type;

    /// The type of the index vector
    typedef std::vector<unsigned> index_type;

    /// The type of the integrand vector
    typedef std::vector<double> integrand_type;

    /// The type of the width vector
    typedef std::vector<double> width_type;

    /// The type of the grid vector
    typedef std::vector<double> grid_type;

    /// \brief Construct a Path with an evaluation function
    ///
    /// \param eval The functor used to compute the integrands
    explicit Path (const eval_type &eval = VSMC_NULLPTR) : eval_(eval) {}

    Path (const Path<T> &other) :
        eval_(other.eval_),
        index_(other.index_), integrand_(other.integrand_),
        width_(other.width_), grid_(other.grid_) {}

    Path<T> & operator= (const Path<T> &other)
    {
        if (&other != this) {
            eval_      = other.eval_;
            index_     = other.index_;
            integrand_ = other.integrand_;
            width_     = other.width_;
            grid_      = other.grid_;
        }

        return *this;
    }

    /// Size of records
    unsigned iter_size () const
    {
        return static_cast<unsigned>(index_.size());
    }

    /// \brief Test if the monitor is valid
    ///
    /// \note This operator will be \c explicit if the C++11 feature is enabled
#if VSMC_HAS_CXX11_EXPLICIT_CONVERSIONS
    explicit
#endif
        operator bool () const
    {
        return bool(eval_);
    }

    /// Iteration index
    const index_type &index () const
    {
        return index_;
    }

    /// Record of path sampling integrands
    const integrand_type &integrand () const
    {
        return integrand_;
    }

    /// Record of path sampling width
    const width_type &width () const
    {
        return width_;
    }

    /// Record of path sampling grid (accumulated width)
    const grid_type &grid () const
    {
        return grid_;
    }

    /// Set the evaluation functor
    void set_eval (const eval_type &new_eval)
    {
        eval_ = new_eval;
    }

    /// \brief Evaluate the integration
    ///
    /// \param iter The iteration number
    /// \param particle The particle set to be operated on by eval()
    void eval (unsigned iter, const Particle<T> &particle)
    {
        VSMC_RUNTIME_ASSERT((bool(eval_)),
                ("CALL **Path::eval** WITH AN INVALID "
                 "EVALUATION FUNCTOR"));

        double w = 0;
        double p = 0;
        buffer_.resize(particle.size());
        w = eval_(iter, particle, buffer_.data());
        p = particle.weight().dot(buffer_);
        width_.push_back(w);
        integrand_.push_back(p);
        index_.push_back(iter);
        grid_.push_back(grid_.size() ?
                grid_.back() + width_.back() : width_.back());
    }

    /// Path sampling estimate of normalizing constant
    double zconst () const
    {
        double sum = 0;
        for (unsigned i = 1; i != iter_size(); ++i)
            sum += 0.5 * width_[i] * (integrand_[i-1] + integrand_[i]);

        return sum;
    }

    /// \brief Clear all recorded data
    ///
    /// \note The evaluation functor is not reset
    void clear ()
    {
        index_.clear();
        integrand_.clear();
        width_.clear();
        grid_.clear();
    }

    private :

    Eigen::VectorXd buffer_;
    eval_type eval_;
    index_type index_;
    integrand_type integrand_;
    width_type width_;
    grid_type grid_;
}; // class PathSampling

} // namespace vsmc

#endif // VSMC_CORE_PATH_HPP
