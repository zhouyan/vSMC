#ifndef V_SMC_CORE_PATH_HPP
#define V_SMC_CORE_PATH_HPP

#include <vSMC/internal/common.hpp>

namespace vSMC {

/// \brief Monitor for path sampling
/// \ingroup Core
///
/// \tparam T Particle<T>::value_type
template <typename T>
class Path
{
    public :

    /// \brief The type of path sampling evaluation functor
    typedef internal::function<double (
            unsigned, const Particle<T> &, double *)> eval_type;

    /// The type of the index vector
    typedef std::deque<unsigned> index_type;

    /// The type of the integrand vector
    typedef std::deque<double> integrand_type;

    /// The type of the width vector
    typedef std::deque<double> width_type;

    /// The type of the grid vector
    typedef std::deque<double> grid_type;

    /// \brief Construct a Path with an evaluation function
    ///
    /// \param eval The functor used to compute the integrands
    explicit Path (const eval_type &eval = NULL) : eval_(eval) {}

    /// \brief Copy constructor
    ///
    /// \param other The Path to by copied
    Path (const Path<T> &other) :
        eval_(other.eval_),
        index_(other.index_), integrand_(other.integrand_),
        width_(other.width_), grid_(other.grid_) {}

    /// \brief Assignment operator
    ///
    /// \param other The Path to be assigned
    /// \return The Path after assignemnt
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

    /// \brief Size of records
    ///
    /// \return The number of iterations recorded
    index_type::size_type iter_size () const
    {
        return index_.size();
    }

    /// \brief Test if the path is empty
    ///
    /// \return \b true if the path is empty
    ///
    /// \note A path is empty means it has no valid evaluation functions.
    /// However its record may not be empty (for that purpose call iter_size()
    /// to see). Call eval(iter, particle) on an empty path is a runtime error.
    bool empty () const
    {
        return !bool(eval_);
    }

    /// \brief Iteration index
    ///
    /// \return A const reference to the index
    const index_type &index () const
    {
        return index_;
    }

    /// \brief Record of path sampling integrand
    ///
    /// \return A const reference to the integrand
    const integrand_type &integrand () const
    {
        return integrand_;
    }

    /// \brief Record of path sampling width
    ///
    /// \return A const reference to the width
    const width_type &width () const
    {
        return width_;
    }

    /// \brief Record of path sampling grid
    ///
    /// \return A const reference to the grid
    const grid_type &grid () const
    {
        return grid_;
    }

    /// \brief Set the evaluation functor
    ///
    /// \param new_eval The functor used to compute the integrands
    void set_eval (const eval_type &new_eval)
    {
        eval_ = new_eval;
    }

    /// \brief Evaluate the integration
    ///
    /// \param iter The iteration number
    /// \param particle The particle set to be operated on by eval()
    ///
    /// \note The evaluaiton functor has to be set to a non-NULL value before
    /// calling eval().  Otherwise exception will be raised when calling
    /// eval().
    void eval (unsigned iter, const Particle<T> &particle)
    {
        assert(eval_);

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

    /// \brief Path sampling estimate of normalizing constant
    ///
    /// \return The Path sampling normalzing constant estimate
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

} // namespace vSMC

#endif // V_SMC_CORE_PATH_HPP
