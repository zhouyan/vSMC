#ifndef V_SMC_PATH_HPP
#define V_SMC_PATH_HPP

#include <vSMC/config.hpp>

namespace vSMC {

template <typename T>
class Path
{
    public :

    /// The type of path sampling integration
    typedef boost::function<double
        (std::size_t, Particle<T> &, double *)> integral_type;

    /// \brief Construct a Path with an integral function
    ///
    /// \param N The size of the particle set
    Path (const integral_type &integral = NULL) : integral_(integral) {}

    /// \brief Copy constructor
    ///
    /// \param path The Path to by copied
    Path (const Path<T> &path) : integral_(path.integral_),
        index_(path.index_), integrand_(path.integrand_),
        width_(path.width_), grid_(path.grid_) {}

    /// \brief Assignment operator
    ///
    /// \param path The Path to be assigned
    /// \return The Path after assignemnt
    Path<T> & operator= (const Path<T> &path)
    {
        if (&path != this) {
            integral_ = path.integral_;
            index_ = path.index_;
            integrand_ = path.integrand_;
            width_ = path.width_;
            grid_ = path.grid_;
        }

        return *this;
    }

    /// \brief Size of records
    ///
    /// \return The number of iterations recorded
    std::size_t iter_size () const
    {
        return index_.size();
    }

    /// \brief Set the integral function
    ///
    /// \param integral The function used to compute the integrands
    void integral (const integral_type &integral)
    {
        integral_ = integral;
    }

    /// \brief Test if the path is empty
    ///
    /// \return True if the path is empty
    bool empty () const
    {
        return integral_.empty();
    }

    /// \brief Get the iteration index
    ///
    /// \return A vector of the index
    std::vector<std::size_t> index () const
    {
        return index_;
    }

    /// \brief Get the iteration index
    ///
    /// \param first An iterator point to where writing starts
    template<typename OIter>
    void index (OIter first) const
    {
        for (std::vector<std::size_t>::const_iterator iter = index_.begin();
               iter != index_.end(); ++iter)
            *first++ = *iter;
    }

    /// \brief Get the record of path sampling integrand
    ///
    /// \return A vector of path sampling integrand
    std::vector<double> integrand () const
    {
        return integrand_;
    }

    /// \brief Get the record of path sampling integrand
    ///
    /// \param first An iterator point to where writing starts
    template<typename OIter>
    void integrand (OIter first) const
    {
        for (std::vector<double>::const_iterator iter = integrand_.begin();
               iter != integrand_.end(); ++iter)
            *first++ = *iter;
    }

    /// \brief Get the record of path sampling width
    ///
    /// \return A vector of path sampling width
    std::vector<double> width () const
    {
        return width_;
    }

    /// \brief Get the record of path sampling width
    ///
    /// \param first An iterator point to where writing starts
    template<typename OIter>
    void width (OIter first) const
    {
        for (std::vector<double>::const_iterator iter = width_.begin();
               iter != width_.end(); ++iter)
            *first++ = *iter;
    }

    /// \brief Get the record of path sampling grid
    ///
    /// \return A vector of path sampling grid
    std::vector<double> grid () const
    {
        return grid_;
    }

    /// \brief Get the record of path sampling grid
    ///
    /// \param first An iterator point to where writing starts
    template<typename OIter>
    void grid (OIter first) const
    {
        for (std::vector<double>::const_iterator iter = grid_.begin();
               iter != grid_.end(); ++iter)
            *first++ = *iter;
    }

    void eval (std::size_t iter, Particle<T> &particle)
    {
        buffer_.resize(particle.size());
        width_.push_back(integral_(iter, particle, buffer_));
        integrand_.push_back(cblas_ddot(particle.size(),
                particle.weight_ptr(), 1, buffer_, 1));
        index_.push_back(iter);
        grid_.push_back(grid_.size() ?
                grid_.back() + width_.back() : width_.back());
    }

    /// \brief Clear the index and record
    void clear ()
    {
        index_.clear();
        integrand_.clear();
        width_.clear();
        grid_.clear();
    }

    private :

    vDist::tool::Buffer<double> buffer_;
    integral_type integral_;
    std::vector<std::size_t> index_;
    std::vector<double> integrand_;
    std::vector<double> width_;
    std::vector<double> grid_;
}; // class PathSampling

} // namespace vSMC

#endif // V_SMC_PATH_HPP
