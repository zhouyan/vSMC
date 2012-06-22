#ifndef VSMC_INTERNAL_TYPES_HPP
#define VSMC_INTERNAL_TYPES_HPP

#ifndef VSMC_SIZE_TYPE
#define VSMC_SIZE_TYPE EIGEN_DEFAULT_DENSE_INDEX_TYPE
#endif // VSMC_SIZE_TYPE

namespace vsmc {

namespace internal {

template <typename T>
class HasSizeType
{
    private :

    template<typename S>
    static char test (typename S::size_type);

    template <typename S>
    static long test (...);

    public :

    static const bool value = sizeof(test<T>(0)) == sizeof(char);
};

template <typename T, bool>
class SizeTypeDispatch;

template <typename T>
class SizeTypeDispatch<T, true>
{
    public :

    typedef typename T::size_type type;
};

template <typename T>
class SizeTypeDispatch<T, false>
{
    public :

    typedef VSMC_SIZE_TYPE type; 
};

} // namesapce vsmc::internal

template <typename T>
class SizeTypeTrait
{
    public :

    typedef typename internal::SizeTypeDispatch<T,
            internal::HasSizeType<T>::value>::type type;
};

/// \brief Resample scheme
/// \ingroup Core
enum ResampleScheme {
    MULTINOMIAL,         ///< Multinomial resampling
    RESIDUAL,            ///< Reisudal resampling
    STRATIFIED,          ///< Startified resampling
    SYSTEMATIC,          ///< Systematic resampling
    RESIDUAL_STRATIFIED, ///< Stratified resampling on the residuals
    RESIDUAL_SYSTEMATIC  ///< Systematic resampling on the residuals
}; // enum ResamleScheme

} // namespace vsmc

#endif // VSMC_INTERNAL_TYPES_HPP
