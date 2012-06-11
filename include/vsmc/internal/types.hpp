#ifndef VSMC_INTERNAL_TYPES_HPP
#define VSMC_INTERNAL_TYPES_HPP

#ifndef VSMC_SIZE_TYPE
#define VSMC_SIZE_TYPE EIGEN_DEFAULT_DENSE_INDEX_TYPE
#endif // VSMC_SIZE_TYPE

namespace vsmc {

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
