#ifndef V_SMC_INTERNAL_TYPES_HPP
#define V_SMC_INTERNAL_TYPES_HPP

#ifndef V_SMC_INDEX_TYPE
#define V_SMC_INDEX_TYPE EIGEN_DEFAULT_DENSE_INDEX_TYPE
#endif // V_SMC_INDEX_TYPE

namespace vSMC {

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

} // namespace vSMC

#endif // V_SMC_INTERNAL_TYPES_HPP
