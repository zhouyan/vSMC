#ifndef VSMC_INTERNAL_DEFINES_HPP
#define VSMC_INTERNAL_DEFINES_HPP

#include <vsmc/internal/config.hpp>

// Avoid MSVC stupid behavior
#define VSMC_MACRO_NO_EXPANSION

namespace vsmc {

enum {Dynamic};
enum MonitorMethod {ImportanceSampling, Simple};
enum MatrixOrder {RowMajor = 101, ColMajor = 102};
enum MatrixTranspose {NoTrans = 111, Trans = 112, ConjTrans = 113};

} // namespace vsmc

#endif // VSMC_INTERNAL_DEFINES_HPP
