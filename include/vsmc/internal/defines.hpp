#ifndef VSMC_INTERNAL_DEFINES_HPP
#define VSMC_INTERNAL_DEFINES_HPP

#include <vsmc/internal/config.hpp>

// Avoid MSVC stupid behavior
#define VSMC_MACRO_NO_EXPANSION

#if !defined(NDEBUG) || VSMC_RUNTIME_ASSERT_AS_EXCEPTION
#define VSMC_SMP_BASE_DESTRUCTOR_PREFIX virtual
#else
#define VSMC_SMP_BASE_DESTRUCTOR_PREFIX
#endif

#define VSMC_DEFINE_SMP_IMPL_PROTECTED(ImplName, BaseName)                    \
BaseName##ImplName () {}                                                      \
BaseName##ImplName () (const BaseName##ImplName<T, Derived> &other) :         \
    BaseName##Base<T, Derived>(other) {}                                      \
BaseName##ImplName<T, Derived> &operator=                                     \
    (const BaseName##ImplName<T, Derived> &other)                             \
{                                                                             \
    if (this != other) BaseName##Base<T, Derived>::operator=(other);          \
    return *this;                                                             \
}                                                                             \
~Classname () {}

namespace vsmc {

enum {Dynamic};
enum MatrixOrder {RowMajor = 101, ColMajor = 102};
enum ResampleScheme {
    Multinomial,
    Residual,
    Stratified,
    Systematic,
    ResidualStratified,
    ResidualSystematic};

} // namespace vsmc

#endif // VSMC_INTERNAL_DEFINES_HPP
