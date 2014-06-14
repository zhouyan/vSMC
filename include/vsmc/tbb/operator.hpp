#ifndef VSMC_TBB_UTILITY_OPERATOR_HPP
#define VSMC_TBB_UTILITY_OPERATOR_HPP

#include <vsmc/internal/common.hpp>
#include <cmath>
#include <limits>

#define VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_UNARY_OBJECT(name, uni) \
template <typename T>                                                        \
class name                                                                   \
{                                                                            \
    public :                                                                 \
                                                                             \
    name (const T *data, T *result) : data_(data), result_(result) {}        \
                                                                             \
    template <typename  Range>                                               \
    void operator() (const Range &range) const                               \
    {                                                                        \
        const T *const data = data_;                                         \
        T *const result = result_;                                           \
        uni op;                                                              \
        for (typename Range::const_iterator i = range.begin();               \
                i != range.end(); ++i)                                       \
            result[i] = op(data[i]);                                         \
    }                                                                        \
                                                                             \
    private :                                                                \
                                                                             \
    const T *const data_;                                                    \
    T *const result_;                                                        \
}; // VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_UNARY_OBJECT

#define VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_BINARY_OBJECT(name, bin) \
template <typename T>                                                        \
class name                                                                   \
{                                                                            \
    public :                                                                 \
                                                                             \
    name (const T *data, T *result, const T &val) :                          \
        data_(data), result_(result), val_(val) {}                           \
                                                                             \
    template <typename Range>                                                \
    void operator() (const Range &range) const                               \
    {                                                                        \
        const T *const data = data_;                                         \
        T *const result = result_;                                           \
        bin op;                                                              \
        for (typename Range::const_iterator i = range.begin();               \
                i != range.end(); ++i)                                       \
            result[i] = op(data[i], val_);                                   \
    }                                                                        \
                                                                             \
    private :                                                                \
                                                                             \
    const T *const data_;                                                    \
    T *const result_;                                                        \
    const T val_;                                                            \
}; // VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_BINARY_OBJECT

#define VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_UNARY_FUNCTION(name, uni) \
template <typename T>                                                        \
class name                                                                   \
{                                                                            \
    public :                                                                 \
                                                                             \
    name (const T *data, T *result) : data_(data), result_(result) {}        \
                                                                             \
    template <typename  Range>                                               \
    void operator() (const Range &range) const                               \
    {                                                                        \
        const T *const data = data_;                                         \
        T *const result = result_;                                           \
        for (typename Range::const_iterator i = range.begin();               \
                i != range.end(); ++i)                                       \
            result[i] = uni(data[i]);                                        \
    }                                                                        \
                                                                             \
    private :                                                                \
                                                                             \
    const T *const data_;                                                    \
    T *const result_;                                                        \
}; // VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_UNARY_FUNCTION

#define VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_BINARY_FUNCTION(name, bin) \
template <typename T>                                                        \
class name                                                                   \
{                                                                            \
    public :                                                                 \
                                                                             \
    name (const T *data, T *result, const T &val) :                          \
        data_(data), result_(result), val_(val) {}                           \
                                                                             \
    template <typename Range>                                                \
    void operator() (const Range &range) const                               \
    {                                                                        \
        const T *const data = data_;                                         \
        T *const result = result_;                                           \
        for (typename Range::const_iterator i = range.begin();               \
                i != range.end(); ++i)                                       \
            result[i] = bin(data[i], val_);                                  \
    }                                                                        \
                                                                             \
    private :                                                                \
                                                                             \
    const T *const data_;                                                    \
    T *const result_;                                                        \
    const T val_;                                                            \
}; // VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_BINARY_FUNCTION

#define VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_UNARY_OPERATOR(name, uni) \
template <typename T>                                                        \
class name                                                                   \
{                                                                            \
    public :                                                                 \
                                                                             \
    name (const T *data, T *result) : data_(data), result_(result) {}        \
                                                                             \
    template <typename Range>                                                \
    void operator() (const Range &range) const                               \
    {                                                                        \
        const T *const data = data_;                                         \
        T *const result = result_;                                           \
        for (typename Range::const_iterator i = range.begin();               \
                i != range.end(); ++i)                                       \
            result[i] = uni data[i];                                         \
    }                                                                        \
                                                                             \
    private :                                                                \
                                                                             \
    const T *const data_;                                                    \
    T *const result_;                                                        \
}; // VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_UNARY_OPERATOR

#define VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_BINARY_OPERATOR(name, bin) \
template <typename T>                                                        \
class name                                                                   \
{                                                                            \
    public :                                                                 \
                                                                             \
    name (const T *data, T *result, const T &val) :                          \
        data_(data), result_(result), val_(val) {}                           \
                                                                             \
    template <typename Range>                                                \
    void operator() (const Range &range) const                               \
    {                                                                        \
        const T *const data = data_;                                         \
        T *const result = result_;                                           \
        for (typename Range::const_iterator i = range.begin();               \
                i != range.end(); ++i)                                       \
            result[i] = data[i] bin val_;                                    \
    }                                                                        \
                                                                             \
    private :                                                                \
                                                                             \
    const T *const data_;                                                    \
    T *const result_;                                                        \
    const T val_;                                                            \
}; // VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_BINARY_OPERATOR

#define VSMC_DEFINE_TBB_UTILITY_OPERATOR_REDUCE_BINARY_OBJECT(name, bin, id) \
template <typename T>                                                        \
class name                                                                   \
{                                                                            \
    public :                                                                 \
                                                                             \
    name (const T *data) : data_(data), result_(id) {}                       \
                                                                             \
    template <typename Split>                                                \
    name (const name<T> &other, Split) : data_(other.data_), result_(id) {}  \
                                                                             \
    template <typename Range>                                                \
    void operator() (const Range &range)                                     \
    {                                                                        \
        const T *const data = data_;                                         \
        T result = result_;                                                  \
        bin op;                                                              \
        for (typename Range::const_iterator i = range.begin();               \
                i != range.end(); ++i)                                       \
            result = op(result, data[i]);                                    \
        result_ = result;                                                    \
    }                                                                        \
                                                                             \
    void join (const name<T> &other)                                         \
    {                                                                        \
        bin op;                                                              \
        result_ = op(result_, other.result_);                                \
    }                                                                        \
                                                                             \
    T result () const {return result_;}                                      \
                                                                             \
    private :                                                                \
                                                                             \
    const T *const data_;                                                    \
    T result_;                                                               \
}; // VSMC_DEFINE_TBB_UTILITY_OPERATOR_REDUCE_BINARY_OBJECT

#define VSMC_DEFINE_TBB_UTILITY_OPERATOR_REDUCE_BINARY_FUNCTION(name, bin, id)\
template <typename T>                                                        \
class name                                                                   \
{                                                                            \
    public :                                                                 \
                                                                             \
    name (const T *data) : data_(data), result_(id) {}                       \
                                                                             \
    template <typename Split>                                                \
    name (const name<T> &other, Split) :  data_(other.data_), result_(id) {} \
                                                                             \
    template <typename Range>                                                \
    void operator() (const Range &range)                                     \
    {                                                                        \
        const T *const data = data_;                                         \
        T result = result_;                                                  \
        for (typename Range::const_iterator i = range.begin();               \
                i != range.end(); ++i)                                       \
            result = bin(result, data[i]);                                   \
        result_ = result;                                                    \
    }                                                                        \
                                                                             \
    void join (const name<T> &other) {result_ = bin(result_, other.result_);}\
                                                                             \
    T result () const {return result_;}                                      \
                                                                             \
    private :                                                                \
                                                                             \
    const T *const data_;                                                    \
    T result_;                                                               \
}; // VSMC_DEFINE_TBB_UTILITY_OPERATOR_REDUCE_BINARY_FUNCTION

#define VSMC_DEFINE_TBB_UTILITY_OPERATOR_REDUCE_BINARY_OPERATOR(name, bin, id)\
template <typename T>                                                        \
class name                                                                   \
{                                                                            \
    public :                                                                 \
                                                                             \
    name (const T *data) : data_(data), result_(id) {}                       \
                                                                             \
    template <typename Split>                                                \
    name (const name<T> &other, Split) : data_(other.data_), result_(id) {}  \
                                                                             \
    template <typename Range>                                                \
    void operator() (const Range &range)                                     \
    {                                                                        \
        const T *const data = data_;                                         \
        T result = result_;                                                  \
        for (typename Range::const_iterator i = range.begin();               \
                i != range.end(); ++i)                                       \
            result = result bin data[i];                                     \
        result_ = result;                                                    \
    }                                                                        \
                                                                             \
    void join (const name<T> &other) {result_ = result_ bin other.result_;}  \
                                                                             \
    T result () const {return result_;}                                      \
                                                                             \
    private :                                                                \
                                                                             \
    const T *const data_;                                                    \
    T result_;                                                               \
}; // VSMC_DEFINE_TBB_UTILITY_OPERATOR_REDUCE_BINARY_OPERATOR

namespace vsmc {

namespace internal {

template <typename T> inline const T &tbb_max_fn (const T &a, const T &b)
{return std::max VSMC_MNE (a, b);}

template <typename T> inline const T &tbb_min_fn (const T &a, const T &b)
{return std::min VSMC_MNE (a, b);}

template <typename T> struct positive_infinity_trait
{static T value () {return std::numeric_limits<T>::max VSMC_MNE();}};

template <typename T> struct negative_infinity_trait
{static T value () {return std::numeric_limits<T>::min VSMC_MNE();}};

template <> struct positive_infinity_trait<float>
{static float value () {return std::numeric_limits<float>::infinity();}};

template <> struct negative_infinity_trait<float>
{static float value () {return -std::numeric_limits<float>::infinity();}};

template <> struct positive_infinity_trait<double>
{static double value () {return std::numeric_limits<double>::infinity();}};

template <> struct negative_infinity_trait<double>
{static double value () {return -std::numeric_limits<double>::infinity();}};

template <> struct positive_infinity_trait<long double>
{
    static long double value ()
    {return std::numeric_limits<long double>::infinity();}
}; // struct positive_infinity_trait

template <> struct negative_infinity_trait<long double>
{
    static long double value ()
    {return -std::numeric_limits<long double>::infinity();}
}; // struct negative_infinity_trait

template <typename T> struct zero_trait
{static T value () {return static_cast<T>(0);}};

template <typename T> struct one_trait
{static T value () {return static_cast<T>(1);}};

} // namespace vsmc::internal

/// \brief Summation of squares
/// \ingroup TBBOp
template <typename T>
class TBBSquareSum
{
    public :

    TBBSquareSum (const T *data) :
        data_(data), result_(internal::zero_trait<T>::value()) {}

    template <typename Split>
    TBBSquareSum (const TBBSquareSum<T> &other, Split) :
        data_(other.data_), result_(internal::zero_trait<T>::value()) {}

    template <typename Range>
    void operator() (const Range &range)
    {
        const T *const data = data_;
        T result = result_;
        for (typename Range::const_iterator i = range.begin();
                i != range.end(); ++i)
            result = result + data[i] * data[i];
        result_ = result;
    }

    void join (const TBBSquareSum<T> &other)
    {result_ = result_ + other.result_;}

    T result () const {return result_;}

    private :

    const T *const data_;
    T result_;
}; // class TBBSquareSum

/// \brief Negation
/// \ingroup TBBOp
VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_UNARY_OPERATOR(TBBNegate, -)

/// \brief Plus
/// \ingroup TBBOp
VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_BINARY_OPERATOR(TBBPlus, +)

/// \brief Minus
/// \ingroup TBBOp
VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_BINARY_OPERATOR(TBBMinus, -)

/// \brief Multiple
/// \ingroup TBBOp
VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_BINARY_OPERATOR(TBBMultiplies, *)

/// \brief Division
/// \ingroup TBBOp
VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_BINARY_OPERATOR(TBBDivides, /)

/// \brief Modulo
/// \ingroup TBBOp
VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_BINARY_OPERATOR(TBBModulus, %)

/// \brief Absolute value
/// \ingroup TBBOp
VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_UNARY_FUNCTION(TBBAbs, std::abs)

/// \brief Exponential
/// \ingroup TBBOp
VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_UNARY_FUNCTION(TBBExp, std::exp)

/// \brief Logarithm
/// \ingroup TBBOp
VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_UNARY_FUNCTION(TBBLog, std::log)

/// \brief Logarithm of 10
/// \ingroup TBBOp
VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_UNARY_FUNCTION(TBBLog10, std::log10)

/// \brief Square root
/// \ingroup TBBOp
VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_UNARY_FUNCTION(TBBSqrt, std::sqrt)

/// \brief Sine
/// \ingroup TBBOp
VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_UNARY_FUNCTION(TBBSin, std::sin)

/// \brief Cosine
/// \ingroup TBBOp
VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_UNARY_FUNCTION(TBBCos, std::cos)

/// \brief Tangent
/// \ingroup TBBOp
VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_UNARY_FUNCTION(TBBTan, std::tan)

/// \brief arc sine
/// \ingroup TBBOp
VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_UNARY_FUNCTION(TBBAsin, std::asin)

/// \brief Arc cosine
/// \ingroup TBBOp
VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_UNARY_FUNCTION(TBBAcos, std::acos)

/// \brief Arc tangent
/// \ingroup TBBOp
VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_UNARY_FUNCTION(TBBAtan, std::atan)

/// \brief Hyperbolic sine
/// \ingroup TBBOp
VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_UNARY_FUNCTION(TBBSinh, std::sinh)

/// \brief Hyperbolic cosine
/// \ingroup TBBOp
VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_UNARY_FUNCTION(TBBCosh, std::cosh)

/// \brief Hyperbolic tangent
/// \ingroup TBBOp
VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_UNARY_FUNCTION(TBBTanh, std::tanh)

/// \brief Ceil
/// \ingroup TBBOp
VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_UNARY_FUNCTION(TBBCeil, std::ceil)

/// \brief Floor
/// \ingroup TBBOp
VSMC_DEFINE_TBB_UTILITY_OPERATOR_FOR_UNARY_FUNCTION(TBBFloor, std::floor)

/// \brief Maximum
/// \ingroup TBBOp
VSMC_DEFINE_TBB_UTILITY_OPERATOR_REDUCE_BINARY_FUNCTION(TBBMaximum, max_fn,
        internal::negative_infinity_trait<T>::value())

/// \brief Minimum
/// \ingroup TBBOp
VSMC_DEFINE_TBB_UTILITY_OPERATOR_REDUCE_BINARY_FUNCTION(TBBMinimum, min_fn,
        internal::positive_infinity_trait<T>::value())

/// \brief Summation
/// \ingroup TBBOp
VSMC_DEFINE_TBB_UTILITY_OPERATOR_REDUCE_BINARY_OPERATOR(TBBSummation, +,
        internal::zero_trait<T>::value())

/// \brief Product
/// \ingroup TBBOp
VSMC_DEFINE_TBB_UTILITY_OPERATOR_REDUCE_BINARY_OPERATOR(TBBProduct, *,
        internal::one_trait<T>::value())

} // namespace vsmc

#endif // VSMC_TBB_UTILITY_OPERATOR_HPP
