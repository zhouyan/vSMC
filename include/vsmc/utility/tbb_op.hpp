#ifndef VSMC_UTILITY_TBB_OP_HPP
#define VSMC_UTILITY_TBB_OP_HPP

#include <vsmc/internal/common.hpp>

#define VSMC_DEFINE_TBB_OP_FOR_UNARY_OBJECT(name, uni) \
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
};

#define VSMC_DEFINE_TBB_OP_FOR_BINARY_OBJECT(name, bin) \
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
};

#define VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(name, uni) \
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
};

#define VSMC_DEFINE_TBB_OP_FOR_BINARY_FUNCTION(name, bin) \
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
};

#define VSMC_DEFINE_TBB_OP_FOR_UNARY_OPERATOR(name, uni) \
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
};

#define VSMC_DEFINE_TBB_OP_FOR_BINARY_OPERATOR(name, bin) \
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
};

#define VSMC_DEFINE_TBB_OP_REDUCE_BINARY_OBJECT(name, bin, identity) \
template <typename T>                                                        \
class name                                                                   \
{                                                                            \
    public :                                                                 \
                                                                             \
    name (const T *data) : data_(data), result_(identity) {}                 \
                                                                             \
    template <typename Split>                                                \
    name (const name<T> &other, Split) :                                     \
        data_(other.data_), result_(identity) {}                             \
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
};

#define VSMC_DEFINE_TBB_OP_REDUCE_BINARY_FUNCTION(name, bin, identity) \
template <typename T>                                                        \
class name                                                                   \
{                                                                            \
    public :                                                                 \
                                                                             \
    name (const T *data) : data_(data), result_(identity) {}                 \
                                                                             \
    template <typename Split>                                                \
    name (const name<T> &other, Split) :                                     \
        data_(other.data_), result_(identity) {}                             \
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
};

#define VSMC_DEFINE_TBB_OP_REDUCE_BINARY_OPERATOR(name, bin, identity) \
template <typename T>                                                        \
class name                                                                   \
{                                                                            \
    public :                                                                 \
                                                                             \
    name (const T *data) : data_(data), result_(identity) {}                 \
                                                                             \
    template <typename Split>                                                \
    name (const name<T> &other, Split) :                                     \
        data_(other.data_), result_(identity) {}                             \
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
};

namespace vsmc {

namespace tbb_op {

template <typename T> inline const T &max_fn (const T &a, const T &b)
{return std::max VSMC_MNE (a, b);}

template <typename T> inline const T &min_fn (const T &a, const T &b)
{return std::min VSMC_MNE (a, b);}

/// \brief Positive infinity of a given floating points type (or maximum for
/// non-floating points type).
/// \ingroup TBBOp
///
/// \details
/// Specialize this trait to use with tbb_op::minimum
template <typename T> struct positive_infinity_trait
{static T value () {return std::numeric_limits<T>::max VSMC_MNE();}};

/// \brief Negative infinity of a given floating points type (or minimum for
/// non-floating points type)
/// \ingroup TBBOp
///
/// \details
/// Specialize this trait to use with tbb_op::maximum
template <typename T> struct negative_infinity_trait
{static T value () {return std::numeric_limits<T>::min VSMC_MNE();}};

/// \brief Positive infinity of a given floating points type
/// \ingroup TBBOp
template <> struct positive_infinity_trait<float>
{static T value () {return std::numeric_limits<float>::infinity();}};

/// \brief Negative infinity of a given floating points type
/// \ingroup TBBOp
template <> struct negative_infinity_trait<float>
{static T value () {return -std::numeric_limits<float>::infinity();}};

/// \brief Positive infinity of a given floating points type
/// \ingroup TBBOp
template <> struct positive_infinity_trait<double>
{static T value () {return std::numeric_limits<double>::infinity();}};

/// \brief Negative infinity of a given floating points type
/// \ingroup TBBOp
template <> struct negative_infinity_trait<double>
{static T value () {return -std::numeric_limits<double>::infinity();}};

/// \brief Positive infinity of a given floating points type
/// \ingroup TBBOp
template <> struct positive_infinity_trait<long double>
{static T value () {return std::numeric_limits<long double>::infinity();}};

/// \brief Negative infinity of a given floating points type
/// \ingroup TBBOp
template <> struct negative_infinity_trait<long double>
{static T value () {return -std::numeric_limits<long double>::infinity();}};

/// \brief Zero of a given type
/// \ingroup TBBOp
template <typename T> struct zero_trait
{static T value () {return static_cast<T>(0);}};

/// \brief One of a given type
/// \ingroup TBBOp
template <typename T> struct one_trait
{static T value () {return static_cast<T>(1);}};

/// \brief Summation of squares
/// \ingroup TBBOp
template <typename T>
class square_sum
{
    public :

    square_sum (const T *data) :
        data_(data), result_(zero_trait<T>::value()) {}

    template <typename Split>
    square_sum (const square_sum<T> &other, Split) :
        data_(other.data_), result_(zero_trait<T>::value()) {}

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

    void join (const square_sum<T> &other) {result_ = result_ + other.result_;}

    T result () const {return result_;}

    private :

    const T *const data_;
    T result_;
}; // class square_sum

/// \brief Negation
/// \ingroup TBBOp
VSMC_DEFINE_TBB_OP_FOR_UNARY_OPERATOR(negate, -)

/// \brief Plus
/// \ingroup TBBOp
VSMC_DEFINE_TBB_OP_FOR_BINARY_OPERATOR(plus,       +)

/// \brief Minus
/// \ingroup TBBOp
VSMC_DEFINE_TBB_OP_FOR_BINARY_OPERATOR(minus,      -)

/// \brief Multiple
/// \ingroup TBBOp
VSMC_DEFINE_TBB_OP_FOR_BINARY_OPERATOR(multiplies, *)

/// \brief Division
/// \ingroup TBBOp
VSMC_DEFINE_TBB_OP_FOR_BINARY_OPERATOR(divides,    /)

/// \brief Modulo
/// \ingroup TBBOp
VSMC_DEFINE_TBB_OP_FOR_BINARY_OPERATOR(modulus,    %)

/// \brief Absolute value
/// \ingroup TBBOp
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(abs,   std::abs)

/// \brief Exponential
/// \ingroup TBBOp
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(exp,   std::exp)

/// \brief Logarithm
/// \ingroup TBBOp
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(log,   std::log)

/// \brief Logarithm of 10
/// \ingroup TBBOp
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(log10, std::log10)

/// \brief Square root
/// \ingroup TBBOp
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(sqrt,  std::sqrt)

/// \brief Sine
/// \ingroup TBBOp
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(sin,   std::sin)

/// \brief Cosine
/// \ingroup TBBOp
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(cos,   std::cos)

/// \brief Tangent
/// \ingroup TBBOp
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(tan,   std::tan)

/// \brief arc sine
/// \ingroup TBBOp
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(asin,  std::asin)

/// \brief arc cosine
/// \ingroup TBBOp
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(acos,  std::acos)

/// \brief arc tangent
/// \ingroup TBBOp
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(atan,  std::atan)

/// \brief Hyperbolic sine
/// \ingroup TBBOp
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(sinh,  std::sinh)

/// \brief Hyperbolic cosine
/// \ingroup TBBOp
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(cosh,  std::cosh)

/// \brief Hyperbolic tangent
/// \ingroup TBBOp
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(tanh,  std::tanh)

/// \brief Ceil
/// \ingroup TBBOp
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(ceil,  std::ceil)

/// \brief Floor
/// \ingroup TBBOp
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(floor, std::floor)

/// \brief Maximum
/// \ingroup TBBOp
VSMC_DEFINE_TBB_OP_REDUCE_BINARY_FUNCTION(maximum, max_fn,
        negative_infinity_trait<T>::value())

/// \brief Minimum
/// \ingroup TBBOp
VSMC_DEFINE_TBB_OP_REDUCE_BINARY_FUNCTION(minimum, min_fn,
        positive_infinity_trait<T>::value())

/// \brief Summation
/// \ingroup TBBOp
VSMC_DEFINE_TBB_OP_REDUCE_BINARY_OPERATOR(summation, +, zero_trait<T>::value())

/// \brief Product
/// \ingroup TBBOp
VSMC_DEFINE_TBB_OP_REDUCE_BINARY_OPERATOR(product,   *, one_trait<T>::value())

} // namespace vsmc::tbb_op

} // namespace vsmc

#endif // VSMC_UTILITY_TBB_OP_HPP
