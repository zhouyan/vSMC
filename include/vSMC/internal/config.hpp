#ifndef V_SMC_CONFIG_CONFIG_HPP
#define V_SMC_CONFIG_CONFIG_HPP

#ifdef NDEBUG
#define V_SMC_NDEBUG
#endif // NDEBUG

// Uncomment the following line to use C++11 <functional>
// #define V_SMC_USE_STD_FUNCTION

// Uncomment the following line to use C++11 <random>
// #define V_SMC_USE_STD_RANDOM

#if !defined(V_SMC_USE_STD_FUNCITON) || !defined(V_SMC_USE_STD_RANDOM)
#include <boost/version.hpp>
#ifndef V_SMC_NDEBUG
#if BOOST_VERSION < 104900
#warning vSMC was only tested against Boost 1.49 and later
#endif // BOOST_VERSION < 104900
#endif // V_SMC_NDEBUG
#endif // !defined(V_SMC_USE_STD_FUNCITON) || !defined(V_SMC_USE_STD_RANDOM)

#endif // V_SMC_CONFIG_CONFIG_HPP
