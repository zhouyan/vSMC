#ifndef VSMC_INTERNAL_CONFIG_HPP
#define VSMC_INTERNAL_CONFIG_HPP

#ifdef NDEBUG
#define VSMC_NDEBUG
#endif // NDEBUG

// Uncomment the following line to use C++11 <functional>
// #define VSMC_USE_STD_FUNCTION

// Uncomment the following line to use C++11 <random>
// #define VSMC_USE_STD_RANDOM

// Uncomment the following line to use C++11 <type_traits>
// #define VSMC_USE_STD_TYPE_TRAITS

// Uncomment the following line to use sequential pseudo RNG
// #define VSMC_USE_SEQUENTIAL_RNG

#include <vsmc/internal/compiler.hpp>

#if defined(VSMC_USE_CONSTEXPR_ENGINE) && !defined(VSMC_USE_SEQUENTIAL_RNG)
#define VSMC_USE_STD_RANDOM
#endif // VSMC_USE_CONSTEXPR_ENGINE

#if !defined(VSMC_USE_STD_FUNCITON) || !defined(VSMC_USE_STD_RANDOM)
#include <boost/version.hpp>
#ifndef VSMC_NDEBUG
#if BOOST_VERSION < 104900
#warning vSMC was only tested against Boost 1.49 and later
#endif // BOOST_VERSION < 104900
#endif // VSMC_NDEBUG
#endif // !defined(VSMC_USE_STD_FUNCITON) || !defined(VSMC_USE_STD_RANDOM)

#endif // VSMC_INTERNAL_CONFIG_HPP
