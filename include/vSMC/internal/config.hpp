#ifndef V_SMC_CONFIG_CONFIG_HPP
#define V_SMC_CONFIG_CONFIG_HPP

#include <vSMC/internal/version.hpp>
#include <boost/version.hpp>

#if BOOST_VERSION < 104700
#define V_SMC_BOOST_RANDOM_NAMESPACE boost
#else
#define V_SMC_BOOST_RANDOM_NAMESPACE boost::random
#endif

#endif // V_SMC_CONFIG_CONFIG_HPP
