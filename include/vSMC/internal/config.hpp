#ifndef V_SMC_CONFIG_CONFIG_HPP
#define V_SMC_CONFIG_CONFIG_HPP

#include <vSMC/internal/version.hpp>
#include <boost/version.hpp>

#ifndef V_SMC_NDEBUG
#if BOOST_VERSION < 104900
#warning vSMC was only tested against Boost 1.49 and later
#endif
#endif

#endif // V_SMC_CONFIG_CONFIG_HPP
