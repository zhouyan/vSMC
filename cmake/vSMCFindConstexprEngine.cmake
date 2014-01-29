FILE (READ ${PROJECT_SOURCE_DIR}/cmake/vSMCFindConstexprEngine.cpp
    VSMC_CONSTEXPR_ENGINE_TEST_SOURCE)

IF (NOT DEFINED VSMC_CONSTEXPR_ENGINE_FOUND)
    INCLUDE (CheckCXXSourceRuns)
    SET (SAFE_CMAKE_REQUIRED_DEFINITIONS ${CMAKE_REQUIRED_DEFINITIONS})
    SET (SAFE_CMAKE_REQUIRED_INCLUDES ${CMAKE_REQUIRED_INCLUDES})

    IF (VSMC_RANDOM_STD_FOUND)
        SET (CMAKE_REQUIRED_DEFINITIONS ${SAFE_CMAKE_REQUIRED_DEFINITIONS}
            -DVSMC_HAS_CXX11LIB_RANDOM=1 -DVSMC_USE_RANDOM123=0)
        CHECK_CXX_SOURCE_RUNS ("${VSMC_CONSTEXPR_ENGINE_TEST_SOURCE}"
            VSMC_CONSTEXPR_ENGINE_FOUND)
    ELSEIF (VSMC_RANDOM_BOOST_FOUND)
        SET (CMAKE_REQUIRED_DEFINITIONS ${SAFE_CMAKE_REQUIRED_DEFINITIONS}
            -DVSMC_HAS_CXX11LIB_RANDOM=0 -DVSMC_USE_RANDOM123=0)
        CHECK_CXX_SOURCE_RUNS ("${VSMC_CONSTEXPR_ENGINE_TEST_SOURCE}"
            VSMC_CONSTEXPR_ENGINE_FOUND)
    ENDIF (VSMC_RANDOM_STD_FOUND)

    IF (VSMC_CONSTEXPR_ENGINE_FOUND)
        SET (VSMC_CONSTEXPR_ENGINE_FOUND TRUE CACHE BOOL
            "Found constexpr RNG engine min/max")
    ELSE (VSMC_CONSTEXPR_ENGINE_FOUND)
        SET (VSMC_CONSTEXPR_ENGINE_FOUND FALSE CACHE BOOL
            "NOT Found constexpr RNG engine min/max")
    ENDIF (VSMC_CONSTEXPR_ENGINE_FOUND)

    SET (CMAKE_REQUIRED_DEFINITIONS ${SAFE_CMAKE_REQUIRED_DEFINITIONS})
    SET (CMAKE_REQUIRED_INCLUDES ${SAFE_CMAKE_REQUIRED_INCLUDES})
ENDIF (NOT DEFINED VSMC_CONSTEXPR_ENGINE_FOUND)
