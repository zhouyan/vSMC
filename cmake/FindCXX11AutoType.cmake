# Find C++11 auto type support
#
# The following variable is set
#
# CXX11_AUTO_FOUND - TRUE if C++11 auto type is found and work correctly

FILE (READ ${CMAKE_CURRENT_LIST_DIR}/FindCXX11AutoType.cpp
    CXX11_AUTO_TYPE_TEST_SOURCE)

IF (NOT DEFINED CXX11_AUTO_TYPE_FOUND)
    INCLUDE (CheckCXXSourceRuns)
    CHECK_CXX_SOURCE_RUNS ("${CXX11_AUTO_TYPE_TEST_SOURCE}"
        CXX11_AUTO_TYPE_FOUND)
    IF (CXX11_AUTO_TYPE_FOUND)
        MESSAGE (STATUS "Found C++11 auto type support")
    ELSE (CXX11_AUTO_TYPE_FOUND)
        MESSAGE (STATUS "NOT Found C++11 auto type support")
    ENDIF (CXX11_AUTO_TYPE_FOUND)
ENDIF (NOT DEFINED CXX11_AUTO_TYPE_FOUND)
