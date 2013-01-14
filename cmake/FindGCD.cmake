# Find Apple GCD support
#
# The following variable is set
#
# GCD_FOUND          - TRUE if Apple GCD is found and work correctly.
#                      But it is untested by real GCD programs
# GCD_INCLUDE_DIR    - The directory containing GCD headers
# GCD_LINK_LIBRARIES - TBB libraries that shall be linked to
#
# The following variables affect the behavior of this module
#
# GCD_INC_PATH - The path CMake shall try to find headers first
# GCD_LIB_PATH - The path CMake shall try to find libraries first

IF (NOT GCD_FOUND)
    UNSET (GCD_FOUND CACHE)
    IF (NOT GCD_LINK_LIBRARIES)
        UNSET (GCD_LINK_LIBRARIES CACHE)
        IF (APPLE)
            SET (GCD_LINK_LIBRARIES_RELEASE)
            SET (GCD_LINK_LIBRARIES_DEBUG)
            MESSAGE (STATUS "GCD link libraries not need (Mac OS X)")
        ELSE (APPLE)
            FIND_LIBRARY (GCD_LINK_LIBRARIES dispatch
                PATHS ${GCD_LIB_PATH} ENV LIBRARY_PATH)
            IF (GCD_LINK_LIBRARIES)
                SET (GCD_LINK_LIBRARIES_RELEASE ${GCD_LINK_LIBRARIES})
                SET (GCD_LINK_LIBRARIES_DEBUG ${GCD_LINK_LIBRARIES})
                MESSAGE (STATUS "Found GCD libraries: ${GCD_LINK_LIBRARIES}")
            ELSE (GCD_LINK_LIBRARIES)
                MESSAGE (STATUS "NOT Found GCD libraries")
            ENDIF (GCD_LINK_LIBRARIES)
        ENDIF (APPLE)
    ENDIF (NOT GCD_LINK_LIBRARIES)

    IF (NOT GCD_INCLUDE_DIR)
        UNSET (GCD_INCLUDE_DIR CACHE)
        FIND_PATH (GCD_INCLUDE_DIR dispatch/dispatch.h
            PATHS ${GCD_INC_PATH} ENV CPATH)
        IF (GCD_INCLUDE_DIR)
            MESSAGE (STATUS "Found GCD headers: ${GCD_INCLUDE_DIR}")
        ELSE (GCD_INCLUDE_DIR)
            MESSAGE (STATUS "NOT Found GCD headers")
            SET (GCD_FOUND FALSE)
        ENDIF (GCD_INCLUDE_DIR)
    ENDIF (NOT GCD_INCLUDE_DIR)

    IF (APPLE)
        IF (GCD_INCLUDE_DIR)
            MESSAGE (STATUS "Found GCD")
            SET (GCD_FOUND TRUE CACHE BOOL "Found GCD")
        ELSE (GCD_INCLUDE_DIR)
            MESSAGE (STATUS "NOT Found GCD")
            SET (GCD_FOUND FALSE CACHE BOOL "Not Found GCD")
        ENDIF (GCD_INCLUDE_DIR)
    ELSE (APPLE)
        IF (GCD_LINK_LIBRARIES AND GCD_INCLUDE_DIR)
            MESSAGE (STATUS "Found GCD")
            SET (GCD_FOUND TRUE CACHE BOOL "Found GCD")
        ELSE (GCD_LINK_LIBRARIES AND GCD_INCLUDE_DIR)
            MESSAGE (STATUS "NOT Found GCD")
            SET (GCD_FOUND FALSE CACHE BOOL "Not Found GCD")
        ENDIF (GCD_LINK_LIBRARIES AND GCD_INCLUDE_DIR)
    ENDIF (APPLE)
ENDIF (NOT GCD_FOUND)
