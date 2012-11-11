# Unix-like systems usually don't need these settings
# However, on Windows, if the PATH are not properly set, you can configure the
# following options to help CMake find vSMC's dependencies

# The Root directory to find Boost
# Under this directory `include/boost` and `lib` directory shall be found
SET (BOOST_ROOT "C:/Program Files/Boost" CACHE STRING "Boost ROOT")

# Whether to use static boost libs
SET (Boost_USE_STATIC_LIBS ON CACHE BOOL "Boost use static libraries")

# The inlude directory of Random123
# Under this directory `Random123/threefry.h` etc shall be found
SET (Random123_INC_PATH "C:/Program Files/Random123/include" CACHE STRING "R123")

# The Root director to find Intel Threading Building Blocks
# Under this directory, on Windows, `bin`, `include` and `lib` shall be found
SET (TBB_ROOT "C:/Program Files/Intel/TBB/4.1" CACHE STRING "TBB ROOT")
