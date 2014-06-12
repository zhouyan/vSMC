FUNCTION (ADD_SMP_EXECUTABLE base header source smp_name)
    STRING (TOUPPER "${smp_name}" SMP)
    STRING (TOLOWER "${smp_name}" smp)

    IF (${smp} STREQUAL "cilk")
        SET (HALF_NUM_THREADS
            "int tnum = __cilkrts_get_nworkers();")
        SET (HALF_NUM_THREADS ${HALF_NUM_THREADS}
            "tnum = tnum > 1 ? tnum / 2 : tnum;")
        SET (HALF_NUM_THREADS ${HALF_NUM_THREADS}
            "std::stringstream tnum_ss; tnum_ss << tnum;")
        SET (HALF_NUM_THREADS ${HALF_NUM_THREADS}
            "__cilkrts_set_param(\"nworkers\", tnum_ss.str().c_str());")
    ELSEIF (${smp} STREQUAL "omp")
        SET (HALF_NUM_THREADS
            "int tnum = omp_get_max_threads();")
        SET (HALF_NUM_THREADS ${HALF_NUM_THREADS}
            "omp_set_num_threads(tnum > 1 ? tnum / 2 : tnum);")
    ELSEIF (${smp} STREQUAL "std")
        SET (HALF_NUM_THREADS
            "vsmc::ThreadInfo &tinfo = vsmc::ThreadInfo::instance();")
        SET (HALF_NUM_THREADS ${HALF_NUM_THREADS}
            "std::size_t tnum = tinfo.thread_num();")
        SET (HALF_NUM_THREADS ${HALF_NUM_THREADS}
            "tinfo.thread_num(tnum > 1 ? tnum / 2 : tnum);")
    ELSEIF (${smp} STREQUAL "tbb")
        SET (HALF_NUM_THREADS
            "int tnum = tbb::task_scheduler_init::default_num_threads();")
        SET (HALF_NUM_THREADS ${HALF_NUM_THREADS}
            "tbb::task_scheduler_init init(tnum > 1 ? tnum / 2 : tnum);")
    ENDIF (${smp} STREQUAL "cilk")

    CONFIGURE_FILE (
        ${PROJECT_SOURCE_DIR}/include/${header}.hpp
        ${PROJECT_BINARY_DIR}/include/${header}_${smp}.hpp)
    CONFIGURE_FILE (
        ${PROJECT_SOURCE_DIR}/src/${source}.cpp
        ${PROJECT_BINARY_DIR}/src/${source}_${smp}.cpp)

    ADD_EXECUTABLE (${source}_${smp}
        ${PROJECT_BINARY_DIR}/src/${source}_${smp}.cpp)

    IF (${smp} STREQUAL "omp")
        SET_TARGET_PROPERTIES (${source}_${smp} PROPERTIES COMPILE_FLAGS
            "${OpenMP_CXX_FLAGS}")
        TARGET_LINK_LIBRARIES (${source}_${smp} ${OpenMP_LINK_LIBRARIES})
    ELSEIF (${smp} STREQUAL "std")
        TARGET_LINK_LIBRARIES (${source}_${smp} ${VSMC_THREAD_LINK_LIBRARIES})
    ELSEIF (${smp} STREQUAL "tbb")
        TARGET_LINK_LIBRARIES (${source}_${smp} ${TBB_LINK_LIBRARIES})
    ENDIF (${smp} STREQUAL "omp")

    IF (${source} MATCHES "_mpi")
        TARGET_LINK_LIBRARIES (${source}_${smp} ${VSMC_MPI_LINK_LIBRARIES})
        ADD_DEPENDENCIES (example_mpi ${souce}_${smp})
    ENDIF (${source} MATCHES "_mpi")

    TARGET_LINK_LIBRARIES (${source}_${smp} ${EXAMPLE_LINK_LIBRARIES})

    ADD_DEPENDENCIES (${base} ${source}_${smp})
    ADD_DEPENDENCIES (example_${smp} ${source}_${smp})
ENDFUNCTION (ADD_SMP_EXECUTABLE)

FUNCTION (ADD_EXAMPLE base)
    FOREACH (smp ${SMP_EXECUTABLES})
        FOREACH (alg ${ARGN})
            ADD_SMP_EXECUTABLE (${base} ${base} ${base}_${alg} ${smp})
        ENDFOREACH (alg)
    ENDFOREACH (smp)
ENDFUNCTION (ADD_EXAMPLE)

FUNCTION (COPY_FILE basename filename)
    ADD_CUSTOM_COMMAND (
        OUTPUT  ${PROJECT_BINARY_DIR}/${filename}
        DEPENDS ${PROJECT_SOURCE_DIR}/${filename}
        COMMAND ${CMAKE_COMMAND} ARGS -E copy
        ${PROJECT_SOURCE_DIR}/${filename} ${PROJECT_BINARY_DIR}/${filename})
    ADD_CUSTOM_TARGET (${basename}-${filename}
        DEPENDS ${PROJECT_BINARY_DIR}/${filename})
    ADD_DEPENDENCIES (${basename}-files ${basename}-${filename})
ENDFUNCTION (COPY_FILE)

FUNCTION (COPY_DIRECTORY basename dirname fullpath)
    FILE (GLOB_RECURSE dirfiles
        ${fullpath}/*.h ${fullpath}/*.hpp
        ${fullpath}/*.c ${fullpath}/*.cpp ${fullpath}/*.cl)
    ADD_CUSTOM_COMMAND (
        OUTPUT ${PROJECT_BINARY_DIR}/${dirname} DEPENDS ${dirfiles}
        COMMAND ${CMAKE_COMMAND} ARGS -E copy_directory
        ${fullpath} ${PROJECT_BINARY_DIR}/${dirname})
    ADD_CUSTOM_TARGET (${basename}-${dirname}
        DEPENDS ${PROJECT_BINARY_DIR}/${dirname})
    ADD_DEPENDENCIES (${basename}-files ${basename}-${dirname})
ENDFUNCTION (COPY_DIRECTORY)

FUNCTION (COPY_FILE_OPTIONAL basename filename)
    IF (EXISTS ${PROJECT_SOURCE_DIR}/${filename})
        COPY_FILE (${basename} ${filename})
    ELSE (EXISTS ${PROJECT_SOURCE_DIR}/${filename})
            MESSAGE (STATUS "File: ${basename}: ${filename} not available")
    ENDIF (EXISTS ${PROJECT_SOURCE_DIR}/${filename})
ENDFUNCTION (COPY_FILE_OPTIONAL)

FUNCTION (CHECK_HEADER headername cond)
    IF (${cond})
        STRING (REPLACE "/" "_" HEADER_NAME "${headername}")
        STRING (REPLACE ".hpp" "" checkname "${HEADER_NAME}")
        IF (EXISTS ${PROJECT_SOURCE_DIR}/src/header_${checkname}.cpp)
            ADD_EXECUTABLE (header_${checkname}_check
                ${PROJECT_SOURCE_DIR}/src/header_${checkname}.cpp)
        ELSE (EXISTS ${PROJECT_SOURCE_DIR}/src/header_${checkname}.cpp)
            CONFIGURE_FILE (${PROJECT_SOURCE_DIR}/src/header_check.cpp
                ${PROJECT_BINARY_DIR}/src/header_${checkname}.cpp)
            ADD_EXECUTABLE (header_${checkname}_check
                ${PROJECT_BINARY_DIR}/src/header_${checkname}.cpp)
        ENDIF (EXISTS ${PROJECT_SOURCE_DIR}/src/header_${checkname}.cpp)

        FOREACH (arg ${ARGN})
            IF (${arg} STREQUAL "MPI")
                TARGET_LINK_LIBRARIES (header_${checkname}_check
                    ${VSMC_MPI_LINK_LIBRARIES})
            ENDIF (${arg} STREQUAL "MPI")
            IF (${arg} STREQUAL "OPENCL")
                TARGET_LINK_LIBRARIES (header_${checkname}_check
                    ${OpenCL_LINK_LIBRARIES})
            ENDIF (${arg} STREQUAL "OPENCL")
            IF (${arg} STREQUAL "GSL")
                TARGET_LINK_LIBRARIES (header_${checkname}_check
                    ${GSL_LINK_LIBRARIES})
                TARGET_LINK_LIBRARIES (header_${checkname}_check
                    ${GSL_CBLAS_LINK_LIBRARIES})
            ENDIF (${arg} STREQUAL "GSL")
            IF (${arg} STREQUAL "MKL")
                TARGET_LINK_LIBRARIES (header_${checkname}_check
                    ${MKL_LINK_LIBRARIES})
            ENDIF (${arg} STREQUAL "MKL")
            IF (${arg} STREQUAL "GCD")
                TARGET_LINK_LIBRARIES (header_${checkname}_check
                    ${GCD_LINK_LIBRARIES})
            ENDIF (${arg} STREQUAL "GCD")
            IF (${arg} STREQUAL "OPENMP")
                SET_TARGET_PROPERTIES (header_${checkname}_check
                    PROPERTIES COMPILE_FLAGS "${OpenMP_CXX_FLAGS}")
                TARGET_LINK_LIBRARIES (header_${checkname}_check
                    ${OpenMP_LINK_LIBRARIES})
            ENDIF (${arg} STREQUAL "OPENMP")
            IF (${arg} STREQUAL "TBB")
                TARGET_LINK_LIBRARIES (header_${checkname}_check
                    ${TBB_LINK_LIBRARIES})
            ENDIF (${arg} STREQUAL "TBB")
            IF (${arg} STREQUAL "THREAD")
                TARGET_LINK_LIBRARIES (header_${checkname}_check
                    ${VSMC_THREAD_LINK_LIBRARIES})
            ENDIF (${arg} STREQUAL "THREAD")
        ENDFOREACH (arg ${ARGN})

        ADD_DEPENDENCIES (header header_${checkname}_check)
    ENDIF (${cond})
ENDFUNCTION (CHECK_HEADER)
