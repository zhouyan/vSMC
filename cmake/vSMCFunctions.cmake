FUNCTION (ADD_EXAMPLE basename algs)
    FOREACH (exe ${EXAMPLE_EXECUTABLES})
        STRING (TOUPPER "-DUSE_${exe}" flags)

        IF (${exe} STREQUAL "tbb")
            ADD_TBB_RUNTIME(${basename})
        ENDIF (${exe} STREQUAL "tbb")

        FOREACH (alg ${algs})
            SET (exe_name ${basename}-${alg}-${exe})
            ADD_EXECUTABLE (${exe_name} ${basename}-${alg}.cpp)

            IF (${exe} STREQUAL "omp")
                SET_TARGET_PROPERTIES (${exe_name} PROPERTIES COMPILE_FLAGS
                    "${flags} ${OpenMP_CXX_FLAGS}")
            ELSE (${exe} STREQUAL "omp")
                SET_TARGET_PROPERTIES (${exe_name} PROPERTIES COMPILE_FLAGS
                    "${flags}")
            ENDIF (${exe} STREQUAL "omp")

            IF (${exe} STREQUAL "std")
                TARGET_LINK_LIBRARIES (${exe_name}
                    ${EXAMPLE_LINK_LIBRARIES} ${VSMC_THREAD_LINK_LIBRARIES})
            ELSEIF (${exe} STREQUAL "tbb")
                TARGET_LINK_LIBRARIES (${exe_name}
                    ${EXAMPLE_LINK_LIBRARIES} ${TBB_LINK_LIBRARIES})
            ELSEIF (${exe} STREQUAL "omp")
                TARGET_LINK_LIBRARIES (${exe_name}
                    ${EXAMPLE_LINK_LIBRARIES} ${OpenMP_LINK_LIBRARIES})
            ELSE (${exe} STREQUAL "tbb")
                TARGET_LINK_LIBRARIES (${exe_name} ${EXAMPLE_LINK_LIBRARIES})
            ENDIF (${exe} STREQUAL "std")

            IF (${basename} STREQUAL "gmm")
                ADD_TEST (NAME ${exe_name} COMMAND ${exe_name}
                    "--particle_num" "1000"
                    "--prior2" "100"
                    "--burnin_num" "100"
                    "--iter_num" "100"
                    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
                    CONFIGURATIONS Debug Release RelWithDebInfo MinSizeRel)
            ENDIF (${basename} STREQUAL "gmm")

            IF (${basename} STREQUAL "node")
                ADD_TEST (NAME ${exe_name} COMMAND ${exe_name}
                    "--particle_num" "100"
                    "--prior2" "30"
                    "--burnin_num" "100"
                    "--iter_num" "100"
                    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
                    CONFIGURATIONS Debug Release RelWithDebInfo MinSizeRel)
            ENDIF (${basename} STREQUAL "node")

            IF (${basename} STREQUAL "pet")
                ADD_TEST (NAME ${exe_name} COMMAND ${exe_name}
                    "--particle_num" "1000"
                    "--prior2" "500"
                    "--burnin_num" "100"
                    "--iter_num" "100"
                    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
                    CONFIGURATIONS Debug Release RelWithDebInfo MinSizeRel)
            ENDIF (${basename} STREQUAL "pet")

            ADD_DEPENDENCIES (${basename} ${exe_name})
        ENDFOREACH(alg)
    ENDFOREACH (exe)
ENDFUNCTION (ADD_EXAMPLE)

FUNCTION (COPY_FILE basename filename)
    ADD_CUSTOM_COMMAND (
        OUTPUT  ${PROJECT_BINARY_DIR}/${filename}
        DEPENDS ${PROJECT_SOURCE_DIR}/${filename}
        COMMAND ${CMAKE_COMMAND} ARGS -E copy
        ${PROJECT_SOURCE_DIR}/${filename} ${PROJECT_BINARY_DIR}/${filename})
    ADD_CUSTOM_TARGET (${basename}-${filename}
        DEPENDS ${PROJECT_BINARY_DIR}/${filename})
    ADD_DEPENDENCIES (${basename} ${basename}-${filename})
ENDFUNCTION (COPY_FILE)
