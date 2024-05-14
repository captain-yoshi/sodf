#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)


    macro(sodf_add_test TYPE)
        # Split ARGN into .test file and other sources
        set(TEST_FILE ${ARGN})
        set(SOURCES ${ARGN})
        list(FILTER TEST_FILE INCLUDE REGEX "\.test$")
        list(FILTER SOURCES EXCLUDE REGEX "\.test$")
        # Determine TARGET name from first source file
        list(GET SOURCES 0 TEST_NAME)
        string(REGEX REPLACE "\.cpp$" "" TEST_NAME ${TEST_NAME})
        string(REGEX REPLACE "_" "-" TEST_NAME ${TEST_NAME})
        # Configure build target
        if(TEST_FILE) # Add rostest if .test file was specified
            _add_rostest_google_test(${TYPE} ${PROJECT_NAME}-${TEST_NAME} ${TEST_FILE} ${SOURCES})
        else()
            _catkin_add_google_test(${TYPE} ${PROJECT_NAME}-${TEST_NAME} ${SOURCES})
        endif()
        target_link_libraries(${PROJECT_NAME}-${TEST_NAME} ${PROJECT_NAME} quarticRootsFlocke ${catkin_LIBRARIES} gtest_main)
    endmacro()
    macro(sodf_add_gtest)
        sodf_add_test("gtest" ${ARGN})
    endmacro()
    macro(sodf_add_gmock)
        sodf_add_test("gmock" ${ARGN})
    endmacro()

    sodf_add_gtest(test_volume.cpp)
    sodf_add_gtest(test_transform.cpp)
    sodf_add_gtest(test_object.cpp)
    sodf_add_gtest(test_graph.cpp)
    sodf_add_gtest(test_state.cpp)

endif()
