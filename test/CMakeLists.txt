set(SODF_TESTS
    test_ecs.cpp
    test_fluid_domain_shape.cpp
    test_xml_parser.cpp
)

set(TEST_DEPENDECIES
    ${SODF_LIBRARY}
    quarticRootsFlocke
    ${MUPARSER_LIB}
)


if(ament_cmake_FOUND)

    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(${SODF_LIBRARY}_test ${SODF_TESTS})
    target_link_libraries(${SODF_LIBRARY}_test
        ${TEST_DEPENDECIES}
        ${ament_LIBRARIES})

elseif(catkin_FOUND AND CATKIN_ENABLE_TESTING)
    catkin_add_gtest(${SODF_LIBRARY}_test ${SODF_TESTS})
    target_link_libraries(${SODF_LIBRARY}_test
        ${TEST_DEPENDECIES}
        Threads::Threads
        ${catkin_LIBRARIES})

else()

    find_package(GTest REQUIRED)
    enable_testing()

    add_executable(${SODF_LIBRARY}_test ${SODF_TESTS})

    target_include_directories(${SODF_LIBRARY}_test
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/../include
            ${CMAKE_CURRENT_SOURCE_DIR}/../ext/better-enums
            ${CMAKE_CURRENT_SOURCE_DIR}/../ext/ginseng
            ${CMAKE_CURRENT_SOURCE_DIR}/../ext/quarticRootsFlocke/src
        )

    target_link_libraries(${SODF_LIBRARY}_test
        ${TEST_DEPENDECIES}
        Threads::Threads
        GTest::gtest)

endif()

target_compile_definitions(${SODF_LIBRARY}_test PRIVATE SODF_TEST_FOLDER="${CMAKE_CURRENT_SOURCE_DIR}")
