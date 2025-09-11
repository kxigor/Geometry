find_package(PkgConfig REQUIRED)
find_package(GTest REQUIRED)

link_libraries(Geometry)
link_libraries(Geometry3d)

function(add_google_test TEST_NAME TEST_SRC)
    add_executable(${TEST_NAME} ${TEST_SRC})
    target_include_directories(${TEST_NAME} SYSTEM PUBLIC
        Threads::Threads
        ${GTEST_INCLUDE_DIRS}
        ${GMOCK_INCLUDE_DIRS}
    )
    set_property(TARGET ${TEST_NAME} PROPERTY RUNTIME_OUTPUT_DIRECTORY "")
    target_link_libraries(${TEST_NAME} ${GTEST_LIBRARIES})
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endfunction()

add_google_test(
    DoubleTests
    DoubleTests.cpp
)

add_google_test(
    GeometryTests
    GeometryTests.cpp
)

add_google_test(
    Geometry3dTests
    Geometry3dTests.cpp
)