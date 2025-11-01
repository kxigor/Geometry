add_google_test(DoubleBasicTests basic.cpp)

target_link_libraries(DoubleBasicTests PRIVATE Double)
target_include_directories(DoubleBasicTests PRIVATE ${CMAKE_SOURCE_DIR}/include)