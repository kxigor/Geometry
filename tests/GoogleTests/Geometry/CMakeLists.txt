add_google_test(GeometryBasicTests basic.cpp)

target_link_libraries(GeometryBasicTests PRIVATE Geometry)
target_include_directories(GeometryBasicTests PRIVATE ${CMAKE_SOURCE_DIR}/include)