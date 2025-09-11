# Geometry library (2D)
add_library(Geometry STATIC
    Geometry/Geometry.cpp
    Geometry/ConvexHull.cpp
    Geometry/VoronoiDiagram.cpp
)

target_include_directories(Geometry PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

# Geometry3d library (3D)
add_library(Geometry3d STATIC
    Geometry3d/Geometry3d.cpp
    Geometry3d/ConvexHull.cpp
)

target_include_directories(Geometry3d PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

# Double (header-only)
add_library(Double INTERFACE)
target_include_directories(Double INTERFACE
    ${CMAKE_SOURCE_DIR}/include/Double
)