# Application D
add_executable(D
    D/D.cpp
)
target_link_libraries(D
    Geometry
    Double
)

# Application F
add_executable(F
    F/F.cpp
)
target_link_libraries(F
    Geometry3d
    Double
)

# Application G
add_executable(G
    G/G.cpp
)
target_link_libraries(G
    Geometry
    Double
)

# Application H
add_executable(H
    H/H.cpp
)
target_link_libraries(H
    Geometry
    Double
)