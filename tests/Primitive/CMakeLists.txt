file(GLOB PRIMITIVE_TESTS 
  "${CMAKE_SOURCE_DIR}/tests/Primitive/*.sh"
)

foreach(test_script ${PRIMITIVE_TESTS})
get_filename_component(test_name ${test_script} NAME_WE)
add_test(
    NAME ${test_name}
    COMMAND ${test_script}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
endforeach()