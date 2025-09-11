# ================================ FORMAT ================================
file(GLOB_RECURSE ALL_SOURCES
  "${CMAKE_SOURCE_DIR}/apps/*.cpp"
  "${CMAKE_SOURCE_DIR}/src/*.cpp"
  "${CMAKE_SOURCE_DIR}/include/*.hpp"
)
set(CLANG_FORMAT_SCRIPT ${CMAKE_SOURCE_DIR}/tests/StaticAnalysis/clang_format_tests.sh)
find_program(CLANG_FORMAT "clang-format-19")

if(CLANG_FORMAT)
  add_test(
    NAME FormatCheck
    COMMAND ${CLANG_FORMAT_SCRIPT} ${ALL_SOURCES}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )
else()
  message(WARNING "clang-format not found, formatting tests disabled")
endif()

# ================================= TIDY =================================
file(GLOB_RECURSE CPP_SOURCES
  "${CMAKE_SOURCE_DIR}/apps/*.cpp"
  "${CMAKE_SOURCE_DIR}/src/*.cpp"
)
set(CLANG_TIDY_SCRIPT ${CMAKE_SOURCE_DIR}/tests/StaticAnalysis/clang_tidy_tests.sh)
find_program(CLANG_TIDY "clang-tidy-19")

if(CLANG_TIDY)
  add_test(
    NAME TidyCheck
    COMMAND ${CLANG_TIDY_SCRIPT} ${CPP_SOURCES}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )
else()
  message(WARNING "clang-tidy not found, tidy tests disabled")
endif()
