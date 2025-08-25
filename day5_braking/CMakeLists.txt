cmake_minimum_required(VERSION 3.12)
project(braking_decision_lib LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Library target
add_library(braking src/braking.cpp)

# Make headers visible to dependents
target_include_directories(braking PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# CLI app
add_executable(brake_cli app/brake_cli.cpp)
target_link_libraries(brake_cli PRIVATE braking)

# Tiny assert-based tests (no framework)
add_executable(braking_tests tests/test_braking.cpp)
target_link_libraries(braking_tests PRIVATE braking)

# Warnings (GNU/Clang) + optional -Werror
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set(COMMON_WARNINGS -Wall -Wextra)
  target_compile_options(braking PRIVATE ${COMMON_WARNINGS})
  target_compile_options(brake_cli PRIVATE ${COMMON_WARNINGS})
  target_compile_options(braking_tests PRIVATE ${COMMON_WARNINGS})

  option(BRK_STRICT "Treat warnings as errors" OFF)
  if (BRK_STRICT)
    target_compile_options(braking PRIVATE -Werror)
    target_compile_options(brake_cli PRIVATE -Werror)
    target_compile_options(braking_tests PRIVATE -Werror)
  endif()
endif()