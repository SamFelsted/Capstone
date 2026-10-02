cmake_minimum_required(VERSION 3.20)
project(meltybrain LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(melty_firmware
  firmware/src/controller.cpp
  firmware/src/runtime.cpp
  firmware/src/board_profile.cpp
)
target_include_directories(melty_firmware PUBLIC firmware/include)

add_library(melty_physics
  sim/src/motor.cpp
  sim/src/plant.cpp
  sim/src/sim_hal.cpp
  sim/src/simulator.cpp
)
target_include_directories(melty_physics PUBLIC sim/include PRIVATE sim/src)
target_link_libraries(melty_physics PUBLIC melty_firmware)
set_target_properties(melty_firmware melty_physics PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(sim_cli sim/src/main.cpp)
target_link_libraries(sim_cli PRIVATE melty_physics)

option(MELTY_BUILD_PYTHON "Build the Python simulator module when pybind11 is available" ON)
if(MELTY_BUILD_PYTHON)
  find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
  execute_process(
    COMMAND "${Python_EXECUTABLE}" -m pybind11 --cmakedir
    OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE PYBIND11_CMAKE_RESULT
  )
  if(PYBIND11_CMAKE_RESULT EQUAL 0)
    list(APPEND CMAKE_PREFIX_PATH "${PYBIND11_CMAKE_DIR}")
  endif()
  find_package(pybind11 CONFIG QUIET)
  if(pybind11_FOUND)
    pybind11_add_module(melty_sim sim/bindings/module.cpp)
    target_link_libraries(melty_sim PRIVATE melty_physics)
    set_target_properties(melty_sim PROPERTIES
      LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python"
    )
  else()
    message(STATUS "pybind11 not found; melty_sim Python module disabled")
  endif()
endif()

include(CTest)
if(BUILD_TESTING)
  foreach(test_name IN ITEMS phase runtime plant closed_loop)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/${test_name}_test.cpp")
      add_executable(${test_name}_test "tests/${test_name}_test.cpp")
      target_link_libraries(${test_name}_test PRIVATE melty_physics)
      add_test(NAME ${test_name} COMMAND ${test_name}_test)
    endif()
  endforeach()
endif()
