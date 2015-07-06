cmake_minimum_required(VERSION 2.6)
project(utexas_planning)

find_package(PkgConfig REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem serialization)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp>=0.5)
find_library(YAML_CPP_LIBRARY NAMES yaml-cpp
  HINTS ${YAML_CPP_LIBDIR} ${YAML_CPP_LIBRARY_DIRS}
)

include_directories(include/)
include_directories(${YAML_CPP_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

add_library(utexas_planning_core
  src/lib/common/farmhash.cc
  src/lib/core/abstract_planner.cpp
  src/lib/core/declarative_model.cpp
  src/lib/core/generative_model.cpp
  src/lib/core/state.cpp
)
target_link_libraries(utexas_planning_core
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARY}
)

add_library(utexas_planning_planners
  src/planners/vi/vi.cpp
  src/planners/vi/estimator.cpp
  src/planners/vi/tabular_estimator.cpp
)
target_link_libraries(utexas_planning_planners utexas_planning_core)

add_executable(vi_example
  src/examples/vi/vi_example.cpp
)
target_link_libraries(vi_example utexas_planning_core)
target_link_libraries(vi_example utexas_planning_planners)
target_link_libraries(vi_example ${YAML_CPP_LIBRARIES})
