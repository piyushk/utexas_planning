cmake_minimum_required(VERSION 2.6)
project(utexas_planning)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp>=0.5)

include_directories(include/)
include_directories(${YAML_CPP_INCLUDE_DIRS})

add_library(utexas_planning_core
  src/lib/common/farmhash.cc
  src/lib/core/abstract_planner.cpp
  src/lib/core/declarative_model.cpp
  src/lib/core/generative_model.cpp
)

add_library(utexas_planners
  src/planners/vi/vi.cpp
  src/planners/vi/estimator.cpp
  src/planners/vi/tabular_estimator.cpp
)
target_link_libraries(utexas_planners utexas_planning_core)
target_link_libraries(utexas_planners ${YAML_CPP_LIBRARIES})
