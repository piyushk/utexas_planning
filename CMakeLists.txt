cmake_minimum_required(VERSION 2.6)
project(utexas_planning)

find_package(Boost REQUIRED COMPONENTS filesystem program_options serialization system)
find_package(class_loader REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp>=0.5)
find_library(YAML_CPP_LIBRARY NAMES yaml-cpp
  HINTS ${YAML_CPP_LIBDIR} ${YAML_CPP_LIBRARY_DIRS}
)

include_directories(include/)
include_directories(${YAML_CPP_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${class_loader_INCLUDE_DIRS})

add_library(utexas_planning_common SHARED
  src/common/farmhash.cc
)

add_library(utexas_planning_core SHARED
  src/core/abstract_planner.cpp
  src/core/action.cpp
  src/core/declarative_model.cpp
  src/core/generative_model.cpp
  src/core/state.cpp
)
target_link_libraries(utexas_planning_core
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARY}
  utexas_planning_common
)

add_library(utexas_planning_planners SHARED
  src/planners/random/random_planner.cpp
  src/planners/mcts/mcts.cpp
  src/planners/vi/vi.cpp
  src/planners/vi/estimator.cpp
  src/planners/vi/tabular_estimator.cpp
)
target_link_libraries(utexas_planning_planners utexas_planning_core)
target_link_libraries(utexas_planning_planners ${class_loader_LIBRARIES})

add_library(utexas_planning_models SHARED
  src/models/grid_model.cpp
)
target_link_libraries(utexas_planning_models utexas_planning_core)
target_link_libraries(utexas_planning_models ${class_loader_LIBRARIES})

add_library(utexas_planning_execution SHARED
  src/execution/class_loader.cpp
  src/execution/evaluation.cpp
)
target_link_libraries(utexas_planning_execution utexas_planning_core)
target_link_libraries(utexas_planning_execution ${class_loader_LIBRARIES})

add_executable(vi_example
  src/examples/vi/vi_example.cpp
)
target_link_libraries(vi_example utexas_planning_planners)
target_link_libraries(vi_example utexas_planning_models)

add_executable(evaluator
  src/execution/evaluator.cpp
)
target_link_libraries(evaluator utexas_planning_execution)
target_link_libraries(evaluator utexas_planning_core)
