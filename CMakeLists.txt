cmake_minimum_required(VERSION 2.6)
project(utexas_planning)

find_package(Boost REQUIRED COMPONENTS filesystem program_options serialization system)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp>=0.5)
pkg_check_modules(CLASS_LOADER REQUIRED class_loader)

find_library(YAML_CPP_LIBRARY NAMES yaml-cpp
  HINTS ${YAML_CPP_LIBDIR} ${YAML_CPP_LIBRARY_DIRS}
)

find_library(CLASS_LOADER_LIBRARY NAMES class_loader
  HINTS ${CLASS_LOADER_LIBDIR} ${CLASS_LOADER_LIBRARY_DIRS}
)

include_directories(include/)
include_directories(${YAML_CPP_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${CLASS_LOADER_INCLUDE_DIRS})

## add_definitions(-DMCTS_DEBUG)

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
target_link_libraries(utexas_planning_planners ${CLASS_LOADER_LIBRARY})
target_link_libraries(utexas_planning_planners ${Boost_LIBRARIES})

add_library(utexas_planning_models SHARED
  src/models/grid_model.cpp
  src/models/grid_model2.cpp
  src/models/grid_model_3d.cpp
  src/models/light_world.cpp
  src/models/simple_guidance.cpp
  include/rddl/evaluatables.cc
  include/rddl/instantiator.cc
  include/rddl/logical_expressions.cc
  include/rddl/planning_task.cc
  include/rddl/preprocessor.cc
  include/rddl/probability_distribution.cc
  include/rddl/rddl_parser.cc
  include/rddl/states.cc
  include/rddl/task_analyzer.cc
  include/rddl/utils/math_utils.cc
  include/rddl/utils/string_utils.cc
  include/rddl/utils/system_utils.cc
  include/rddl/utils/timer.cc
  include/rddl/rddl_model.cpp
)
target_link_libraries(utexas_planning_models utexas_planning_core)
target_link_libraries(utexas_planning_models ${CLASS_LOADER_LIBRARY})

add_library(utexas_planning_execution SHARED
  src/execution/class_loader.cpp
  src/execution/evaluation.cpp
)
target_link_libraries(utexas_planning_execution utexas_planning_core)
target_link_libraries(utexas_planning_execution ${CLASS_LOADER_LIBRARY})

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

install(
  TARGETS
    utexas_planning_common
    utexas_planning_core
    utexas_planning_planners
    utexas_planning_models
    utexas_planning_execution
    vi_example
    evaluator
  ## ARCHIVE DESTINATION bin
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
