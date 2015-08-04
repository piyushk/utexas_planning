#include <map>

#include <utexas_planning/core/abstract_planner.h>

namespace utexas_planning {

  AbstractPlanner::~AbstractPlanner() {}

  std::string AbstractPlanner::getName() const {
    return typeid(*this).name();
  }

  std::map<std::string, std::string> AbstractPlanner::getParamsAsMap() const {
    return std::map<std::string, std::string>();
  }

} /* utexas_planning */
