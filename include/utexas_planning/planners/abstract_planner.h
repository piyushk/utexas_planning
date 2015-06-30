#ifndef UTEXAS_PLANNING_ABSTRACT_PLANNER_H_
#define UTEXAS_PLANNING_ABSTRACT_PLANNER_H_

#include <boost/shared_ptr.hpp>

#include <utexas_planning/core/GenerativeModel.h>

namespace utexas_planning {

  class AbstractPlanner {

    public:
      ~AbstractPlanner() {}

      virtual void init(const boost::shared_ptr<const GenerativeModel> &model, const YAML::Node &params, const std::string &output_directory) = 0;
      virtual void performEpisodeStartProcessing(const State &start_state, float timeout = NO_TIMEOUT) = 0;
      virtual Action getBestAction(const State &state) = 0;
      virtual void performPostActionProcessing(const State& state, const Action& action, float timeout = NO_TIMEOUT) = 0;
      virtual std::string getSolverName() = 0;
      virtual std::map<std::string, std::string> getParamsAsMap() = 0;
  };

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_ABSTRACT_PLANNER_H_ */

