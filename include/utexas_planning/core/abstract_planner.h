#ifndef UTEXAS_PLANNING_ABSTRACT_PLANNER_H_
#define UTEXAS_PLANNING_ABSTRACT_PLANNER_H_

#include <boost/shared_ptr.hpp>
#include <yaml-cpp/yaml.h>

#include <utexas_planning/core/generative_model.h>
#include <utexas_planning/common/constants.h>

namespace utexas_planning {

  class AbstractPlanner {

    public:

      typedef boost::shared_ptr<AbstractPlanner> Ptr;
      typedef boost::shared_ptr<const AbstractPlanner> ConstPtr;

      ~AbstractPlanner() {}

      virtual void init(const GenerativeModel::ConstPtr &model,
                        const YAML::Node &params,
                        const std::string &output_directory) = 0;

      virtual void performEpisodeStartProcessing(const State::ConstPtr &start_state,
                                                 float timeout = NO_TIMEOUT) = 0;

      virtual Action::ConstPtr getBestAction(const State::ConstPtr &state) const = 0;

      virtual void performPostActionProcessing(const State::ConstPtr& state,
                                               const Action::ConstPtr& action,
                                               float timeout = NO_TIMEOUT) = 0;

      virtual std::string getName() const;

      virtual std::map<std::string, std::string> getParamsAsMap() const;
  };

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_ABSTRACT_PLANNER_H_ */

