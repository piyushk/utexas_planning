#ifndef UTEXAS_PLANNING_EVALUATOR_H_
#define UTEXAS_PLANNING_EVALUATOR_H_

#include <string>
#include <utexas_planning/core/generative_model.h>
#include <utexas_planning/core/abstract_planner.h>

namespace utexas_planning {

  std::map<std::string, std::string> runSingleTrial(const GenerativeModel::ConstPtr& model,
                                                    const AbstractPlanner::Ptr& planner,
                                                    std::string results_directory,
                                                    int seed);

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_EVALUATOR_H_ */