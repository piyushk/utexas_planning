#ifndef UTEXAS_PLANNING_EVALUATOR_H_
#define UTEXAS_PLANNING_EVALUATOR_H_

#include <string>
#include <utexas_planning/core/abstract_planner.h>
#include <utexas_planning/core/generative_model.h>
#include <utexas_planning/core/visualizer.h>

namespace utexas_planning {

  std::map<std::string, std::string> runSingleTrial(const GenerativeModel::Ptr& model,
                                                    const AbstractPlanner::Ptr& planner,
                                                    const Visualizer::Ptr& visualizer,
                                                    std::string results_directory,
                                                    int seed,
                                                    int max_trial_depth,
                                                    float max_trial_time,
                                                    bool post_action_processing,
                                                    bool verbose,
                                                    bool manual_action_selection);

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_EVALUATOR_H_ */
