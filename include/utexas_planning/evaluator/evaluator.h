#ifndef UTEXAS_PLANNING_EVALUATOR_H_
#define UTEXAS_PLANNING_EVALUATOR_H_

#include <class_loader/multi_library_class_loader.h>

namespace utexas_planning {


  void runSingleTrial(const GenerativeModel::ConstPtr& model,
                      const AbstractPlanner::Ptr& planner,
                      std::string results_directory,
                      int seed) {

    RewardMetrics::Ptr reward_metrics = model->getRewardMetricsAtEpisodeStart();
    float cumulative_reward = 0.0f;

    State::ConstPtr state = model->getStartState(seed);
    std::map<std::string, std::string> record = state->asMap();
    State::ConstPtr next_state;
    Action::ConstPtr action;
    float reward;
    float post_action_timeout;

    planner->performEpisodeStartProcessing(state, model->getInitialTimeout());

    while (!model->isTerminalState(state)) {
      action = planner->getBestAction(state);
      model->takeAction(state,
                        action,
                        reward,
                        reward_metrics,
                        next_state,
                        depth_count,
                        post_action_timeout,
                        rng);
      cumulative_reward += reward;
      planner->performPostActionProcessing(state, action, post_action_timeout);
      state = next_state;
    }

    std::map<std::string, std::string> model_params = model->getParamsAsMap();
    record.insert(model_params.begin(), model_params.end());
    std::map<std::string, std::string> solver_params = solver_->getParamsAsMap();
    record.insert(solver_params.begin(), solver_params.end());
    std::map<std::string, std::string> reward_metrics_map = reward_metrics->asMap();
    record.insert(reward_metrics_map.begin(), reward_metrics_map.end());
    record["reward"] = cumulative_reward;

    bwi_tools::writeRecordsAsCSV(results_directory + "/part." + boost::lexical_cast<std::string>(seed), record);

  }

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_EVALUATOR_H_ */
