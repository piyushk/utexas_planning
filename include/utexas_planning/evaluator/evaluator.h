#ifndef UTEXAS_PLANNING_EVALUATOR_H_
#define UTEXAS_PLANNING_EVALUATOR_H_

namespace utexas_planning {

  class Evaluator {

    // TODO Add params for timeout etc.

    public:

      /* Assumes that the model and planners have already been initialized. */
      Evaluator(const GenerativeModel::ConstPtr& model,
                const AbstractPlanner::Ptr& planner) : model_(model), planner_(planner) {}

      void runSingleTrial(int seed) {

        RewardMetrics::Ptr reward_metrics = model_->getRewardMetricsAtEpisodeStart();

        State::ConstPtr state = model_->getStartState(seed);
        std::map<std::string, std::string> record = state->asMap();
        State::ConstPtr next_state;
        Action::ConstPtr action;
        float reward;
        float post_action_timeout;

        planner->performEpisodeStartProcessing(state, params_.start_timeout);

        while (!model_->isTerminalState(state)) {
          action = planner_->getBestAction(state);
          model_->takeAction(state,
                             action,
                             reward,
                             reward_metrics,
                             next_state,
                             depth_count,
                             post_action_timeout,
                             rng);
          planner->performPostActionProcessing(state, action, post_action_timeout);
          state = next_state;
        }

        std::map<std::string, std::string> model_params = model_->getParamsAsMap();
        record.insert(model_params.begin(), model_params.end());
        std::map<std::string, std::string> solver_params = solver_->getParamsAsMap();
        record.insert(solver_params.begin(), solver_params.end());
        std::map<std::string, std::string> reward_metrics_map = reward_metrics->asMap();
        record.insert(reward_metrics_map.begin(), reward_metrics_map.end());

        bwi_tools::writeRecordsAsCSV(results_directory_ + "/result_s/part." + boost::lexical_cast<std::string>(seed),
      }

    private:

  }; /* Evaluator */

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_EVALUATOR_H_ */
