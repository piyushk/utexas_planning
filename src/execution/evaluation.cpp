#include <boost/lexical_cast.hpp>

#include <utexas_planning/execution/evaluation.h>

namespace utexas_planning {

  std::map<std::string, std::string> runSingleTrial(const GenerativeModel::ConstPtr& model,
                                                    const AbstractPlanner::Ptr& planner,
                                                    std::string results_directory,
                                                    int seed,
                                                    bool verbose) {

    RewardMetrics::Ptr reward_metrics = model->getRewardMetricsAtEpisodeStart();
    float cumulative_reward = 0.0f;

    State::ConstPtr state = model->getStartState(seed);
    std::map<std::string, std::string> record = state->asMap();
    State::ConstPtr next_state;
    Action::ConstPtr action;
    float reward;
    float post_action_timeout;
    int depth_count;
    boost::shared_ptr<RNG> rng(new RNG(seed));

    planner->performEpisodeStartProcessing(state, model->getInitialTimeout());

    if (verbose) {
      /* std::cout << "Testing model " << model->getName() << " using planner " << planner->getName() << std::endl; */
      std::cout << "  Start State: " << *state << std::endl;
    }
    while (!model->isTerminalState(state)) {
      action = planner->getBestAction(state);
      if (verbose) {
        std::cout << "  Taking action " << *action << " at state: " << *state << std::endl;
      }
      model->takeAction(state,
                        action,
                        reward,
                        reward_metrics,
                        next_state,
                        depth_count,
                        post_action_timeout,
                        rng);
      if (verbose) {
        std::cout << "    Reached state " << *next_state << " with reward: " << reward << std::endl;
      }
      cumulative_reward += reward;
      planner->performPostActionProcessing(state, action, post_action_timeout);
      state = next_state;
    }

    std::map<std::string, std::string> model_params = model->getParamsAsMap();
    record.insert(model_params.begin(), model_params.end());
    std::map<std::string, std::string> planner_params = planner->getParamsAsMap();
    record.insert(planner_params.begin(), planner_params.end());
    std::map<std::string, std::string> reward_metrics_map = reward_metrics->asMap();
    record.insert(reward_metrics_map.begin(), reward_metrics_map.end());
    record["reward"] = boost::lexical_cast<std::string>(cumulative_reward);

    return record;

  }

} /* utexas_planning */
