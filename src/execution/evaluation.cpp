#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/lexical_cast.hpp>

#include <utexas_planning/execution/evaluation.h>

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
                                                    bool manual_action_selection) {

    verbose = (manual_action_selection) ? true : verbose;

    RewardMetrics::Ptr reward_metrics = model->getRewardMetricsAtEpisodeStart();
    float cumulative_reward = 0.0f;

    State::ConstPtr state = model->getStartState(seed);
    std::map<std::string, std::string> record = state->asMap();
    record["model_name"] = model->getName();
    record["planner_name"] = planner->getName();
    State::ConstPtr next_state;
    Action::ConstPtr action;
    float reward;
    float post_action_timeout;
    int depth_count;
    boost::shared_ptr<RNG> rng(new RNG(seed));

    if (!manual_action_selection) {
      planner->performEpisodeStartProcessing(state, model->getInitialTimeout());
    }
    visualizer->startEpisode(state);

    if (verbose) {
      std::cout << "Testing model " << model->getName() << " using planner " << planner->getName() << std::endl;
      std::cout << "  Start State: " << *state << std::endl;
    }
    int current_depth = 0;
    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::ptime end_time = start_time + boost::posix_time::milliseconds(max_trial_time * 1000);
    boost::posix_time::ptime current_time = start_time;

    while (!model->isTerminalState(state) &&
           ((max_trial_time == NO_TIMEOUT) || (current_time < end_time)) &&
           ((max_trial_depth == NO_MAX_DEPTH) || (current_depth < max_trial_depth))) {
      if (!manual_action_selection) {
        action = planner->getBestAction(state);
        if (verbose) {
          std::cout << "  Taking action " << *action << std::endl;
        }
      } else {
        // List out all actions and ask a user to select an action.
        std::vector<Action::ConstPtr> actions;
        model->getActionsAtState(state, actions);
        std::cout << "Action choices:" << std::endl;
        for (unsigned int i = 0; i < actions.size(); ++i) {
          std::cout << "  " << i << ")" << *(actions[i]) << std::endl;
        }
        int choice;
        std::cout << "Choice: ";
        std::cin >> choice;
        action = actions[choice];
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

      current_depth += depth_count;
      current_time = boost::posix_time::microsec_clock::local_time();

      // Only perform search for next state, if
      if (((max_trial_time != NO_TIMEOUT) && (current_time >= end_time)) ||
          ((max_trial_depth != NO_MAX_DEPTH) && (current_depth >= max_trial_depth))) {
        break;
      }

      visualizer->updateState(next_state, post_action_timeout);

      if (!manual_action_selection && !(model->isTerminalState(next_state))) {
        if (post_action_processing) {
          planner->performPostActionProcessing(state, action, post_action_timeout);
        } else {
          planner->performPreActionProcessing(next_state, action, post_action_timeout);
        }
      }

      state = next_state;
      current_time = boost::posix_time::microsec_clock::local_time();
    }

    record["terminal"] = boost::lexical_cast<std::string>(model->isTerminalState(state));
    std::map<std::string, std::string> model_params = model->getParamsAsMap();
    record.insert(model_params.begin(), model_params.end());
    std::map<std::string, std::string> planner_params = planner->getParamsAsMap();
    record.insert(planner_params.begin(), planner_params.end());
    std::map<std::string, std::string> reward_metrics_map = reward_metrics->asMap();
    record.insert(reward_metrics_map.begin(), reward_metrics_map.end());
    record["reward"] = boost::lexical_cast<std::string>(cumulative_reward);

    // Store termination criteria.
    current_time = boost::posix_time::microsec_clock::local_time();
    record["time"] = boost::lexical_cast<std::string>((current_time - start_time).total_milliseconds());
    record["max_trial_depth"] = boost::lexical_cast<std::string>(max_trial_depth);

    return record;

  }

} /* utexas_planning */
