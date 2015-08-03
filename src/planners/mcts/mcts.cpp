#include <boost/date_time/posix_time/posix_time_types.hpp>

#include <utexas_planning/common/exceptions.h>
#include <utexas_planning/planners/mcts/mcts.h>

#ifdef MCTS_DEBUG
#define MCTS_OUTPUT(x) std::cout << x << std::endl
#else
#define MCTS_OUTPUT(x) ((void) 0)
#endif

namespace utexas_planning {

  void MCTS::init(const GenerativeModel::ConstPtr& model,
                  const YAML::Node& params,
                  const std::string& /* output_directory */,
                  const boost::shared_ptr<RNG>& rng) {
    model_ = model;
    params_.fromYaml(params);
    rng_ = rng;
    start_action_available_ = false;
  }

  void MCTS::search(const State::ConstPtr& start_state,
                    const Action::ConstPtr& start_action,
                    float timeout,
                    int max_playouts) {
    start_action_ = start_action;
    start_action_available_ = true;
    search(start_state, timeout, max_playouts);
  }

  void MCTS::search(const State::ConstPtr& start_state,
                    float timeout,
                    int max_playouts) {

    // Do some basic sanity checks.
    if (timeout <= 0 && max_playouts <= 0) {
      throw IncorrectUsageException("MCTS: either timeout or max_playouts has to be greater than 0 for MCTS search.");
    }

    int current_playouts = 0;
    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::ptime end_time = start_time + boost::posix_time::milliseconds(timeout * 1000);
    boost::posix_time::ptime current_time = start_time;

    while (((timeout <= 0.0) || (current_time < end_time)) &&
           ((max_playouts <= 0) || (current_playouts < max_playouts))) {

      MCTS_OUTPUT("------------START ROLLOUT--------------");
      std::vector<HistoryStep> history;
      State::ConstPtr state = start_state;

      State::Ptr discretized_state = start_state->clone();
      if (!root_node_) {
        root_node_ = getNewStateNode(discretized_state);
      }
      StateNode::Ptr state_node = root_node_;

      State::ConstPtr next_state;
      Action::ConstPtr action;
      unsigned int action_id;
      int depth_count;
      float reward;

      bool at_start_state = true;

      // TODO state mapping.
      // stateMapping->map(discretizedState); // discretize state

      unsigned int new_states_added_in_rollout = 0;

      // Version of the code with no max depth (and hence no pre-cached history memory).
      for (unsigned int depth = 0;
           ((params_.max_depth != 0) || (depth < params_.max_depth) &&
            (timeout <= 0.0) || (current_time < end_time));
           depth += depth_count) {

        // Select action, take it and update the model with the action taken in simulation.
        MCTS_OUTPUT("MCTS State: " << *state << " " << "DEPTH: " << depth);

        if (at_start_state && start_action_available_) {
          action = start_action_;
        } else {
          if (state_node) {
            action = getPlanningAction(discretized_state, state_node);
          } else {
            action = default_planner_->getBestAction(discretized_state);
          }
        }

        if (state_node->actions.find(action) == state_node->actions.end()) {
          std::stringstream ss;
          ss << "MCTS: Action " << *action << " being executed at state " << *state << " is not a valid action. " <<
            "This can also happen due to incorrect initialization in getNewStateNode()." << std::endl;
          throw IncorrectUsageException(ss.str());
        }

        model_->takeAction(state, action, reward, next_state, depth_count, rng_);

        MCTS_OUTPUT(" Action Selected: " << action);
        MCTS_OUTPUT("   Reward: " << reward);

        // Record this step in history.
        HistoryStep step;
        step.state = state_node;
        step.action = action;
        step.reward = reward;
        history.push_back(step);

        if (model_->isTerminalState(next_state)) {
          break;
        }

        state = next_state;
        discretized_state = next_state->clone();
        // TODO handle state discretization
        //stateMapping->map(discretized_state);

        // Hunt for next state in the current state_node
        StateActionNode::Ptr& action_node = state_node->actions[action];
        if (action_node->next_states.find(discretized_state) == action_node->next_states.end()) {
          if ((params_.max_new_states_per_rollout == 0) ||
              (new_states_added_in_rollout < params_.max_new_states_per_rollout)) {
            state_node = action_node->next_states[discretized_state] = getNewStateNode(discretized_state);
            ++new_states_added_in_rollout;
          } else {
            state_node.reset();
          }
        } else {
          state_node = action_node->next_states[discretized_state];
        }
        at_start_state = false;
      }

      MCTS_OUTPUT("------------ BACKPROPAGATION --------------");
      float backup_value = 0;

      MCTS_OUTPUT("At final discretized state: " << discretizedState <<
                  " the backprop value is " << backup_value);

      for (int step = history.size() - 1; step >= 0; --step) {
        backup_value = history[step].reward + params_.gamma * backup_value;
        if (history[step].state) {
          updateState(history[step], backup_value);
        }
      }

      ++current_playouts;
      current_time = boost::posix_time::microsec_clock::local_time();
    }

    start_action_available_ = false;
  }

  Action::ConstPtr MCTS::getBestAction(const State::ConstPtr& state) const {

    typedef std::pair<Action::ConstPtr, StateActionNode::ConstPtr> Action2StateActionInfoPair;

    // Look for this state from the root node.
    State::Ptr discretized_state = state->clone();
    // TODO handle state discretization
    //stateMapping->map(discretized_state);

    StateNode::ConstPtr state_node;
    bool use_default_action = true;
    if (root_node_) {
      StateActionNode::ConstPtr action = root_node_->actions.find(last_action_selected_)->second;
      if (action->next_states.find(discretized_state) != action->next_states.end()) {
        state_node = action->next_states.find(discretized_state)->second;
        use_default_action = false;
      }
    }

    if (use_default_action) {
      return default_planner_->getBestAction(state);
    }

    float max_value = -std::numeric_limits<float>::max();
    std::vector<Action::ConstPtr> best_actions;
    BOOST_FOREACH(const Action2StateActionInfoPair& action_info_pair, state_node->actions) {
      float val = getStateActionValue(action_info_pair.second);
      if (fabs(val - max_value) < 1e-10) {
        best_actions.push_back(action_info_pair.first);
      } else if (val > max_value) {
        max_value = val;
        best_actions.clear();
        best_actions.push_back(action_info_pair.first);
      }
    }

    return best_actions[rng_->randomInt(best_actions.size() - 1)];
  }

  void MCTS::performEpisodeStartProcessing(const State::ConstPtr& start_state,
                                           float timeout) {
    restart();
    search(start_state, timeout);
  }

  void MCTS::performPostActionProcessing(const State::ConstPtr& state,
                                         const Action::ConstPtr& action,
                                         float timeout) {
    restart();
    last_action_selected_ = action;
    search(state, action, timeout);
  }

  void MCTS::restart() {
    root_node_.reset();
  }

  Action::ConstPtr MCTS::getPlanningAction(const State::ConstPtr& state,
                                           const StateNode::ConstPtr& state_node) const {

    std::vector<Action::ConstPtr> best_actions;

    if (params_.action_selection_strategy == UCT) {
      typedef std::pair<Action::ConstPtr, StateActionNode::ConstPtr> Action2StateActionInfoPair;

      // TODO change to assert.
      if (state_node->state_visits == 0) {
        throw std::runtime_error("getPlanningAction() called with a state where no visits in state info!");
      }

      float max_value = -std::numeric_limits<float>::max();
      std::vector<Action::ConstPtr> best_actions;
      BOOST_FOREACH(const Action2StateActionInfoPair& action_info_pair, state_node->actions) {
        float planning_bound = 1e10f;
        if (action_info_pair.second->visits != 0) {
          planning_bound = action_info_pair.second->mean_value +
            params_.uct_reward_bound * sqrtf(logf(state_node->state_visits) / action_info_pair.second->visits);
        }
        if (fabs(planning_bound - max_value) < 1e-10f) {
          best_actions.push_back(action_info_pair.first);
        } else if (planning_bound > max_value) {
          max_value = planning_bound;
          best_actions.clear();
          best_actions.push_back(action_info_pair.first);
        }
      }
    } else {
      throw IncorrectUsageException("MCTS: Unknown backup strategy provided in parameter set.");
    }

    return best_actions[rng_->randomInt(best_actions.size() - 1)];
  }

  void MCTS::updateState(HistoryStep& step, float& backup_value) {

    /* Update mean value for this action. */
    StateNode::Ptr& state_info = step.state;
    ++(state_info->state_visits);
    StateActionNode::Ptr& action_info = state_info->actions[step.action];
    ++(action_info->visits);
    action_info->mean_value += (1.0 / action_info->visits) * (backup_value - action_info->mean_value);

    if (params_.backup_strategy == ELIGIBILITY_TRACE) {
      float max_value = maxValueForState(state_info);
      backup_value = params_.eligibility_lambda * backup_value + (1.0 - params_.eligibility_lambda) * max_value;
    } else {
      throw IncorrectUsageException("MCTS: Unknown backup strategy provided in parameter set.");
    }

  }

  StateNode::Ptr MCTS::getNewStateNode(const State::ConstPtr& state) {
    std::vector<Action::ConstPtr> actions;
    model_->getActionsAtState(state, actions);
    StateNode::Ptr state_node(new StateNode);
    BOOST_FOREACH(const Action::ConstPtr& action, actions) {
      StateActionNode::Ptr state_action_node(new StateActionNode);
      state_node->actions[action] = state_action_node;
    }
    return state_node;
  }

} /* utexas_planning */
