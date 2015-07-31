#ifndef UTEXAS_PLANNING_MCTS_H_
#define UTEXAS_PLANNING_MCTS_H_

#include <boost/foreach.hpp>
#include <boost/thread/thread.hpp>
#include <boost/tuple/tuple.hpp>

#include <bwi_tools/common/RNG.h>
#include <bwi_tools/common/Params.h>
#include <bwi_tools/common/Util.h>

#include <bwi_rl/planning/DefaultPolicy.h>
#include <bwi_rl/planning/Model.h>
/* #include <bwi_rl/planning/MultiThreadedMCTSEstimator.h> */
#include <bwi_rl/planning/StateMapping.h>

#ifdef MCTS_DEBUG
#define MCTS_OUTPUT(x) std::cout << x << std::endl
#else
#define MCTS_OUTPUT(x) ((void) 0)
#endif

class StateActionNode;

class StateNode {
  public:

    typdef boost::shared_ptr<StateNode> Ptr;
    typdef boost::shared_ptr<StateNode const> ConstPtr;

    StateNode(unsigned int num_actions = 0) : action_infos(num_actions), state_visits(0) {}
    std::map<Action::ConstPtr, StateActionNode::Ptr> actions;
    unsigned int state_visits;
};

class StateActionNode {
  public:
    StateActionNode(unsigned int visits = 0, float val = 0.0f) : visits(visits), mean_value(mean_value) {}
    std::map<State::ConstPtr, StateNode::Ptr> next_states;
    unsigned int visits;
    float mean_value;
};

class MCTS : public AbstractPlanner {

  public:

    class HistoryStep {
      public:
        typename StateNode::Ptr state;
        Action::ConstPtr action;
        float reward;
    };

    static const int NO_MAX_PLAYOUTS = -1;

#define PARAMS(_) \
    _(unsigned int,maxDepth,maxDepth,0) \
    _(float,lambda,lambda,0.0) \
    _(float,gamma,gamma,1.0) \
    _(float,rewardBound,rewardBound,10000) \
    _(float,maxNewStatesPerRollout,maxNewStatesPerRollout,0) \
    _(float,unknownActionValue,unknownActionValue,-1e10) \
    _(float,unknownBootstrapValue,unknownBootstrapValue,0.0) \

    Params_STRUCT(PARAMS)
#undef PARAMS

    virtual ~MCTS() {}

    virtual void init(const GenerativeModel::ConstPtr& model,
                      const YAML::Node& params,
                      const std::string& output_directory,
                      const boost::shared_ptr<RNG>& rng) = 0;

    virtual void search(const State::ConstPtr& start_state,
                        float timeout = 1.0,
                        int max_playouts = NO_MAX_PLAYOUTS);
    virtual void search(const State::ConstPtr& start_state,
                        const Action::ConstPtr& start_action,
                        float timeout = 1.0,
                        int max_playouts = NO_MAX_PLAYOUTS);

    virtual Action::ConstPtr getBestAction(const State::ConstPtr& state) const;

    virtual void performEpisodeStartProcessing(const State::ConstPtr& start_state,
                                               float timeout = NO_TIMEOUT);

    virtual void performPostActionProcessing(const State::ConstPtr& state,
                                             const Action::ConstPtr& action,
                                             float timeout = NO_TIMEOUT);

    virtual std::map<std::string, std::string> getParamsAsMap() const;

  protected:

    inline float getStateActionValue(const StateActionNode::ConstPtr &state_action) const {
      return (state_action->visits == 0) ? p.unknownActionValue : state_action->mean_value;
    }

    virtual Action::ConstPtr getPlanningAction(const State::ConstPtr& state,
                                               const StateNode::ConstPtr& state_info) const = 0;
    virtual void updateState(const HistoryStep& step, float cummulative_value) = 0;

    std::string getStateValuesDescription(const State& state);
    std::string getStateTableDescription();

    GenerativeModel::ConstPtr model_;
    AbstractPlanner::ConstPtr default_planner_;

    State start_state_;
    Action start_actions_;
    bool start_action_available_;

    Params params_;
    boost::shared_ptr<RNG> rng_;

};

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
  boost::posix_time::ptime end_time = boost::posix_time::microsec_clock::local_time() +
      boost::posix_time::milliseconds(timeout * 1000);
  boost::posix_time::ptime current_time = start_time;

  while (((timeout <= 0.0) || (current_time < end_time)) &&
         ((max_playouts <= 0) || (current_playouts < max_playouts))) {

    MCTS_OUTPUT("------------START ROLLOUT--------------");
    std::vector<HistoryStep> history;
    State::ConstPtr state = start_state;

    State::Ptr discretized_state = start_state->clone();
    // TODO remove root node when clearing search.
    if (!root_node_) {
      root_node_ = getNewStateNode(discretized_state);
    }
    StateNode::Ptr state_node = root_node_;

    State::Ptr next_state;
    Action::ConstPtr action;
    unsigned int action_id;
    int depth_count;
    float reward

    bool at_start_state = true;

    // TODO state mapping.
    // stateMapping->map(discretizedState); // discretize state

    unsigned int new_states_added_in_rollout = 0;

    // Version of the code with no max depth (and hence no pre-cached history memory).
    for (unsigned int depth = 0;
         ((timeout <= 0.0) || (current_time < end_time));
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

      model_->takeAction(state, action, reward, next_state, depth_count, rng);

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
        if ((params.maxNewStatesPerRollout == 0) ||
            (new_states_added_in_rollout < params.maxNewStatesPerRollout)) {
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
    float backpropValue = 0;
    // TODO: does this mean that the unknown bootstrap value param has no significance?
    /* backpropValue = p.unknownBootstrapValue; */

    MCTS_OUTPUT("At final discretized state: " << discretizedState << " the backprop value is " << backpropValue);

    for (int step = history.size() - 1; step >= 0; --step) {

      backpropValue = history[step].reward + p.gamma * backpropValue;

      if (history[step].state) {
        updateState(history[step], backpropValue);

        // stateCount[state_info->first]--;
        // // Modify the action appropriately
        // if (stateCount[state_info->first] == 0) { // First Visit Monte Carlo
        //   ++(state_info->second.state_visits);
        //   StateActionInfo& action_info = state_info->second.action_infos[action_id];
        //   (action_info.visits)++;
        //   action_info.val += (1.0 / action_info.visits) * (backpropValue - action_info.val);
        //   MCTS_OUTPUT("  Set value of action " << action_id << " to " << action_info.val);
        // }

        // if (state_info->second.state_visits > 1) {
        //   float maxValue = maxValueForState(state_info->first, state_info->second);
        //   MCTS_OUTPUT("  Interpolating backpropagation value between current " << backpropValue << " and max " << maxValue);
        //   backpropValue = p.lambda * backpropValue + (1.0 - p.lambda) * maxValue;
        // } // else don't change the value being backed up
      }

    }

    ++current_playouts;
    current_time = boost::posix_time::microsec_clock::local_time();
  }

  start_action_available_ = false;

  termination_count = terminatedPlayouts;
  return currentPlayouts;
}

virtual Action::ConstPtr MCTS::getBestAction(const State::ConstPtr& state) const {

  float max_value = -std::numeric_limits<float>::max();
  typedef std::pair<Action::ConstPtr, StateActionNode::ConstPtr> Action2StateActionInfoPair;
  std::vector<Action::ConstPtr> best_actions;
  BOOST_FOREACH(const Action2StateActionInfoPair& action_info_pair, state.actions) {
    float val = getStateActionValue(action_info_pair.second);
    if (fabs(val - max_value) < 1e-10) {
      best_actions.push_back(action_info_pair.first);
    } else if (val > max_value) {
      max_value = val;
      best_actions.clear();
      best_actions.push_back(action_info_pair.first);
    }
  }

  return best_actions[rng->randomInt(best_actions.size() - 1)];
}

virtual void MCTS::performEpisodeStartProcessing(const State::ConstPtr& start_state,
                                                 float timeout = NO_TIMEOUT) {
  restart();
  search(start_state, timeout);
}

virtual void MCTS::performPostActionProcessing(const State::ConstPtr& state,
                                               const Action::ConstPtr& action,
                                               float timeout = NO_TIMEOUT) {
  restart();
  search(state, action, timeout);
}

void MCTS::restart() {
  root_node_.reset();
}

#endif /* end of include guard: UTEXAS_PLANNING_MCTS_H_ */
