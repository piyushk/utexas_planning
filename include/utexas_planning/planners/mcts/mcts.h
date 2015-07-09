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

    inline float getMaxValueForState(const StateNode::ConstPtr& state) const {
      float max_value = -std::numeric_limits<float>::max();
      BOOST_FOREACH(const StateActionNode::ConstPtr& state_action, state.actions) {
        float val = getStateActionValue(state_action);
        max_value = std::max(val, max_value);
      }
      return max_value;
    }

    virtual Action::ConstPtr getPlanningAction(const State::ConstPtr& state,
                                               const StateNode::ConstPtr& state_info);

    /* TODO add a function to do different types of backups here. */

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
      }

      at_start_state = false;
    }

    MCTS_OUTPUT("------------ BACKPROPAGATION --------------");
    float backpropValue = 0;
    // TODO: does this mean that the unknown bootstrap value paramhas no significance?
    backpropValue = p.unknownBootstrapValue;

    MCTS_OUTPUT("At final discretized state: " << discretizedState << " the backprop value is " << backpropValue);

    for (int step = history.size() - 1; step >= 0; --step) {

      // Get information about this state
      typename StateInfoTable::iterator& state_info = history[step].state_info;
      unsigned int& action_id = history[step].action_id;
      float& reward = history[step].reward;

      MCTS_OUTPUT("Reviewing state: " << state_info->first << " with reward: " << reward);

      backpropValue = reward + p.gamma * backpropValue;

      MCTS_OUTPUT("Total backprop value: " << backpropValue);

      if (history[step].update_this_state) {

        stateCount[state_info->first]--;
        // Modify the action appropriately
        if (stateCount[state_info->first] == 0) { // First Visit Monte Carlo
          ++(state_info->second.state_visits);
          StateActionInfo& action_info = state_info->second.action_infos[action_id];
          (action_info.visits)++;
          action_info.val += (1.0 / action_info.visits) * (backpropValue - action_info.val);
          MCTS_OUTPUT("  Set value of action " << action_id << " to " << action_info.val);
        }

        if (state_info->second.state_visits > 1) {
          float maxValue = maxValueForState(state_info->first, state_info->second);
          MCTS_OUTPUT("  Interpolating backpropagation value between current " << backpropValue << " and max " << maxValue);
          backpropValue = p.lambda * backpropValue + (1.0 - p.lambda) * maxValue;
        } // else don't change the value being backed up
      }

      MCTS_OUTPUT("  At state: " << state_info->first << " the backprop value is " << backpropValue);
    }

    MCTS_OUTPUT("State Table: " << std::endl << getStateTableDescription());
#ifdef MCTS_DEBUG
    if (++count == 10) throw std::runtime_error("argh!");
#endif

    ++current_playouts;
    current_time = boost::posix_time::microsec_clock::local_time();
  }

  start_action_available_ = false;

  termination_count = terminatedPlayouts;
  return currentPlayouts;
}

template<class State, class StateHash, class Action>
Action MultiThreadedMCTS<State, StateHash, Action>::getBestAction(const State& state) {
  State mappedState(state);
  stateMapping->map(mappedState); // discretize state
#ifdef MCTS_VALUE_DEBUG
  std::cout << "    " << getStateValuesDescription(mappedState) << std::endl;
#endif
  boost::shared_ptr<RNG> rng(new RNG(masterRng->randomUInt()));
  HistoryStep unused_step;
  unsigned int unused_new_states_counter = 0;


  return selectAction(mappedState, false, unused_step, unused_new_states_counter, rng);
}

template<class State, class StateHash, class Action>
void MultiThreadedMCTS<State, StateHash, Action>::printBestTrajectory(const State& state, Action action) {
  // Don't remove the code, this generates the string for the best trajectory given the current state table.
  std::stringstream ss;
  ss << "    Best Trajectory: " << std::endl;
  State start_state = state;
  State discretized_state = start_state;
  stateMapping->map(discretized_state);
  int depth = 0;
  bool terminal = false;
  float total_reward = 0.0f;
  bool first = true;
  while(depth < p.maxDepth && !terminal) {
    // Select action, take it and update the model with the action taken in simulation.
    ss << "      State: " << start_state << std::endl;
    ss << "      Discretized State: " << discretized_state << ", Depth: " << depth << std::endl;
    ss << "    " << getStateValuesDescription(discretized_state) << std::endl;
    HistoryStep unused_step;
    unsigned int unused_new_states_counter = 0;
    if (!first) {
      action = selectAction(discretized_state, false, unused_step, unused_new_states_counter, masterRng);
    }
    std::vector<Action> stateActions;
    model->getAllActions(discretized_state, stateActions);
    ss << "      Default Policy Suggests: " << defaultPolicy->getBestAction(discretized_state, stateActions, masterRng) << std::endl;
    State next_state;
    float reward;
    int depth_count;
    model->takeAction(start_state, action, reward, next_state, terminal, depth_count, masterRng);
    total_reward += reward;
    depth += depth_count;
    ss << "      Action Selected: " << action << ", Reward: " << reward << std::endl;
    start_state = next_state;
    discretized_state = next_state;
    stateMapping->map(discretized_state);
    first = false;
  }
  std::cout << ss.str();
  ss << "    Total Reward in best trajectory: " << total_reward << std::endl;
}

template<class State, class StateHash, class Action>
Action MultiThreadedMCTS<State, StateHash, Action>::selectAction(const State& state,
    bool use_planning_bound, HistoryStep& history_step, unsigned int& new_states_added_in_rollout,
    boost::shared_ptr<RNG>& rng) {

  // Get all the actions available at this state.
  std::vector<Action> stateActions;
  model->getAllActions(state, stateActions);

  // There are three cases here
  //  1. The state information exists, and we should use UCT action selection to choose.
  //  2. The state does not exist in the state info table and needs to be added.
  //  3. The state does not exist in the state info table and does not need to be added.

  history_step.state_info = stateInfoTable.find(state);
  if (history_step.state_info != stateInfoTable.end()) {
    // It may be possible that we hit this state for the first time in this rollout itself.
    // Use the default policy if state visits are zero (i.e. we have not back-propogated yet).
    if (history_step.state_info->second.state_visits != 0) {
      float maxVal = -std::numeric_limits<float>::max();
      std::vector<unsigned int> maxActionIdx;
      unsigned int currentActionIdx = 0;
      BOOST_FOREACH(const StateActionInfo& action_info, history_step.state_info->second.action_infos) {
        float val = calcActionValue(action_info, history_step.state_info->second, use_planning_bound);
        if (fabs(val - maxVal) < 1e-10) {
          maxActionIdx.push_back(currentActionIdx);
        } else if (val > maxVal) {
          maxVal = val;
          maxActionIdx.clear();
          maxActionIdx.push_back(currentActionIdx);
        }
        ++currentActionIdx;
      }
      history_step.action_id = maxActionIdx[rng->randomInt(maxActionIdx.size() - 1)];
    } else {
      history_step.action_id = defaultPolicy->getBestAction(state, stateActions, rng);
    }
    history_step.update_this_state = true;
  } else if ((new_states_added_in_rollout < p.maxNewStatesPerRollout) || (p.maxNewStatesPerRollout == 0)) {
    // Add the state + state-action, and choose an action using the default policy.
    StateInfo new_state_info(stateActions.size(), 0);
    bool unused_bool;
    boost::tie(history_step.state_info, unused_bool) =
      stateInfoTable.insert(std::pair<State, StateInfo>(state, new_state_info));
    history_step.action_id = defaultPolicy->getBestAction(state, stateActions, rng);
    history_step.update_this_state = true;
    ++new_states_added_in_rollout;
  } else {
    // Just store the history step since it's not necessary to add the state-action pair.
    history_step.action_id = defaultPolicy->getBestAction(state, stateActions, rng);
    history_step.update_this_state = false;
  }

  return stateActions[history_step.action_id];
}

template<class State, class StateHash, class Action>
float MultiThreadedMCTS<State, StateHash, Action>::maxValueForState(const State& state,
    const StateInfo& state_info) {

  int idx;
  float maxVal = -std::numeric_limits<float>::max();
  BOOST_FOREACH(const StateActionInfo& stateActionInfo, state_info.action_infos) {
    float val = calcActionValue(stateActionInfo, state_info, false);
    if (val > maxVal) {
      maxVal = val;
    }
  }

  return maxVal;
}

template<class State, class StateHash, class Action>
void MultiThreadedMCTS<State, StateHash, Action>::restart() {
  stateInfoTable.clear();
}

template<class State, class StateHash, class Action>
std::string MultiThreadedMCTS<State, StateHash, Action>::generateDescription(unsigned int indentation) {
  std::stringstream ss;
  std::string prefix = indent(indentation);
  std::string prefix2 = indent(indentation + 1);
  ss << prefix  << "MultiThreadedMCTS: " << std::endl;
  ss << prefix2 << p << std::endl;
  return ss.str();
}

template<class State, class StateHash, class Action>
std::string MultiThreadedMCTS<State, StateHash, Action>::getStateValuesDescription(const State& state) {
  std::stringstream ss;
  typename StateInfoTable::const_iterator a = stateInfoTable.find(state);
  if (a == stateInfoTable.end()) { // The state does not exist, choose an action randomly
    ss << state << ": Not in table!";
    return ss.str();
  }

  float maxVal = maxValueForState(state, a->second);
  ss << state << " " << maxVal << "(" << a->second.state_visits << "): " << std::endl;
  unsigned int count = 0;
  std::vector<Action> actions;
  model->getAllActions(state, actions);
  BOOST_FOREACH(const StateActionInfo& action_info, a->second.action_infos) {
    float val = calcActionValue(action_info, a->second, false);
    unsigned int na = action_info.visits;
    ss << "      #" << actions[count] << " " << val << "(" << na << ")" << std::endl;
    ++count;
  }

  return ss.str();
}

template<class State, class StateHash, class Action>
std::string MultiThreadedMCTS<State, StateHash, Action>::getStateTableDescription() {
  std::stringstream ss;
  int count = 0;
  for (typename StateInfoTable::const_iterator v = stateInfoTable.begin();
      v != stateInfoTable.end(); ++v) {
    ss << "State #" << count << ": " << getStateValuesDescription(v->first) << std::endl;
    ++count;
  }
  return ss.str();
}


#endif /* end of include guard: UTEXAS_PLANNING_MCTS_H_ */
