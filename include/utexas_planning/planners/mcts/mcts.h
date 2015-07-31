#ifndef UTEXAS_PLANNING_MCTS_H_
#define UTEXAS_PLANNING_MCTS_H_

#include <boost/foreach.hpp>

namespace utexas_planning {

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
                        const boost::shared_ptr<RNG>& rng);

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

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_MCTS_H_ */
