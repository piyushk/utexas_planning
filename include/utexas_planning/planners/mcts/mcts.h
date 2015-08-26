#ifndef UTEXAS_PLANNING_MCTS_H_
#define UTEXAS_PLANNING_MCTS_H_

#include <map>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <utexas_planning/common/params.h>

#include <utexas_planning/core/abstract_planner.h>
#include <utexas_planning/core/action.h>
#include <utexas_planning/core/state.h>

namespace utexas_planning {

  class StateActionNode;

  class StateNode {
    public:

      typedef boost::shared_ptr<StateNode> Ptr;
      typedef boost::shared_ptr<StateNode const> ConstPtr;

      StateNode() : state_visits(0) {}
      std::map<Action::ConstPtr, boost::shared_ptr<StateActionNode>, Action::PtrComparator> actions;
      State::ConstPtr state;
      unsigned int state_visits;
  };

  class StateActionNode {
    public:
      typedef boost::shared_ptr<StateActionNode> Ptr;
      typedef boost::shared_ptr<StateActionNode const> ConstPtr;

      StateActionNode(unsigned int visits = 0, float mean_value = 0.0f) :
        visits(visits), mean_value(mean_value), variance(0), sum_squares(0) {}
      std::map<State::ConstPtr, StateNode::Ptr, State::PtrComparator> next_states;
      unsigned int visits;
      float mean_value;
      float variance;
      float sum_squares;
  };

  const int NO_MAX_PLAYOUTS = -1;
  const std::string UCT = "uct";
  const std::string THOMPSON = "thompson";
  const std::string ELIGIBILITY_TRACE = "eligibility";

  class MCTS : public AbstractPlanner {

    public:

      class HistoryStep {
        public:
          typename StateNode::Ptr state;
          Action::ConstPtr action;
          float reward;
      };


#define PARAMS(_) \
      _(unsigned int,max_depth,max_depth,0) \
      _(float,gamma,gamma,1.0) \
      _(float,max_new_states_per_rollout,max_new_states_per_rollout,0) \
      _(float,max_playouts,max_playouts,NO_MAX_PLAYOUTS) \
      _(float,unknown_action_value,unknown_action_value,-1e10f) \
      _(std::string,action_selection_strategy,action_selection_strategy,UCT) \
      _(float,uct_reward_bound,uct_reward_bound,10000) \
      _(int,thompson_initial_random_trials,thompson_initial_random_trials,5) \
      _(std::string,backup_strategy,backup_strategy,ELIGIBILITY_TRACE) \
      _(float,eligibility_lambda,eligibility_lambda,0.0) \

      Params_STRUCT(PARAMS)
#undef PARAMS

      virtual ~MCTS() {}

      virtual void init(const GenerativeModel::ConstPtr& model,
                        const YAML::Node& params,
                        const std::string& output_directory,
                        const boost::shared_ptr<RNG>& rng,
                        bool verbose = false);

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
      virtual std::string getName() const;

    protected:

      inline float getStateActionValue(const StateActionNode::ConstPtr& state_action) const {
        return (state_action->visits == 0) ? params_.unknown_action_value : state_action->mean_value;
      }

      inline float maxValueForState(const StateNode::ConstPtr& state_node) const {
        typedef std::pair<Action::ConstPtr, StateActionNode::ConstPtr> Action2StateActionInfoPair;
        float max_value = -std::numeric_limits<float>::max();
        BOOST_FOREACH(const Action2StateActionInfoPair& action_info_pair, state_node->actions) {
          float val = getStateActionValue(action_info_pair.second);
          max_value = std::max(val, max_value);
        }
        return max_value;
      }

      virtual void restart();
      virtual Action::ConstPtr getPlanningAction(const State::ConstPtr& state,
                                                 const StateNode::ConstPtr& state_info) const;
      virtual void updateState(HistoryStep& step, float& backup_value);
      virtual StateNode::Ptr getNewStateNode(const State::ConstPtr& state);

      std::string getStateValuesDescription(const State& state);
      std::string getStateTableDescription();

      GenerativeModel::ConstPtr model_;
      AbstractPlanner::Ptr default_planner_;

      StateNode::Ptr root_node_;
      Action::ConstPtr start_action_;
      bool start_action_available_;
      Action::ConstPtr last_action_selected_;

      Params params_;
      boost::shared_ptr<RNG> rng_;

      bool verbose_;

  };

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_MCTS_H_ */
