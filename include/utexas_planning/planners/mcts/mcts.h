#ifndef UTEXAS_PLANNING_MCTS_H_
#define UTEXAS_PLANNING_MCTS_H_

#include <map>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <utexas_planning/common/params.h>

#include <utexas_planning/core/abstract_planner.h>
#include <utexas_planning/core/action.h>
#include <utexas_planning/core/state.h>
#include <utexas_planning/execution/class_loader.h>

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

      StateActionNode(unsigned int visits = 0,
                      float mean_value = 0.0f,
                      float variance = 0.0f,
                      float sum_squares = 0.0f,
                      float alpha = 1.0f,
                      float beta = 1.0f) :
        visits(visits),
        mean_value(mean_value),
        variance(variance),
        sum_squares(sum_squares),
        alpha(alpha),
        beta(beta) {}
      std::map<State::ConstPtr, StateNode::Ptr, State::PtrComparator> next_states;
      unsigned int visits;
      float mean_value;
      float variance;
      float sum_squares;
      float alpha;
      float beta;
  };

  const int NO_MAX_PLAYOUTS = -1;
  const std::string UCT = "uct";
  const std::string UCT_TUNED = "uct_tuned";
  const std::string THOMPSON = "thompson";
  const std::string THOMPSON_BETA = "thompson_beta";
  const std::string RANDOM = "random";
  const std::string HIGHEST_MEAN = "mean";
  const std::string UNIFORM = "uniform";

  const std::string BACKUP_LAMBDA_SARSA = "backup_lambda_sarsa";
  const std::string BACKUP_GAMMA_SARSA = "backup_gamma_sarsa";
  const std::string BACKUP_LAMBDA_Q = "backup_lambda_q";
  const std::string BACKUP_GAMMA_Q = "backup_gamma_q";

  const std::string BACKUP_OMEGA_SARSA = "backup_omega_sarsa";
  const std::string BACKUP_OMEGA_Q = "backup_omega_q";

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
      _(int,thompson_beta_max_reward,thompson_beta_max_reward,100) \
      _(int,thompson_beta_min_reward,thompson_beta_min_reward,-100) \
      _(int,mean_initial_random_trials,mean_initial_random_trials,5) \
      _(std::string,backup_strategy,backup_strategy,BACKUP_LAMBDA_Q) \
      _(float,backup_lambda_value,backup_lambda_value,0.0) \
      _(unsigned int,backup_gamma_max_depth,backup_gamma_max_depth,100) \
      _(std::string,default_policy,default_policy,"utexas_planning::RandomPlanner") \

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

      virtual void performPreActionProcessing(const State::ConstPtr& state,
                                              const Action::ConstPtr& prev_action = Action::ConstPtr(),
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

      void calculateOmegaWeightsFromParams();

      std::string getStateValuesDescription(const State& state);
      std::string getStateTableDescription();

      GenerativeModel::ConstPtr model_;
      ClassLoader::Ptr loader_;
      AbstractPlanner::Ptr default_planner_;

      StateNode::Ptr root_node_;
      Action::ConstPtr start_action_;
      bool start_action_available_;
      Action::ConstPtr last_action_selected_;

      Params params_;
      boost::shared_ptr<RNG> rng_;

      bool verbose_;

      /* Fixed coefficients required for the gamma return. */
      /* They correspond to Eqn 16 in
       * http://papers.nips.cc/paper/4472-td_gamma-re-evaluating-complex-backups-in-temporal-difference-learning.pdf
       * the outer indexing represents different L, and the inner indexing represents different n. */
      std::vector<std::vector<float> > gamma_return_coefficients_;

      /* Variable coefficients required for the omega return */
      std::vector<float> omega_return_coefficients_;
      int omega_max_encountered_trajectory_length_;
      int omega_num_sample_trajectories_;
      std::vector<float> omega_nreturn_sum_squares_;
      std::vector<float> omega_nreturn_sum_;
      std::vector<float> omega_nreturn_variance_;

      float omega_vl_;
      float omega_vplus_;
      float omega_k1_;
      float omega_k2_;
      bool omega_values_initialized_;

  };

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_MCTS_H_ */
