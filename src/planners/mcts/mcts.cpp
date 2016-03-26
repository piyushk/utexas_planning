#include <boost/date_time/posix_time/posix_time_types.hpp>

#include <boost/math/special_functions/beta.hpp>
#include <boost/math/special_functions/digamma.hpp>

#include <class_loader/class_loader.h>

#include <Eigen/Core>
#include <Eigen/LU>

#include <utexas_planning/common/exceptions.h>
#include <utexas_planning/common/least_squares.h>
#include <utexas_planning/planners/mcts/mcts.h>
#include <utexas_planning/planners/random/random_planner.h>

#ifdef MCTS_DEBUG
#define MCTS_DEBUG_OUTPUT(x) std::cout << x << std::endl
#else
#define MCTS_DEBUG_OUTPUT(x) ((void) 0)
#endif

#define MCTS_VERBOSE_OUTPUT(x) if (verbose_) std::cout << x << std::endl

namespace utexas_planning {

  void MCTS::init(const GenerativeModel::ConstPtr& model,
                  const YAML::Node& params,
                  const std::string& output_directory,
                  const boost::shared_ptr<RNG>& rng,
                  bool verbose) {
    model_ = model;
    params_.fromYaml(params);

    // TODO parametrize the default planner using ClassLoader
    default_planner_.reset(new RandomPlanner);
    default_planner_->init(model, params, output_directory, rng);

    rng_ = rng;
    verbose_ = verbose;
    start_action_available_ = false;

    // Compute gamma return coefficients up to max depth.
    if (params_.backup_strategy == BACKUP_GAMMA_SARSA ||
        params_.backup_strategy == BACKUP_GAMMA_Q ||
        params_.backup_strategy == BACKUP_OMEGA_SARSA ||
        params_.backup_strategy == BACKUP_OMEGA_Q) {

      if (params_.backup_gamma_max_depth <= 0) {
        params_.backup_gamma_max_depth = params_.max_depth;
      }

      if (params_.backup_gamma_max_depth <= 0) {
        throw IncorrectUsageException("MCTS: Both backup_gamma_max_depth and max_depth cannot be <= 0 when gamma "
                                      "backup strategy is used. You should at-least set backup_gamma_max_depth");
      }

      gamma_return_coefficients_.resize(params_.backup_gamma_max_depth);
      /* std::cout << "Printing gamma return coefficients: " << std::endl; */
      for (int L = 1; L <= params_.backup_gamma_max_depth; ++L) {
        gamma_return_coefficients_[L - 1].resize(L);
        float weight_sum = 0;
        for (int n = 1; n <= L; ++n) {
          float weight;
          if (params_.gamma == 1.0f) {
            weight = 1.0f / n;
          } else {
            weight = (1.0f - powf(params_.gamma, 2)) / (1.0f - powf(params_.gamma, 2 * n));
          }
          weight_sum += weight;
          gamma_return_coefficients_[L - 1][n - 1] = weight;
        }
        // if (L < 5) {
        //   std::cout << "L=" << L << ": ";
        // }
        for (int n = 1; n <= L; ++n) {
          gamma_return_coefficients_[L - 1][n - 1] /= weight_sum;
          // if (L < 5) {
          //   std::cout << gamma_return_coefficients_[L - 1][n - 1] << " ";
          // }
        }
        // if (L < 5) {
        //   std::cout << std::endl;
        // }
        // if (L == 4) {
        //   throw std::runtime_error("blah!");
        // }
      }
    }
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

    /* Re-init omega values every search */
    omega_values_initialized_ = false;
    omega_nreturn_sum_.clear();
    omega_nreturn_sum_squares_.clear();
    omega_nreturn_variance_.clear();
    omega_max_encountered_trajectory_length_ = 0;
    omega_num_sample_trajectories_ = 0;

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

      MCTS_DEBUG_OUTPUT("------------START ROLLOUT--------------");
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
           ((params_.max_depth == 0) || (depth < params_.max_depth) &&
            (timeout <= 0.0) || (current_time < end_time));
           depth += depth_count) {

        // Select action, take it and update the model with the action taken in simulation.
        MCTS_DEBUG_OUTPUT("MCTS State: " << *state << " " << "DEPTH: " << depth);

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

        MCTS_DEBUG_OUTPUT(" Action Selected: " << *action);
        MCTS_DEBUG_OUTPUT("   Reward: " << reward);

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

      MCTS_DEBUG_OUTPUT("------------ BACKPROPAGATION --------------");

      // TODO: Double check to see if the current history length exceeded the omega_max_encountered_trajectory_length,
      // and if so, recompute the omega weights.

      if (params_.backup_strategy == BACKUP_LAMBDA_Q || params_.backup_strategy == BACKUP_LAMBDA_SARSA) {

        float backup_value = 0;

        MCTS_DEBUG_OUTPUT("At final discretized state: " << *discretized_state << " the backprop value is " << backup_value);

        for (int step = history.size() - 1; step >= 0; --step) {
          backup_value = history[step].reward + params_.gamma * backup_value;
          if (history[step].state) {
            MCTS_DEBUG_OUTPUT("  Updating state-action " << *(history[step].state->state) << " " <<
                              *(history[step].action) << " with backup value " << backup_value);
            updateState(history[step], backup_value);

            // MCTS_DEBUG_OUTPUT("    After update: " << action_info->mean_value << "+-" << action_info->variance << " (" <<
            //                   action_info->visits << ")");

            // Prepare backup using eligiblity trace methodology.
            if (params_.backup_lambda_value != 1.0f) {
              StateNode::Ptr& state_info = history[step].state;
              float interpolation_value;
              if (params_.backup_strategy == BACKUP_LAMBDA_Q) {
                interpolation_value = maxValueForState(state_info);
              } else { /* params_.backup_strategy == BACKUP_SARSA_Q */
                // Get the up-to-date value for the state-action.
                StateActionNode::Ptr& action_info = state_info->actions[history[step].action];
                interpolation_value = action_info->mean_value;
              }

              // Update the backup value.
              backup_value =
                (params_.backup_lambda_value * backup_value) +
                ((1.0f - params_.backup_lambda_value) * interpolation_value);
            }
          }
        }

      } else if (params_.backup_strategy == BACKUP_GAMMA_Q || params_.backup_strategy == BACKUP_GAMMA_SARSA ||
                 params_.backup_strategy == BACKUP_OMEGA_Q || params_.backup_strategy == BACKUP_OMEGA_SARSA) {

        std::vector<float> return_array(history.size(), 0.0f);
        float last_interpolation_value = 0.0f;

        for (int step = history.size() - 1; step >= 0; --step) {
          for (int step_diff = 0; step_diff < history.size() - step; ++step_diff) {
            return_array[step_diff] += history[step].reward;
          }
          return_array[history.size() - 1 - step] += last_interpolation_value;

          if (step == 0 &&
              (params_.backup_strategy == BACKUP_OMEGA_Q || params_.backup_strategy == BACKUP_OMEGA_SARSA)) {

            // Get the omega nreturn tally to the right point.
            if (omega_num_sample_trajectories_ == 0) {
              omega_nreturn_sum_.resize(history.size(), 0.0f);
              omega_nreturn_sum_squares_.resize(history.size(), 0.0f);
            } else if (history.size() > omega_max_encountered_trajectory_length_) {
              omega_nreturn_sum_.resize(history.size(), omega_nreturn_sum_.back());
              omega_nreturn_sum_squares_.resize(history.size(), omega_nreturn_sum_squares_.back());
            }

            // Increment the number of trajectories.
            ++omega_num_sample_trajectories_;
            // Add to the mean and sum
            for (int i = 0; i < omega_nreturn_sum_.size(); ++i) {
              int nreturn_idx = omega_nreturn_sum_.size() - i - 1;
              if (nreturn_idx >= history.size()) {
                nreturn_idx = history.size() - 1;
              }
              omega_nreturn_sum_[i] += return_array[nreturn_idx];
              omega_nreturn_sum_squares_[i] += return_array[nreturn_idx] * return_array[nreturn_idx];
            }

          }

          if (history[step].state) {

            float sample_value = 0.0f;
            /* std::cout << "Calculating sample value: " << std::endl; */
            for (int mult_idx = 0; mult_idx < history.size() - step; ++mult_idx) {
              /* std::cout << "    " << return_array[mult_idx] << " " << gamma_return_coefficients_[history.size() - step - 1][history.size() - step - 1 - mult_idx] << std::endl; */
              float coefficient =
                gamma_return_coefficients_[history.size() - step - 1][history.size() - step - 1 - mult_idx];
              if (omega_values_initialized_ &&
                  (params_.backup_strategy == BACKUP_OMEGA_Q || params_.backup_strategy == BACKUP_OMEGA_SARSA)) {
                coefficient =
                  omega_return_coefficients_[history.size() - step - 1 - mult_idx];
              }
              sample_value += return_array[mult_idx] * coefficient;
            }

            MCTS_DEBUG_OUTPUT("  Updating state-action " << *(history[step].state->state) << " " <<
                              *(history[step].action) << " with sample value " << sample_value);
            updateState(history[step], sample_value);

            // MCTS_DEBUG_OUTPUT("    After update: " << action_info->mean_value << "+-" << action_info->variance << " (" <<
            //                   action_info->visits << ")");

            // Prepare backup using eligiblity trace methodology.
            StateNode::Ptr& state_info = history[step].state;
            float interpolation_value;
            if (params_.backup_strategy == BACKUP_GAMMA_Q) {
              last_interpolation_value = maxValueForState(state_info);
            } else /* params_.backup_strategy == BACKUP_SARSA_Q */ {
              // Get the up-to-date value for the state-action.
              StateActionNode::Ptr& action_info = state_info->actions[history[step].action];
              last_interpolation_value = action_info->mean_value;
            }

          } else {
            // Provide the monte carlo estimate for the last return value if this state is not yet being tracked.
            last_interpolation_value = return_array[0];
          }
        }

      } else /* unknown backup strategy - probably set incorrectly in params file */ {
        throw IncorrectUsageException(std::string("MCTS: Unknown backup strategy provided in parameter set: ") +
                                      params_.backup_strategy);
      }

      ++current_playouts;
      // TODO parametrize based on number of planning simulations.
      if (current_playouts % 500 == 0) {
        if (params_.backup_strategy == BACKUP_OMEGA_Q || params_.backup_strategy == BACKUP_OMEGA_SARSA) {
          omega_nreturn_variance_.resize(omega_nreturn_sum_.size());
          omega_vplus_ = 0.0f;
          std::vector<float> xrange(omega_nreturn_sum_.size() - 1);
          for (int i = 0; i < omega_nreturn_sum_.size(); ++i) {
            omega_nreturn_variance_[i] = (omega_nreturn_sum_squares_[i] - ((omega_nreturn_sum_[i] * omega_nreturn_sum_[i]) / omega_num_sample_trajectories_)) / omega_num_sample_trajectories_;
            omega_vplus_ = std::max(omega_vplus_, omega_nreturn_variance_[i]);
            if (i != omega_nreturn_sum_.size() - 1) {
              xrange[i] = i;
            } else {
              // Last variance.
              omega_vl_ = omega_nreturn_variance_[i];
            }
          }
          leastSquaresExponentialFit2(xrange, omega_nreturn_variance_, omega_k1_, omega_k2_);
          omega_values_initialized_ = true;
          calculateOmegaWeightsFromParams();
        }
      }
      // if (current_playouts == 20) {
      //   throw std::runtime_error("blah!");
      // }
      current_time = boost::posix_time::microsec_clock::local_time();
    }

    start_action_available_ = false;
  }

  Action::ConstPtr MCTS::getBestAction(const State::ConstPtr& state) const {

    typedef std::pair<Action::ConstPtr, StateActionNode::ConstPtr> Action2StateActionInfoPair;
    typedef std::pair<State::ConstPtr, StateNode::ConstPtr> State2StateInfoPair;

    // Look for this state from the root node.
    State::Ptr discretized_state = state->clone();
    // TODO handle state discretization
    //stateMapping->map(discretized_state);

    // TODO clean this logic up, check if the state is the state corresponding to the root node or not.
    StateNode::ConstPtr state_node = root_node_;
    bool use_default_action = true;
    if (root_node_) {
      if (last_action_selected_) {
        // We were at root_node_ and selected last_action_selected_. However getBestAction probably wants the next state
        // afterwards.
        StateActionNode::ConstPtr action = root_node_->actions.find(last_action_selected_)->second;
        if (action->next_states.find(discretized_state) != action->next_states.end()) {
          state_node = action->next_states.find(discretized_state)->second;
          use_default_action = false;
        } else {
          MCTS_DEBUG_OUTPUT("  This state was not found in the search tree! Taking action using default policy!");
          MCTS_DEBUG_OUTPUT("    The following states were found!");
          BOOST_FOREACH(const State2StateInfoPair& state_info_pair, action->next_states) {
            MCTS_DEBUG_OUTPUT("    " << *(state_info_pair.first));
          }
        }
      } else {
        // We are at the start of the episode, and we've performed search at state.
        state_node = root_node_;
        use_default_action = false;
      }
    } else {
      MCTS_VERBOSE_OUTPUT("  The root search node for MCTS does not exist! Did you call search?");
    }

    if (use_default_action) {
      MCTS_VERBOSE_OUTPUT("  Returning action as per default policy!");
      return default_planner_->getBestAction(state);
    }

    float max_value = -std::numeric_limits<float>::max();
    std::vector<Action::ConstPtr> best_actions;
    MCTS_VERBOSE_OUTPUT("  Total visits to this state: " << state_node->state_visits);
    BOOST_FOREACH(const Action2StateActionInfoPair& action_info_pair, state_node->actions) {
      float val = getStateActionValue(action_info_pair.second);
      MCTS_VERBOSE_OUTPUT("    " << *(action_info_pair.first) << ": " << val << "(" << action_info_pair.second->visits << ")");
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
    last_action_selected_.reset();
    search(start_state, timeout, params_.max_playouts);
  }

  void MCTS::performPreActionProcessing(const State::ConstPtr& start_state,
                                        const Action::ConstPtr& prev_action,
                                        float timeout) {

    // TODO discretize this state.
    State::Ptr discretized_state = start_state->clone();
    if (root_node_) {
      if (prev_action) {
        StateActionNode::ConstPtr action = root_node_->actions.find(prev_action)->second;
        if (action->next_states.find(discretized_state) != action->next_states.end()) {
          root_node_ = action->next_states.find(discretized_state)->second;
        } else {
          restart();
        }
      } else {
        restart();
      }
    }

    last_action_selected_.reset();
    search(start_state, timeout, params_.max_playouts);
  }

  void MCTS::performPostActionProcessing(const State::ConstPtr& state,
                                         const Action::ConstPtr& action,
                                         float timeout) {
    // TODO also lookup past search history.
    restart();
    search(state, action, timeout, params_.max_playouts);
  }

  void MCTS::restart() {
    root_node_.reset();
  }

  Action::ConstPtr MCTS::getPlanningAction(const State::ConstPtr& state,
                                           const StateNode::ConstPtr& state_node) const {

    typedef std::pair<Action::ConstPtr, StateActionNode::ConstPtr> Action2StateActionInfoPair;
    std::vector<Action::ConstPtr> best_actions;

    if (params_.action_selection_strategy == UCT ||
        params_.action_selection_strategy == UCT_TUNED ||
        params_.action_selection_strategy == THOMPSON ||
        params_.action_selection_strategy == THOMPSON_BETA ||
        params_.action_selection_strategy == HIGHEST_MEAN ||
        params_.action_selection_strategy == RANDOM ||
        params_.action_selection_strategy == UNIFORM) {

      if (state_node->state_visits == 0) {
        // This is the first time this state node is being visited. Simply return the action from the default policy.
        best_actions.push_back(default_planner_->getBestAction(state));
      } else {
        float max_value = -std::numeric_limits<float>::max();
        BOOST_FOREACH(const Action2StateActionInfoPair& action_info_pair, state_node->actions) {
          // Give a really high value for cases where the state-action pair has not been visited.
          float planning_value = 1e10f;
          if (action_info_pair.second->visits != 0) {
            if (params_.action_selection_strategy == UCT) {
              // UCT comptues the planning value as the upper confidence bound.
              planning_value = action_info_pair.second->mean_value +
                params_.uct_reward_bound * sqrtf(logf(state_node->state_visits) / action_info_pair.second->visits);
            } else if (params_.action_selection_strategy == UCT_TUNED) {
              float reward_bound = (action_info_pair.second->variance +
                                    sqrtf((2 * logf(state_node->state_visits)) / action_info_pair.second->visits));
              planning_value = (action_info_pair.second->mean_value +
                                sqrtf(params_.uct_reward_bound * (logf(state_node->state_visits) / action_info_pair.second->visits)));
            } else if (params_.action_selection_strategy == UNIFORM) {
              planning_value = 1e5f / (1 + action_info_pair.second->visits);
            } else {
              // Give this action a high value, and change it as per .
              planning_value = 1e9f; // This value is good enough for random exploration.
              if (params_.action_selection_strategy == THOMPSON) {
                if (action_info_pair.second->visits >= params_.thompson_initial_random_trials) {
                  planning_value = rng_->normalFloat(action_info_pair.second->mean_value,
                                                     action_info_pair.second->variance);
                }
              } else if (params_.action_selection_strategy == THOMPSON_BETA) {
                planning_value = rng_->betaFloat(action_info_pair.second->alpha,
                                                 action_info_pair.second->beta);
              } else if (params_.action_selection_strategy == HIGHEST_MEAN) {
                if (action_info_pair.second->visits >= params_.mean_initial_random_trials) {
                  planning_value = action_info_pair.second->mean_value;
                }
              }
            }
          }
          /* std::cout << planning_value << " " << max_value << " " << best_actions.size() << std::endl; */
          if (fabs(planning_value - max_value) < 1e-10f) {
            best_actions.push_back(action_info_pair.first);
          } else if (planning_value > max_value) {
            max_value = planning_value;
            best_actions.clear();
            best_actions.push_back(action_info_pair.first);
          }
        }
      }

    } else {
      throw IncorrectUsageException(std::string("MCTS: Unknown action selection strategy provided in parameter set: ") +
                                    params_.action_selection_strategy);
    }

    /* std::cout << best_actions.size() << std::endl; */
    return best_actions[rng_->randomInt(best_actions.size() - 1)];
  }

  void MCTS::updateState(HistoryStep& step, float& value_sample) {

    /* Update mean value for this action. */
    StateNode::Ptr& state_info = step.state;
    ++(state_info->state_visits);
    StateActionNode::Ptr& action_info = state_info->actions[step.action];
    MCTS_DEBUG_OUTPUT("  Original value and visits for this state-action: " << action_info->mean_value <<
                      " (" << action_info->visits << ")");
    ++(action_info->visits);
    action_info->mean_value += (1.0 / action_info->visits) * (value_sample - action_info->mean_value);

    MCTS_DEBUG_OUTPUT("    Updated value and visits for this state-action: " << action_info->mean_value <<
                      " (" << action_info->visits << ")");

    action_info->sum_squares += value_sample * value_sample;
    action_info->variance = (action_info->sum_squares / action_info->visits);
    action_info->variance -= (action_info->mean_value * action_info->mean_value) / action_info->visits;
    // Always ensure that variance is positive to avoid floating point arithmetic issues.
    action_info->variance = std::max(action_info->variance, 0.0f);

    if (params_.action_selection_strategy == THOMPSON_BETA) {
      float normalized_reward =
        (value_sample - params_.thompson_beta_min_reward) /
        (params_.thompson_beta_max_reward - params_.thompson_beta_min_reward);
      if (normalized_reward < 0.0f) {
        std::cerr << "Received reward: " << value_sample <<
          " below specified min: " << params_.thompson_beta_min_reward << std::endl;
        normalized_reward = 0.0f;
      } else if (normalized_reward > 1.0f) {
        std::cerr << "Received reward: " << value_sample <<
          " above specified max: " << params_.thompson_beta_max_reward << std::endl;
        normalized_reward = 1.0f;
      }
      action_info->alpha += normalized_reward;
      action_info->beta += (1.0f - normalized_reward);
    }


  }

  StateNode::Ptr MCTS::getNewStateNode(const State::ConstPtr& state) {
    std::vector<Action::ConstPtr> actions;
    model_->getActionsAtState(state, actions);
    StateNode::Ptr state_node(new StateNode);
    state_node->state = state;
    BOOST_FOREACH(const Action::ConstPtr& action, actions) {
      StateActionNode::Ptr state_action_node(new StateActionNode);
      state_node->actions[action] = state_action_node;
    }
    return state_node;
  }

  std::map<std::string, std::string> MCTS::getParamsAsMap() const {
    return params_.asMap();
  }

  std::string MCTS::getName() const {
    return std::string("MCTS");
  }

  void MCTS::calculateOmegaWeightsFromParams() {

    Eigen::MatrixXf cov(omega_nreturn_variance_.size(), omega_nreturn_variance_.size());
    for (int i = 0; i < omega_nreturn_variance_.size() - 1; ++i) {
      int val = std::min(omega_vplus_, omega_k1_ * expf(omega_k2_));
      cov (i,i) = val;
      for (int j = i + 1; j < omega_nreturn_variance_.size(); ++j) {
        cov(i,j) = val;
        cov(j,i) = val;
      }
    }
    cov(omega_nreturn_variance_.size() - 1,  omega_nreturn_variance_.size() - 1) = omega_vl_;
    Eigen::MatrixXf cov_inv = cov.inverse();

    float matrix_sum = cov_inv.sum();
    Eigen::VectorXf rowwise_sum = cov_inv.rowwise().sum();
    omega_return_coefficients_.resize(omega_nreturn_variance_.size());
    for (int i = 0; i < omega_nreturn_variance_.size(); ++i) {
      omega_return_coefficients_[i] = rowwise_sum[i] / matrix_sum;
    }
  }

} /* utexas_planning */

CLASS_LOADER_REGISTER_CLASS(utexas_planning::MCTS, utexas_planning::AbstractPlanner);

      // NOTE: DEPRECATED Automated lambda per node code.

      // if (params_.use_automated_lambda) {
      //   // state_info->max_value = std::max(state_info->max_value, value_sample);
      //   state_info->max_value = maxValueForState(state_info);
      //   state_info->lambda = 0.0f;
      //   /* std::cout << "start loop: " << std::endl; */
      //   typedef std::pair<const Action::ConstPtr, boost::shared_ptr<StateActionNode> > ActionInfoPair;
      //   BOOST_FOREACH(ActionInfoPair &action_pair, state_info->actions) {
      //     StateActionNode::Ptr &action = action_pair.second;
      //     if (action->visits >= 2) {
      //       if (action->mean_value < state_info->max_value) {
      //         float z_score = (state_info->max_value - action->mean_value) / sqrtf(action->variance);
      //         float lambda_from_z = 1.0f / expf(z_score);
      //         /* std::cout << "lambda_from_z: " << lambda_from_z << "/" << state_info->lambda << std::endl; */
      //         state_info->lambda = std::max(state_info->lambda, lambda_from_z);
      //       }
      //     } else {
      //       // Insufficient samples on one of the samples, backpropagate the current value.
      //       state_info->lambda = 1.0f;
      //     }
      //   }
      //   // if (state_info->lambda != 1.0f) {
      //   //   std::cout << "selecting lambda: " << state_info->lambda << std::endl;
      //   // }
      // } else if (params_.use_automated_lambda_2) {

      //   typedef std::pair<Action::ConstPtr, StateActionNode::ConstPtr> Action2StateActionInfoPair;
      //   state_info->max_value = -std::numeric_limits<float>::max();
      //   float max_alpha, max_beta;
      //   BOOST_FOREACH(const Action2StateActionInfoPair& action_info_pair, state_info->actions) {
      //     float val = getStateActionValue(action_info_pair.second);
      //     if (val > state_info->max_value && action_info_pair.second->visits >= 2) {
      //       state_info->max_value = val;
      //       max_alpha = action_info_pair.second->alpha;
      //       max_beta = action_info_pair.second->beta;
      //     }
      //     state_info->max_value = std::max(val, state_info->max_value);

      //   }

      //   state_info->max_value = maxValueForState(state_info);
      //   if (state_info->actions.size() <= 1) {
      //     state_info->lambda = 1.0f;
      //   } else {
      //     state_info->lambda = 0.0f;

      //     BOOST_FOREACH(const Action2StateActionInfoPair &action_pair, state_info->actions) {
      //       const StateActionNode::ConstPtr &action = action_pair.second;
      //       if (action->visits >= 2) {
      //         if (action->mean_value < state_info->max_value) {
      //           namespace bm = boost::math;
      //           float kl_divergence = logf(bm::beta(action->alpha, action->beta) / bm::beta(max_alpha, max_beta)) +
      //             (max_alpha - action->alpha) * bm::digamma(max_alpha) +
      //             (max_beta - action->beta) * bm::digamma(max_beta) +
      //             (action->alpha + action->beta - max_alpha - max_beta) * bm::digamma(max_alpha + max_beta);
      //           kl_divergence = std::max(0.0f, kl_divergence);
      //           float lambda_from_kl = 1.0f / expf(kl_divergence);
      //           // if (lambda_from_kl < 0.1f) {
      //           //   std::cout << "lambda_from_kl: " << lambda_from_kl << "," << kl_divergence << std::endl;
      //           //   std::cout << "Found KL (" << max_alpha << "," << max_beta << ")->(" << action->alpha << "," << action->beta << "): " << kl_divergence << std::endl;
      //           // }
      //           /* std::cout << "lambda_from_kl: " << lambda_from_kl << "/" << state_info->lambda << std::endl; */
      //           state_info->lambda = std::max(state_info->lambda, lambda_from_kl);
      //         }
      //       } else {
      //         // Insufficient samples on one of the samples, backpropagate the current value.
      //         state_info->lambda = 1.0f;
      //         break;
      //       }
      //     }
      //   }

      //   // if (state_info->lambda <= 0.5f) {
      //   //   std::cout << "selecting lambda: " << state_info->lambda << std::endl;
      //   // }

      // } else {
