#include <boost/lexical_cast.hpp>
#include <class_loader/class_loader.h>
#include <utexas_planning/common/exceptions.h>
#include <utexas_planning/models/light_world.h>

namespace utexas_planning {

  bool LightWorldAction::operator<(const Action& other_base) const {
    try {
      const LightWorldAction& other = dynamic_cast<const LightWorldAction&>(other_base);
      return type < other.type;
    } catch(const std::bad_cast& exp) {
      throw DowncastException("Action", "LightWorldAction");
    }
  }

  void LightWorldAction::serialize(std::ostream& stream) const {
    if (type == UP) {
      stream << "Up";
    } else if (type == DOWN) {
      stream << "Down";
    } else if (type == LEFT) {
      stream << "Left";
    } else if (type == RIGHT) {
      stream << "Right";
    } else if (type == PICKUP) {
      stream << "Pickup";
    } else {
      stream << "Unlock";
    }
  }

} /* utexas_planning */

BOOST_CLASS_EXPORT_IMPLEMENT(utexas_planning::LightWorldAction);

namespace utexas_planning {

  bool LightWorldState::operator<(const State& other_base) const {
    try {
      const LightWorldState& other = dynamic_cast<const LightWorldState&>(other_base);
      if (x < other.x) return true;
      if (x > other.x) return false;

      if (y < other.y) return true;
      if (y > other.y) return false;

      if (key_picked_up < other.key_picked_up) return true;
      if (key_picked_up > other.key_picked_up) return false;

      if (goal_unlocked < other.goal_unlocked) return true;
      if (goal_unlocked > other.goal_unlocked) return false;

      if (unlock_attempts_left < other.unlock_attempts_left) return true;
      if (unlock_attempts_left > other.unlock_attempts_left) return false;

      return false;
    } catch(const std::bad_cast& exp) {
      throw DowncastException("State", "LightWorldState");
    }
  }

  void LightWorldState::serialize(std::ostream& stream) const {
    stream << "(" << x << "," << y << ",";
    if (key_picked_up) {
      stream << "key";
    } else {
      stream << "no key";
    }
    stream << ",";
    if (goal_unlocked) {
      stream << "unlocked";
    } else {
      stream << "locked";
    }
    stream << ",UnlockAttemptsLeft=";
    stream << unlock_attempts_left << ")";
  }

  State::Ptr LightWorldState::cloneImpl() const {
    boost::shared_ptr<LightWorldState> clone(new LightWorldState);
    clone->x = x;
    clone->y = y;
    clone->key_picked_up = key_picked_up;
    clone->goal_unlocked = goal_unlocked;
    clone->unlock_attempts_left = unlock_attempts_left;
    return clone;
  }

  std::map<std::string, std::string> LightWorldState::asMap() const {
    std::map<std::string, std::string> state_map;
    state_map["x"] = boost::lexical_cast<std::string>(x);
    state_map["y"] = boost::lexical_cast<std::string>(y);
    state_map["key_picked_up"] = boost::lexical_cast<std::string>(key_picked_up);
    state_map["goal_unlocked"] = boost::lexical_cast<std::string>(goal_unlocked);
    state_map["unlock_attempts_left"] = boost::lexical_cast<std::string>(unlock_attempts_left);
    return state_map;
  }

} /* utexas_planning */

BOOST_CLASS_EXPORT_IMPLEMENT(utexas_planning::LightWorldState);

namespace utexas_planning {

  void LightWorldModel::init(const YAML::Node& params,
                       const std::string& /* output_directory */,
                       const boost::shared_ptr<RNG>& rng) {

    params_.fromYaml(params);

    // Precache the complete state vector of 100 states.
    for (int x = 0; x < params_.grid_size; ++x) {
      for (int y = 0; y < params_.grid_size; ++y) {
        for (int key_picked_up = 0; key_picked_up < 2; ++key_picked_up) {
          for (int goal_unlocked = 0; goal_unlocked < 2; ++goal_unlocked) {
            for (int unlock_attempts_left = 0; unlock_attempts_left <= params_.initial_unlock_attempts;
                 ++unlock_attempts_left) {
              boost::shared_ptr<LightWorldState> s(new LightWorldState);
              s->x = x;
              s->y = y;
              s->key_picked_up = key_picked_up;
              s->goal_unlocked = goal_unlocked;
              s->unlock_attempts_left = unlock_attempts_left;
              complete_state_vector_.push_back(s);
            }
          }
        }
      }
    }

    up_action.reset(new LightWorldAction);
    up_action->type = UP;
    down_action.reset(new LightWorldAction);
    down_action->type = DOWN;
    left_action.reset(new LightWorldAction);
    left_action->type = LEFT;
    right_action.reset(new LightWorldAction);
    right_action->type = RIGHT;
    pickup_action.reset(new LightWorldAction);
    pickup_action->type = PICKUP;
    unlock_action.reset(new LightWorldAction);
    unlock_action->type = UNLOCK;

  }

  bool LightWorldModel::isTerminalState(const State::ConstPtr& state_base) const {
    boost::shared_ptr<const LightWorldState> state = boost::dynamic_pointer_cast<const LightWorldState>(state_base);
    if (!state) {
      throw DowncastException("State", "LightWorldState");
    }
    return
      /* ((!(state->goal_unlocked)) && (state->unlock_attempts_left == 0)) || */
      ((state->x == params_.goal_x) && (state->y == params_.goal_y) && (state->goal_unlocked));
  }

  void LightWorldModel::getActionsAtState(const State::ConstPtr& state_base,
                                          std::vector<Action::ConstPtr>& actions) const {
    actions.clear();
    boost::shared_ptr<const LightWorldState> state = boost::dynamic_pointer_cast<const LightWorldState>(state_base);
    if (!state) {
      throw DowncastException("State", "LightWorldState");
    }
    if (!isTerminalState(state)) {
      boost::shared_ptr<const LightWorldState> state = boost::dynamic_pointer_cast<const LightWorldState>(state_base);
      if (!state) {
        throw DowncastException("State", "LightWorldState");
      }
      if (state->x > 0) {
        actions.push_back(left_action);
      }
      if (state->x < params_.grid_size - 1) {
        actions.push_back(right_action);
      }
      if (state->y > 0) {
        actions.push_back(down_action);
      }
      if (state->y < params_.grid_size - 1) {
        actions.push_back(up_action);
      }
      if (!state->key_picked_up) {
        actions.push_back(pickup_action);
      } else {
        // Can only unlock when you have the key.
        if (!state->goal_unlocked && state->unlock_attempts_left > 0) {
          actions.push_back(unlock_action);
        }
      }
    }
    if (actions.size() == 0) {
      std::stringstream ss;
      ss << *state;
      throw std::runtime_error("Bug! Found 0 actions at state " + ss.str());
    }
  }

  std::vector<State::ConstPtr> LightWorldModel::getStateVector() const {
    return complete_state_vector_;
  }

  void LightWorldModel::getTransitionDynamics(const State::ConstPtr& state_base,
                                              const Action::ConstPtr& action_base,
                                              std::vector<State::ConstPtr>& next_states,
                                              std::vector<float>& rewards,
                                              std::vector<float>& probabilities) const {

    boost::shared_ptr<const LightWorldState> state = boost::dynamic_pointer_cast<const LightWorldState>(state_base);
    if (!state) {
      throw DowncastException("State", "LightWorldState");
    }

    boost::shared_ptr<const LightWorldAction> action = boost::dynamic_pointer_cast<const LightWorldAction>(action_base);
    if (!action) {
      throw DowncastException("Action", "LightWorldAction");
    }

    next_states.clear();
    rewards.clear();
    probabilities.clear();

    if (!isTerminalState(state)) {

      if (action->type == PICKUP || action->type == UNLOCK) {
        LightWorldState next_state = *state;
        float reward = -1.0f;
        if (action->type == PICKUP) {
          if (state->x == params_.key_x && state->y == params_.key_y) {
            next_state.key_picked_up = true;
          } else {
            reward = params_.incorrect_pickup_reward;
          }
        } else if (action->type == UNLOCK) {
          if (state->unlock_attempts_left > 0) {
            next_state.unlock_attempts_left -= 1;
            if (state->x == params_.lock_x &&
                state->y == params_.lock_y) {
              next_state.goal_unlocked = true;
            } else {
              reward = params_.incorrect_unlock_reward;
            }
          }
        }
        rewards.push_back(reward);
        probabilities.push_back(1.0f);

        int idx = getStateIndex(next_state);
        next_states.push_back(complete_state_vector_[idx]);
      } else {
        // We're performing a navigation action.
        int num_valid_nav_actions = 0;
        if (state->x > 0) {
          ++num_valid_nav_actions;
        }
        if (state->x < params_.grid_size - 1) {
          ++num_valid_nav_actions;
        }
        if (state->y > 0) {
          ++num_valid_nav_actions;
        }
        if (state->y < params_.grid_size - 1) {
          ++num_valid_nav_actions;
        }

        if (state->x > 0) {
          // Left action is valid;
          LightWorldState next_state = *state;
          --next_state.x;
          float p = (action->type == LEFT) ?
            ((1.0f - params_.nondeterminism) + (params_.nondeterminism / num_valid_nav_actions)) :
            (params_.nondeterminism / num_valid_nav_actions);
          int idx = getStateIndex(next_state);
          float reward = -1.0f;
          if (isTerminalState(complete_state_vector_[idx])) {
            reward += 100.0f;
          }
          next_states.push_back(complete_state_vector_[idx]);
          probabilities.push_back(p);
          rewards.push_back(reward);
        }
        if (state->x < params_.grid_size - 1) {
          // Right action is valid;
          LightWorldState next_state = *state;
          ++next_state.x;
          float p = (action->type == RIGHT) ?
            ((1.0f - params_.nondeterminism) + (params_.nondeterminism / num_valid_nav_actions)) :
            (params_.nondeterminism / num_valid_nav_actions);
          int idx = getStateIndex(next_state);
          float reward = -1.0f;
          if (isTerminalState(complete_state_vector_[idx])) {
            reward += 100.0f;
          }
          next_states.push_back(complete_state_vector_[idx]);
          probabilities.push_back(p);
          rewards.push_back(reward);
        }
        if (state->y > 0) {
          // Down action is valid;
          LightWorldState next_state = *state;
          --next_state.y;
          float p = (action->type == DOWN) ?
            ((1.0f - params_.nondeterminism) + (params_.nondeterminism / num_valid_nav_actions)) :
            (params_.nondeterminism / num_valid_nav_actions);
          int idx = getStateIndex(next_state);
          float reward = -1.0f;
          if (isTerminalState(complete_state_vector_[idx])) {
            reward += 100.0f;
          }
          next_states.push_back(complete_state_vector_[idx]);
          probabilities.push_back(p);
          rewards.push_back(reward);
        }
        if (state->y < params_.grid_size - 1) {
          // Up action is valid;
          LightWorldState next_state = *state;
          ++next_state.y;
          float p = (action->type == UP) ?
            ((1.0f - params_.nondeterminism) + (params_.nondeterminism / num_valid_nav_actions)) :
            (params_.nondeterminism / num_valid_nav_actions);
          int idx = getStateIndex(next_state);
          float reward = -1.0f;
          if (isTerminalState(complete_state_vector_[idx])) {
            reward += 100.0f;
          }
          next_states.push_back(complete_state_vector_[idx]);
          probabilities.push_back(p);
          rewards.push_back(reward);
        }

      }
    }
  }

  void LightWorldModel::takeAction(const State::ConstPtr& state,
                             const Action::ConstPtr& action,
                             float& reward,
                             const RewardMetrics::Ptr& reward_metrics,
                             State::ConstPtr& next_state,
                             int& depth_count,
                             float& post_action_timeout,
                             boost::shared_ptr<RNG> rng) const {
    DeclarativeModel::takeAction(state,
                                 action,
                                 reward,
                                 reward_metrics,
                                 next_state,
                                 depth_count,
                                 post_action_timeout,
                                 rng);
    post_action_timeout = params_.per_step_planning_time;
  }

  State::ConstPtr LightWorldModel::getStartState(long seed) const {
    LightWorldState start_state;
    start_state.x = params_.start_x;
    start_state.y = params_.start_y;
    start_state.unlock_attempts_left = params_.initial_unlock_attempts;
    start_state.goal_unlocked = false;
    start_state.key_picked_up = false;
    return complete_state_vector_[getStateIndex(start_state)];
  }

  float LightWorldModel::getInitialTimeout() const {
    return params_.initial_planning_time;
  }

  std::map<std::string, std::string> LightWorldModel::getParamsAsMap() const {
    return params_.asMap();
  }

  std::string LightWorldModel::getName() const {
    return std::string("LightWorld");
  }

} /* utexas_planning */

CLASS_LOADER_REGISTER_CLASS(utexas_planning::LightWorldModel, utexas_planning::GenerativeModel);
