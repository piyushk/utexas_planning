#include <boost/lexical_cast.hpp>
#include <class_loader/class_loader.h>
#include <utexas_planning/common/exceptions.h>
#include <utexas_planning/models/grid_model.h>

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
        for (int key_pickup_up = 0; key_pickup_up < 2; ++key_pickup_up) {
          for (int goal_unlocked = 0; goal_unlocked < 2; ++goal_unlocked) {
            for (int unlock_attempts_left = 0;
                 unlock_attempts_left <= params_.initial_unlock_attempts;
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
    return ((state->x == params_.goal_x) && (state->y == params_.goal_y) && (state->goal_unlocked));
  }

  void LightWorldModel::getActionsAtState(const State::ConstPtr& state_base,
                                          std::vector<Action::ConstPtr>& actions) const {
    actions.clear();
    if (!isTerminalState(state)) {
      boost::shared_ptr<const LightWorldState> state = boost::dynamic_pointer_cast<const LightWorldState>(state_base);
      if (!state) {
        throw DowncastException("State", "LightWorldState");
      }
      if (state.x > 0) {
        actions.push_back(left_action);
      }
      if (state.x < params_.grid_size - 1) {
        actions.push_back(right_action);
      }
      if (state.y > 0) {
        actions.push_back(down_action);
      }
      if (state.y < params_.grid_size - 1) {
        actions.push_back(up_action);
      }
      if (!state.key_picked_up) {
        actions.push_back(pickup_action);
      }
      if (!state.goal_unlocked) {
        actions.push_back(unlock_action);
      }
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

      if (state == 0
      LightWorldState next_state = *state;

      //
      boost::shared_ptr<LightWorldState> ns(new LightWorldState);
      ns->x = state->x;
      ns->y = (state->y == 0) ? params_.grid_size - 1 : state->y - 1;
      next_states.push_back(ns);
      ns.reset(new LightWorldState);
      ns->x = state->x;
      ns->y = (state->y + 1) % params_.grid_size;
      next_states.push_back(ns);
      ns.reset(new LightWorldState);
      ns->x = (state->x == 0) ? params_.grid_size - 1 : state->x - 1;
      ns->y = state->y;
      next_states.push_back(ns);
      ns.reset(new LightWorldState);
      ns->x = (state->x + 1) % params_.grid_size;
      ns->y = state->y;
      next_states.push_back(ns);

      rewards.push_back(-1);
      rewards.push_back(-1);
      rewards.push_back(-1);
      rewards.push_back(-1);

      float p_up = (action->type == UP) ? 0.85f : 0.05f;
      float p_down = (action->type == DOWN) ? 0.85f : 0.05f;
      float p_left = (action->type == LEFT) ? 0.85f : 0.05f;
      float p_right = (action->type == RIGHT) ? 0.85f : 0.05f;

      probabilities.push_back(p_up);
      probabilities.push_back(p_down);
      probabilities.push_back(p_left);
      probabilities.push_back(p_right);
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
    if ((params_.start_x < 0 || params_.start_x >= params_.grid_size) ||
        (params_.start_y < 0 || params_.start_y >= params_.grid_size)) {
      RNG rng(seed);
      int idx = rng.randomInt(complete_state_vector_.size() - 1);
      while (isTerminalState(complete_state_vector_[idx])) {
        idx = rng.randomInt(complete_state_vector_.size() - 1);
      }
      return complete_state_vector_[idx];
    }
    // Vector is ordered in column major.
    return complete_state_vector_[params_.start_x * params_.grid_size + params_.start_y];
  }

  float LightWorldModel::getInitialTimeout() const {
    return params_.initial_planning_time;
  }

  std::map<std::string, std::string> LightWorldModel::getParamsAsMap() const {
    return params_.asMap();
  }

  std::string LightWorldModel::getName() const {
    return std::string("Grid");
  }

} /* utexas_planning */

CLASS_LOADER_REGISTER_CLASS(utexas_planning::LightWorldModel, utexas_planning::GenerativeModel);
