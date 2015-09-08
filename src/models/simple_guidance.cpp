#include <boost/lexical_cast.hpp>
#include <class_loader/class_loader.h>
#include <utexas_planning/common/exceptions.h>
#include <utexas_planning/models/simple_guidance.h>

namespace utexas_planning {

  bool SimpleGuidanceAction::operator<(const Action& other_base) const {
    try {
      const SimpleGuidanceAction& other = dynamic_cast<const SimpleGuidanceAction&>(other_base);
      return type < other.type;
    } catch(const std::bad_cast& exp) {
      throw DowncastException("Action", "SimpleGuidanceAction");
    }
  }

  void SimpleGuidanceAction::serialize(std::ostream& stream) const {
    if (type == UP) {
      stream << "Up";
    } else if (type == DOWN) {
      stream << "Down";
    } else if (type == LEFT) {
      stream << "Left";
    } else if (type == RIGHT) {
      stream << "Right";
    } else {
      stream << "Noop";
    }
  }

} /* utexas_planning */

BOOST_CLASS_EXPORT_IMPLEMENT(utexas_planning::SimpleGuidanceAction);

namespace utexas_planning {

  bool SimpleGuidanceState::operator<(const State& other_base) const {
    try {
      const SimpleGuidanceState& other = dynamic_cast<const SimpleGuidanceState&>(other_base);
      if (x < other.x) return true;
      if (x > other.x) return false;

      if (y < other.y) return true;
      if (y > other.y) return false;

      if (prev_action < other.prev_action) return true;
      if (other.prev_action < prev_action) return false;

      return false;
    } catch(const std::bad_cast& exp) {
      throw DowncastException("State", "SimpleGuidanceState");
    }
  }

  void SimpleGuidanceState::serialize(std::ostream& stream) const {
    stream << "(" << x << "," << y << "," << prev_action << ")";
  }

  State::Ptr SimpleGuidanceState::cloneImpl() const {
    boost::shared_ptr<SimpleGuidanceState> clone(new SimpleGuidanceState);
    clone->x = x;
    clone->y = y;
    clone->prev_action = prev_action;
    return clone;
  }

  std::map<std::string, std::string> SimpleGuidanceState::asMap() const {
    std::map<std::string, std::string> state_map;
    state_map["x"] = boost::lexical_cast<std::string>(x);
    state_map["y"] = boost::lexical_cast<std::string>(y);
    state_map["prev_action"] = boost::lexical_cast<std::string>(prev_action.type);
    return state_map;
  }

} /* utexas_planning */

BOOST_CLASS_EXPORT_IMPLEMENT(utexas_planning::SimpleGuidanceState);

namespace utexas_planning {

  void SimpleGuidanceModel::init(const YAML::Node& params,
                       const std::string& /* output_directory */,
                       const boost::shared_ptr<RNG>& rng) {

    params_.fromYaml(params);

    // It may not be possible to actually get to some of these states.
    for (int x = 0; x < params_.grid_size; ++x) {
      for (int y = 0; y < params_.grid_size; ++y) {
        for (int prev_action_type = 0; prev_action_type < 4; ++prev_action_type) {
          boost::shared_ptr<SimpleGuidanceState> s(new SimpleGuidanceState);
          s->x = x;
          s->y = y;
          s->prev_action.type = (SimpleGuidanceActionType)prev_action_type;
          complete_state_vector_.push_back(s);
        }
      }
    }

    up_action.reset(new SimpleGuidanceAction);
    up_action->type = UP;
    down_action.reset(new SimpleGuidanceAction);
    down_action->type = DOWN;
    left_action.reset(new SimpleGuidanceAction);
    left_action->type = LEFT;
    right_action.reset(new SimpleGuidanceAction);
    right_action->type = RIGHT;
    noop_action.reset(new SimpleGuidanceAction);
    noop_action->type = NOOP;

    // Construct the domain using the seed.
    // Should we keep the domain constant for a particular number of roundabouts?
    for (int roundabout_idx = 0; roundabout_idx < params_.num_roundabouts; ++roundabout_idx) {
      bool new_roundabout_found = false;
      int x = -1, y = -1;
      while (!new_roundabout_found) {
        x = rng->randomInt(params_.grid_size - 1);
        y = rng->randomInt(params_.grid_size - 1);
        new_roundabout_found = true;
        for (int other_roundabout_idx = 0; other_roundabout_idx < roundabout_idx; ++other_roundabout_idx) {
          std::pair<int, int>& other_roundabout = roundabouts_[other_roundabout_idx];
          if ((x == other_roundabout.first) && (y == other_roundabout.second)) {
            new_roundabout_found = false;
            break;
          }
        }
      }

      if (new_roundabout_found) {
        std::pair<int, int> new_roundabout(x, y);
        roundabouts_.push_back(new_roundabout);
      }
    }

    // Print domain to screen
    for (int x = 0; x < params_.grid_size; ++x) {
      for (int y = 0; y < params_.grid_size; ++y) {
        bool at_roundabout = false;
        for (int roundabout_idx = 0; roundabout_idx < roundabouts_.size(); ++roundabout_idx) {
          const std::pair<int, int>& roundabout = roundabouts_[roundabout_idx];
          if ((x == roundabout.first) && (y == roundabout.second)) {
            at_roundabout = true;
            break;
          }
        }

        if (at_roundabout) {
          std::cout << "O ";
        } else {
          std::cout << "x ";
        }
      }
      std::cout << std::endl;
    }

  }
  bool SimpleGuidanceModel::isTerminalState(const State::ConstPtr& state_base) const {
    boost::shared_ptr<const SimpleGuidanceState> state =
      boost::dynamic_pointer_cast<const SimpleGuidanceState>(state_base);
    if (!state) {
      throw DowncastException("State", "SimpleGuidanceState");
    }
    return ((state->x == params_.grid_size/2) && (state->y == params_.grid_size/2));
  }

  void SimpleGuidanceModel::getActionsAtState(const State::ConstPtr& state_base,
                                              std::vector<Action::ConstPtr>& actions) const {
    actions.clear();
    boost::shared_ptr<const SimpleGuidanceState> state = boost::dynamic_pointer_cast<const SimpleGuidanceState>(state_base);
    if (!state) {
      throw DowncastException("State", "SimpleGuidanceState");
    }
    if (!isTerminalState(state)) {
      boost::shared_ptr<const SimpleGuidanceState> state = boost::dynamic_pointer_cast<const SimpleGuidanceState>(state_base);
      if (!state) {
        throw DowncastException("State", "SimpleGuidanceState");
      }

      // Check if we're at a roundabout;
      bool at_roundabout = false;
      for (int roundabout_idx = 0; roundabout_idx < roundabouts_.size(); ++roundabout_idx) {
        const std::pair<int, int>& roundabout = roundabouts_[roundabout_idx];
        if ((state->x == roundabout.first) && (state->y == roundabout.second)) {
          at_roundabout = true;
          break;
        }
      }

      if (!at_roundabout) {
        actions.push_back(noop_action);
      } else {
        actions.push_back(left_action);
        actions.push_back(right_action);
        actions.push_back(up_action);
        actions.push_back(down_action);
      }

    }
    if (actions.size() == 0) {
      std::stringstream ss;
      ss << *state;
      throw std::runtime_error("Bug! Found 0 actions at state " + ss.str());
    }
  }

  std::vector<State::ConstPtr> SimpleGuidanceModel::getStateVector() const {
    return complete_state_vector_;
  }

  void SimpleGuidanceModel::getTransitionDynamics(const State::ConstPtr& state_base,
                                              const Action::ConstPtr& action_base,
                                              std::vector<State::ConstPtr>& next_states,
                                              std::vector<float>& rewards,
                                              std::vector<float>& probabilities) const {

    boost::shared_ptr<const SimpleGuidanceState> state = boost::dynamic_pointer_cast<const SimpleGuidanceState>(state_base);
    if (!state) {
      throw DowncastException("State", "SimpleGuidanceState");
    }

    boost::shared_ptr<const SimpleGuidanceAction> action = boost::dynamic_pointer_cast<const SimpleGuidanceAction>(action_base);
    if (!action) {
      throw DowncastException("Action", "SimpleGuidanceAction");
    }

    next_states.clear();
    rewards.clear();
    probabilities.clear();

    if (!isTerminalState(state)) {

      float p_left = (state->prev_action.type == LEFT) ? 1.0f : 0.0f;
      float p_right = (state->prev_action.type == RIGHT) ? 1.0f : 0.0f;
      float p_up = (state->prev_action.type == UP) ? 1.0f : 0.0f;
      float p_down = (state->prev_action.type == DOWN) ? 1.0f : 0.0f;

      if (action->type != NOOP) {
        float success_prob = ((1.0f - params_.nondeterminism) + (params_.nondeterminism / 4));
        float failure_prob = (params_.nondeterminism / 4);
        p_left = (action->type == LEFT) ? success_prob : failure_prob;
        p_right = (action->type == RIGHT) ? success_prob : failure_prob;
        p_up = (action->type == UP) ? success_prob : failure_prob;
        p_down = (action->type == DOWN) ? success_prob : failure_prob;
      }

      bool left_blocked = (state->x == 0);
      bool right_blocked = (state->x == params_.grid_size - 1);
      bool up_blocked = (state->y == params_.grid_size - 1);
      bool down_blocked = (state->y == 0);

      if (left_blocked) {
        if (up_blocked) {
          p_down += p_left;
        } else if (down_blocked) {
          p_up += p_left;
        } else {
          p_up += p_left / 2;
          p_down += p_left / 2;
        }
        p_left = 0.0f;
      }

      if (right_blocked) {
        if (up_blocked) {
          p_down += p_right;
        } else if (down_blocked) {
          p_up += p_right;
        } else {
          p_up += p_right / 2;
          p_down += p_right / 2;
        }
        p_right = 0.0f;
      }

      if (up_blocked) {
        if (left_blocked) {
          p_right += p_up;
        } else if (right_blocked) {
          p_left += p_up;
        } else {
          p_right += p_up / 2;
          p_left += p_up / 2;
        }
        p_up = 0.0f;
      }

      if (down_blocked) {
        if (left_blocked) {
          p_right += p_down;
        } else if (right_blocked) {
          p_left += p_down;
        } else {
          p_right += p_down / 2;
          p_left += p_down / 2;
        }
        p_down = 0.0f;
      }

      if (p_up != 0.0f) {
        SimpleGuidanceState next_state = *state;
        next_state.y += 1;
        next_state.prev_action.type = UP;

        probabilities.push_back(p_up);
        rewards.push_back(-1);
        next_states.push_back(complete_state_vector_[getStateIndex(next_state)]);
      }

      if (p_down != 0.0f) {
        SimpleGuidanceState next_state = *state;
        next_state.y -= 1;
        next_state.prev_action.type = DOWN;

        probabilities.push_back(p_down);
        rewards.push_back(-1);
        next_states.push_back(complete_state_vector_[getStateIndex(next_state)]);
      }

      if (p_left != 0.0f) {
        SimpleGuidanceState next_state = *state;
        next_state.x -= 1;
        next_state.prev_action.type = LEFT;

        probabilities.push_back(p_left);
        rewards.push_back(-1);
        next_states.push_back(complete_state_vector_[getStateIndex(next_state)]);
      }

      if (p_right != 0.0f) {
        SimpleGuidanceState next_state = *state;
        next_state.x += 1;
        next_state.prev_action.type = RIGHT;

        probabilities.push_back(p_right);
        rewards.push_back(-1);
        next_states.push_back(complete_state_vector_[getStateIndex(next_state)]);
      }

      if (probabilities.size() == 0) {
        std::cout << *state << " " << *action << std::endl;
        assert(probabilities.size() == 0);
      }
    }
  }

  void SimpleGuidanceModel::takeAction(const State::ConstPtr& state,
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

  State::ConstPtr SimpleGuidanceModel::getStartState(long seed) const {
    RNG rng(seed);
    int idx = rng.randomInt(complete_state_vector_.size() - 1);
    while (isTerminalState(complete_state_vector_[idx])) {
      idx = rng.randomInt(complete_state_vector_.size() - 1);
    }
    return complete_state_vector_[idx];
  }

  float SimpleGuidanceModel::getInitialTimeout() const {
    return params_.initial_planning_time;
  }

  std::map<std::string, std::string> SimpleGuidanceModel::getParamsAsMap() const {
    return params_.asMap();
  }

  std::string SimpleGuidanceModel::getName() const {
    return std::string("SimpleGuidance");
  }

} /* utexas_planning */

CLASS_LOADER_REGISTER_CLASS(utexas_planning::SimpleGuidanceModel, utexas_planning::GenerativeModel);
