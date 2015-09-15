#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <class_loader/class_loader.h>
#include <utexas_planning/common/exceptions.h>
#include <utexas_planning/models/grid_model2.h>

namespace utexas_planning {

  bool GridAction2::operator<(const Action& other_base) const {
    try {
      const GridAction2& other = dynamic_cast<const GridAction2&>(other_base);
      return type < other.type;
    } catch(const std::bad_cast& exp) {
      throw DowncastException("Action", "GridAction2");
    }
  }

  void GridAction2::serialize(std::ostream& stream) const {
    if (type == UP) {
      stream << "Up";
    } else if (type == DOWN) {
      stream << "Down";
    } else if (type == LEFT) {
      stream << "Left";
    } else {
      stream << "Right";
    }
  }

} /* utexas_planning */

BOOST_CLASS_EXPORT_IMPLEMENT(utexas_planning::GridAction2);

namespace utexas_planning {

  bool GridState2::operator<(const State& other_base) const {
    try {
      const GridState2& other = dynamic_cast<const GridState2&>(other_base);
      if (x < other.x) return true;
      if (x > other.x) return false;

      if (y < other.y) return true;
      if (y > other.y) return false;

      return false;
    } catch(const std::bad_cast& exp) {
      throw DowncastException("State", "GridState2");
    }
  }

  void GridState2::serialize(std::ostream& stream) const {
    stream << "(" << x << "," << y << ")";
  }

  State::Ptr GridState2::cloneImpl() const {
    boost::shared_ptr<GridState2> clone(new GridState2);
    clone->x = x;
    clone->y = y;
    return clone;
  }

  std::map<std::string, std::string> GridState2::asMap() const {
    std::map<std::string, std::string> state_map;
    state_map["x"] = boost::lexical_cast<std::string>(x);
    state_map["y"] = boost::lexical_cast<std::string>(y);
    return state_map;
  }

} /* utexas_planning */

BOOST_CLASS_EXPORT_IMPLEMENT(utexas_planning::GridState2);

namespace utexas_planning {

  void GridModel2::init(const YAML::Node& params,
                       const std::string& /* output_directory */,
                       const boost::shared_ptr<RNG>& rng) {

    params_.fromYaml(params);

    // Precache the complete state vector of 100 states.
    for (int x = 0; x < params_.grid_size; ++x) {
      for (int y = 0; y < params_.grid_size; ++y) {
        boost::shared_ptr<GridState2> s(new GridState2);
        s->x = x;
        s->y = y;
        complete_state_vector_.push_back(s);
      }
    }

    for (int action_type = 0; action_type < NUM_ACTIONS; ++action_type) {
      boost::shared_ptr<GridAction2> a(new GridAction2);
      a->type = (GridActionType2)action_type;
      default_action_list_.push_back(a);
    }

    // If no terminal state specified, assume it is center of the grid.
    int idx = params_.goal_y + params_.goal_x * params_.grid_size;
    terminal_states_to_reward_map_[complete_state_vector_[idx]] = params_.goal_reward;

    for (int i = 0; i < params_.num_terminal_states; ++i) {
      while (true) {
        int x_idx = rng->randomInt(0,8);
        int y_idx = rng->randomInt(1,7);
        float reward = 0.0f;
        int idx = y_idx + x_idx * params_.grid_size;
        if (terminal_states_to_reward_map_.find(complete_state_vector_[idx]) ==
            terminal_states_to_reward_map_.end()) {
          terminal_states_to_reward_map_[complete_state_vector_[idx]] = reward;
          break;
        }
      }
    }
  }

  bool GridModel2::isTerminalState(const State::ConstPtr& state_base) const {
    boost::shared_ptr<const GridState2> state = boost::dynamic_pointer_cast<const GridState2>(state_base);
    if (!state) {
      throw DowncastException("State", "GridState2");
    }
    return terminal_states_to_reward_map_.find(state) != terminal_states_to_reward_map_.end();
  }

  void GridModel2::getActionsAtState(const State::ConstPtr& state,
                                    std::vector<Action::ConstPtr>& actions) const {
    actions.clear();
    if (!isTerminalState(state)) {
      actions = default_action_list_;
    }
  }

  std::vector<State::ConstPtr> GridModel2::getStateVector() const {
    return complete_state_vector_;
  }

  void GridModel2::getTransitionDynamics(const State::ConstPtr& state_base,
                                                const Action::ConstPtr& action_base,
                                                std::vector<State::ConstPtr>& next_states,
                                                std::vector<float>& rewards,
                                                std::vector<float>& probabilities) const {

    boost::shared_ptr<const GridState2> state = boost::dynamic_pointer_cast<const GridState2>(state_base);
    if (!state) {
      throw DowncastException("State", "GridState2");
    }

    boost::shared_ptr<const GridAction2> action = boost::dynamic_pointer_cast<const GridAction2>(action_base);
    if (!action) {
      throw DowncastException("Action", "GridAction2");
    }

    next_states.clear();
    rewards.clear();
    probabilities.clear();

    if (!isTerminalState(state)) {
      boost::shared_ptr<GridState2> ns(new GridState2);
      ns->x = state->x;
      if (params_.toroidal) {
        ns->y = (state->y == 0) ? params_.grid_size - 1 : state->y - 1;
      } else {
        ns->y = (state->y == 0) ? 0 : state->y - 1;
      }
      next_states.push_back(ns);
      ns.reset(new GridState2);
      ns->x = state->x;
      if (params_.toroidal) {
        ns->y = (state->y + 1) % params_.grid_size;
      } else {
        ns->y = (state->y == params_.grid_size - 1) ? params_.grid_size - 1 : state->y + 1;
      }
      next_states.push_back(ns);
      ns.reset(new GridState2);
      if (params_.toroidal) {
        ns->x = (state->x == 0) ? params_.grid_size - 1 : state->x - 1;
      } else {
        ns->x = (state->x == 0) ? 0 : state->x - 1;
      }
      ns->y = state->y;
      next_states.push_back(ns);
      ns.reset(new GridState2);
      if (params_.toroidal) {
        ns->x = (state->x + 1) % params_.grid_size;
      } else {
        ns->x = (state->x == params_.grid_size - 1) ? params_.grid_size - 1 : state->x + 1;
      }
      ns->y = state->y;
      next_states.push_back(ns);

      float r_up = (isTerminalState(next_states[0])) ? -1.0f + (terminal_states_to_reward_map_.find(next_states[0]))->second : -1.0f;
      float r_down = (isTerminalState(next_states[1])) ? -1.0f + (terminal_states_to_reward_map_.find(next_states[1]))->second : -1.0f;
      float r_left = (isTerminalState(next_states[2])) ? -1.0f + (terminal_states_to_reward_map_.find(next_states[2]))->second : -1.0f;
      float r_right = (isTerminalState(next_states[3])) ? -1.0f + (terminal_states_to_reward_map_.find(next_states[3]))->second : -1.0f;

      rewards.push_back(r_up);
      rewards.push_back(r_down);
      rewards.push_back(r_left);
      rewards.push_back(r_right);

      float p_up = (action->type == UP) ? (1.0f - params_.nondeterminism) + params_.nondeterminism/4 : params_.nondeterminism/4;
      float p_down = (action->type == DOWN) ? (1.0f - params_.nondeterminism) + params_.nondeterminism/4 : params_.nondeterminism/4;
      float p_left = (action->type == LEFT) ? (1.0f - params_.nondeterminism) + params_.nondeterminism/4 : params_.nondeterminism/4;
      float p_right = (action->type == RIGHT) ? (1.0f - params_.nondeterminism) + params_.nondeterminism/4 : params_.nondeterminism/4;

      probabilities.push_back(p_up);
      probabilities.push_back(p_down);
      probabilities.push_back(p_left);
      probabilities.push_back(p_right);

    }

  }

  void GridModel2::takeAction(const State::ConstPtr& state,
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

    // if (reward_metrics) {
    //   boost::shared_ptr<GridModelRewardMetrics> reward_metrics_derived =
    //     boost::dynamic_pointer_cast<GridModelRewardMetrics>(reward_metrics);
    //   if (!reward_metrics_derived) {
    //     throw DowncastException("State", "GridState2");
    //   }
    //   if (!reward_metrics_derived->halfway_reached) {
    //     boost::shared_ptr<const GridState2> state_derived = boost::dynamic_pointer_cast<const GridState2>(state);
    //     if (!state_derived) {
    //       throw DowncastException("State", "GridState2");
    //     }
    //     reward_metrics_derived->halfway_reward += reward;

    //     boost::shared_ptr<const GridState2> next_state_derived = boost::dynamic_pointer_cast<const GridState2>(next_state);
    //     if (!next_state_derived) {
    //       throw DowncastException("State", "GridState2");
    //     }
    //     if (abs(next_state_derived->x - params_.grid_size/2) <= params_.grid_size/4 &&
    //         abs(next_state_derived->y - params_.grid_size/2) <= params_.grid_size/4) {
    //       reward_metrics_derived->halfway_reached = true;
    //     }
    //   }
    // }

    post_action_timeout = params_.per_step_planning_time;
  }

  State::ConstPtr GridModel2::getStartState(long seed) const {
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

  float GridModel2::getInitialTimeout() const {
    return params_.initial_planning_time;
  }

  std::map<std::string, std::string> GridModel2::getParamsAsMap() const {
    std::map<std::string, std::string> params_map = params_.asMap();
    // params_map["terminal_states"] = "";
    // int positive_terminal_states = 0;
    // int negative_terminal_states = 0;
    // int num_terminal_states = 0;
    // typedef std::pair<State::ConstPtr, float> State2RewardPair;
    // BOOST_FOREACH(const State2RewardPair& pair, terminal_states_to_reward_map_) {
    //   if (pair.second < 0) {
    //     ++negative_terminal_states;
    //   }
    //   if (pair.second > 0) {
    //     ++positive_terminal_states;
    //   }
    //   ++num_terminal_states;
    // }
    // params_map["positive_terminal_states"] = boost::lexical_cast<std::string>(positive_terminal_states);
    // params_map["negative_terminal_states"] = boost::lexical_cast<std::string>(negative_terminal_states);
    // params_map["num_terminal_states"] = boost::lexical_cast<std::string>(num_terminal_states);
    return params_map;
  }

  std::string GridModel2::getName() const {
    return std::string("Grid2");
  }

} /* utexas_planning */

CLASS_LOADER_REGISTER_CLASS(utexas_planning::GridModel2, utexas_planning::GenerativeModel);
