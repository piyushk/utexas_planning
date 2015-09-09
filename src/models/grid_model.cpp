#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <class_loader/class_loader.h>
#include <utexas_planning/common/exceptions.h>
#include <utexas_planning/models/grid_model.h>

namespace utexas_planning {

  bool GridAction::operator<(const Action& other_base) const {
    try {
      const GridAction& other = dynamic_cast<const GridAction&>(other_base);
      return type < other.type;
    } catch(const std::bad_cast& exp) {
      throw DowncastException("Action", "GridAction");
    }
  }

  void GridAction::serialize(std::ostream& stream) const {
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

BOOST_CLASS_EXPORT_IMPLEMENT(utexas_planning::GridAction);

namespace utexas_planning {

  bool GridState::operator<(const State& other_base) const {
    try {
      const GridState& other = dynamic_cast<const GridState&>(other_base);
      if (x < other.x) return true;
      if (x > other.x) return false;

      if (y < other.y) return true;
      if (y > other.y) return false;

      return false;
    } catch(const std::bad_cast& exp) {
      throw DowncastException("State", "GridState");
    }
  }

  void GridState::serialize(std::ostream& stream) const {
    stream << "(" << x << "," << y << ")";
  }

  State::Ptr GridState::cloneImpl() const {
    boost::shared_ptr<GridState> clone(new GridState);
    clone->x = x;
    clone->y = y;
    return clone;
  }

  std::map<std::string, std::string> GridState::asMap() const {
    std::map<std::string, std::string> state_map;
    state_map["x"] = boost::lexical_cast<std::string>(x);
    state_map["y"] = boost::lexical_cast<std::string>(y);
    return state_map;
  }

} /* utexas_planning */

BOOST_CLASS_EXPORT_IMPLEMENT(utexas_planning::GridState);

namespace utexas_planning {

  void GridModel::init(const YAML::Node& params,
                       const std::string& /* output_directory */,
                       const boost::shared_ptr<RNG>& rng) {

    params_.fromYaml(params);

    // Precache the complete state vector of 100 states.
    for (int x = 0; x < params_.grid_size; ++x) {
      for (int y = 0; y < params_.grid_size; ++y) {
        boost::shared_ptr<GridState> s(new GridState);
        s->x = x;
        s->y = y;
        complete_state_vector_.push_back(s);
      }
    }

    for (int action_type = 0; action_type < NUM_ACTIONS; ++action_type) {
      boost::shared_ptr<GridAction> a(new GridAction);
      a->type = (GridActionType)action_type;
      default_action_list_.push_back(a);
    }

    // If no terminal state specified, assume it is center of the grid.
    if (params_.terminal_states.empty()) {
      int idx =
        params_.grid_size / 2 +
        (params_.grid_size / 2) * (params_.grid_size);
      terminal_states_to_reward_map_[complete_state_vector_[idx]] = params_.goal_reward;
    } else {
      std::vector<std::string> terminal_state_tuples;
      boost::split(terminal_state_tuples, params_.terminal_states, boost::is_any_of(";"));
      BOOST_FOREACH(const std::string& tuple, terminal_state_tuples) {
        std::vector<std::string> coordinates;
        boost::split(coordinates, tuple, boost::is_any_of(","));
        int x_idx = boost::lexical_cast<int>(coordinates[0]);
        int y_idx = boost::lexical_cast<int>(coordinates[1]);
        float reward = 0.0f;
        if (coordinates.size() == 3) {
          reward = boost::lexical_cast<float>(coordinates[2]);
        }
        int idx = y_idx + x_idx * params_.grid_size;
        terminal_states_to_reward_map_[complete_state_vector_[idx]] = reward;
      }
    }
  }

  bool GridModel::isTerminalState(const State::ConstPtr& state_base) const {
    boost::shared_ptr<const GridState> state = boost::dynamic_pointer_cast<const GridState>(state_base);
    if (!state) {
      throw DowncastException("State", "GridState");
    }
    return terminal_states_to_reward_map_.find(state) != terminal_states_to_reward_map_.end();
  }

  void GridModel::getActionsAtState(const State::ConstPtr& state,
                                    std::vector<Action::ConstPtr>& actions) const {
    actions.clear();
    if (!isTerminalState(state)) {
      actions = default_action_list_;
    }
  }

  std::vector<State::ConstPtr> GridModel::getStateVector() const {
    return complete_state_vector_;
  }

  void GridModel::getTransitionDynamics(const State::ConstPtr& state_base,
                                                const Action::ConstPtr& action_base,
                                                std::vector<State::ConstPtr>& next_states,
                                                std::vector<float>& rewards,
                                                std::vector<float>& probabilities) const {

    boost::shared_ptr<const GridState> state = boost::dynamic_pointer_cast<const GridState>(state_base);
    if (!state) {
      throw DowncastException("State", "GridState");
    }

    boost::shared_ptr<const GridAction> action = boost::dynamic_pointer_cast<const GridAction>(action_base);
    if (!action) {
      throw DowncastException("Action", "GridAction");
    }

    next_states.clear();
    rewards.clear();
    probabilities.clear();

    if (!isTerminalState(state)) {
      boost::shared_ptr<GridState> ns(new GridState);
      ns->x = state->x;
      ns->y = (state->y == 0) ? params_.grid_size - 1 : state->y - 1;
      next_states.push_back(ns);
      ns.reset(new GridState);
      ns->x = state->x;
      ns->y = (state->y + 1) % params_.grid_size;
      next_states.push_back(ns);
      ns.reset(new GridState);
      ns->x = (state->x == 0) ? params_.grid_size - 1 : state->x - 1;
      ns->y = state->y;
      next_states.push_back(ns);
      ns.reset(new GridState);
      ns->x = (state->x + 1) % params_.grid_size;
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

  void GridModel::takeAction(const State::ConstPtr& state,
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

    if (reward_metrics) {
      boost::shared_ptr<GridModelRewardMetrics> reward_metrics_derived =
        boost::dynamic_pointer_cast<GridModelRewardMetrics>(reward_metrics);
      if (!reward_metrics_derived) {
        throw DowncastException("State", "GridState");
      }
      if (!reward_metrics_derived->halfway_reached) {
        boost::shared_ptr<const GridState> state_derived = boost::dynamic_pointer_cast<const GridState>(state);
        if (!state_derived) {
          throw DowncastException("State", "GridState");
        }
        reward_metrics_derived->halfway_reward += reward;

        boost::shared_ptr<const GridState> next_state_derived = boost::dynamic_pointer_cast<const GridState>(next_state);
        if (!next_state_derived) {
          throw DowncastException("State", "GridState");
        }
        if (abs(next_state_derived->x - params_.grid_size/2) <= params_.grid_size/4 &&
            abs(next_state_derived->y - params_.grid_size/2) <= params_.grid_size/4) {
          reward_metrics_derived->halfway_reached = true;
        }
      }
    }

    post_action_timeout = params_.per_step_planning_time;
  }

  State::ConstPtr GridModel::getStartState(long seed) const {
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

  float GridModel::getInitialTimeout() const {
    return params_.initial_planning_time;
  }

  std::map<std::string, std::string> GridModel::getParamsAsMap() const {
    return params_.asMap();
  }

  std::string GridModel::getName() const {
    return std::string("Grid");
  }

  RewardMetrics::Ptr GridModel::getRewardMetricsAtEpisodeStart() const {
    GridModelRewardMetrics::Ptr metric(new GridModelRewardMetrics);
    metric->halfway_reached = false;
    metric->halfway_reward = 0.0f;
    return metric;
  }

} /* utexas_planning */

CLASS_LOADER_REGISTER_CLASS(utexas_planning::GridModel, utexas_planning::GenerativeModel);
