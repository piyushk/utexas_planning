#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <class_loader/class_loader.h>
#include <utexas_planning/common/exceptions.h>
#include <utexas_planning/models/grid_model_3d.h>

namespace utexas_planning {

  bool GridAction3D::operator<(const Action& other_base) const {
    try {
      const GridAction3D& other = dynamic_cast<const GridAction3D&>(other_base);
      if (xdiff < other.xdiff) return true;
      if (xdiff > other.xdiff) return false;

      if (ydiff < other.ydiff) return true;
      if (ydiff > other.ydiff) return false;

      if (zdiff < other.zdiff) return true;
      if (zdiff > other.zdiff) return false;

      return false;
    } catch(const std::bad_cast& exp) {
      throw DowncastException("Action", "GridAction3D");
    }
  }

  bool GridAction3D::operator==(const Action& other_base) const {
    try {
      const GridAction3D& other = dynamic_cast<const GridAction3D&>(other_base);
      return
        (xdiff == other.xdiff) &&
        (ydiff == other.ydiff) &&
        (zdiff == other.zdiff);
    } catch(const std::bad_cast& exp) {
      throw DowncastException("Action", "GridAction3D");
    }
  }

  void GridAction3D::serialize(std::ostream& stream) const {
    stream << "(" << xdiff << "," << ydiff << "," << zdiff << ")";
  }

} /* utexas_planning */

BOOST_CLASS_EXPORT_IMPLEMENT(utexas_planning::GridAction3D);

namespace utexas_planning {

  bool GridState3D::operator<(const State& other_base) const {
    try {
      const GridState3D& other = dynamic_cast<const GridState3D&>(other_base);
      if (x < other.x) return true;
      if (x > other.x) return false;

      if (y < other.y) return true;
      if (y > other.y) return false;

      if (z < other.z) return true;
      if (z > other.z) return false;

      return false;
    } catch(const std::bad_cast& exp) {
      throw DowncastException("State", "GridState3D");
    }
  }

  void GridState3D::serialize(std::ostream& stream) const {
    stream << "(" << x << "," << y << "," << z << ")";
  }

  State::Ptr GridState3D::cloneImpl() const {
    boost::shared_ptr<GridState3D> clone(new GridState3D);
    clone->x = x;
    clone->y = y;
    clone->z = z;
    return clone;
  }

  std::map<std::string, std::string> GridState3D::asMap() const {
    std::map<std::string, std::string> state_map;
    state_map["x"] = boost::lexical_cast<std::string>(x);
    state_map["y"] = boost::lexical_cast<std::string>(y);
    state_map["z"] = boost::lexical_cast<std::string>(z);
    return state_map;
  }

} /* utexas_planning */

BOOST_CLASS_EXPORT_IMPLEMENT(utexas_planning::GridState3D);

namespace utexas_planning {

  void GridModel3D::init(const YAML::Node& params,
                         const std::string& /* output_directory */,
                         const boost::shared_ptr<RNG>& rng) {

    params_.fromYaml(params);

    // Precache the complete state vector of 100 states.
    for (int x = 0; x < params_.grid_size; ++x) {
      for (int y = 0; y < params_.grid_size; ++y) {
        for (int z = 0; z < params_.grid_size; ++z) {
          boost::shared_ptr<GridState3D> s(new GridState3D);
          s->x = x;
          s->y = y;
          s->z = z;
          complete_state_vector_.push_back(s);
        }
      }
    }

    int actions[26][3] = {
      {1, 0, 0},
      {0, 1, 0},
      {0, 0, 1},
      {-1, 0, 0},
      {0, -1, 0},
      {0, 0, -1},
      {1, 1, 0},
      {1, -1, 0},
      {1, 0, 1},
      {1, 0, -1},
      {0, 1, 1},
      {0, 1, -1},
      {-1, 1, 0},
      {-1, -1, 0},
      {-1, 0, 1},
      {-1, 0, -1},
      {0, -1, 1},
      {0, -1, -1},
      {1, 1, 1},
      {-1, 1, 1},
      {-1, -1, 1},
      {-1, -1, -1},
      {1, -1, -1},
      {1, 1, -1},
      {1, -1, 1},
      {-1, 1, -1}
    };

    if (params_.num_actions <= 6) {
      params_.num_actions = 6;
    } else if (params_.num_actions <= 18) {
      params_.num_actions = 18;
    } else {
      params_.num_actions = 26;
    }

    // Add all the basic actions.
    for (int action_type = 0; action_type < params_.num_actions; ++action_type) {
      boost::shared_ptr<GridAction3D> action(new GridAction3D);
      action->xdiff = actions[action_type][0];
      action->ydiff = actions[action_type][1];
      action->zdiff = actions[action_type][2];
      default_action_list_.push_back(action);
    }

  }

  bool GridModel3D::isTerminalState(const State::ConstPtr& state_base) const {
    boost::shared_ptr<const GridState3D> state = boost::dynamic_pointer_cast<const GridState3D>(state_base);
    if (!state) {
      throw DowncastException("State", "GridState3D");
    }
    return ((state->x == params_.grid_size / 2) &&
            (state->y == params_.grid_size / 2) &&
            (state->z == params_.grid_size / 2));
  }

  void GridModel3D::getActionsAtState(const State::ConstPtr& state,
                                    std::vector<Action::ConstPtr>& actions) const {
    actions.clear();
    if (!isTerminalState(state)) {
      actions = default_action_list_;
    }
  }

  std::vector<State::ConstPtr> GridModel3D::getStateVector() const {
    return complete_state_vector_;
  }

  void GridModel3D::getTransitionDynamics(const State::ConstPtr& state_base,
                                                const Action::ConstPtr& action_base,
                                                std::vector<State::ConstPtr>& next_states,
                                                std::vector<float>& rewards,
                                                std::vector<float>& probabilities) const {

    boost::shared_ptr<const GridState3D> state = boost::dynamic_pointer_cast<const GridState3D>(state_base);
    if (!state) {
      throw DowncastException("State", "GridState3D");
    }

    boost::shared_ptr<const GridAction3D> action = boost::dynamic_pointer_cast<const GridAction3D>(action_base);
    if (!action) {
      throw DowncastException("Action", "GridAction3D");
    }

    next_states.clear();
    rewards.clear();
    probabilities.clear();

    if (!isTerminalState(state)) {

      // Compute next state for each
      BOOST_FOREACH(const Action::ConstPtr& default_action_base, default_action_list_) {

        boost::shared_ptr<const GridAction3D> default_action =
          boost::dynamic_pointer_cast<const GridAction3D>(default_action_base);

        int new_x = state->x + default_action->xdiff;
        new_x = (new_x == -1) ? params_.grid_size - 1 : new_x;
        new_x = (new_x == params_.grid_size) ? 0 : new_x;
        int new_y = state->y + default_action->ydiff;
        new_y = (new_y == -1) ? params_.grid_size - 1 : new_y;
        new_y = (new_y == params_.grid_size) ? 0 : new_y;
        int new_z = state->z + default_action->zdiff;
        new_z = (new_z == -1) ? params_.grid_size - 1 : new_z;
        new_z = (new_z == params_.grid_size) ? 0 : new_z;

        // Ordered in row major form.
        int idx =
          new_x * params_.grid_size * params_.grid_size +
          new_y * params_.grid_size +
          new_z;
        next_states.push_back(complete_state_vector_[idx]);

        // Decide probabilities as necessary.
        float probability = params_.non_determinism / params_.num_actions;
        if (*default_action == *action) {
          probability += (1 - params_.non_determinism);
        }
        probabilities.push_back(probability);

        rewards.push_back(-1);
      }

    }

  }

  void GridModel3D::takeAction(const State::ConstPtr& state,
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

  State::ConstPtr GridModel3D::getStartState(long seed) {
    RNG rng(seed);
    int idx;
    if ((params_.start_x > 0 && params_.start_x < params_.grid_size) &&
        (params_.start_y > 0 && params_.start_y < params_.grid_size) &&
        (params_.start_z > 0 && params_.start_z < params_.grid_size)) {
      idx =
        params_.start_x * params_.grid_size * params_.grid_size +
        params_.start_y * params_.grid_size +
        params_.start_z;
      if (isTerminalState(complete_state_vector_[idx])) {
        throw IncorrectUsageException("GridModel3D: Provided start state is terminal.");
      }
    } else {
      if (params_.goal_in_cardinal_direction) {
        int directions[6][3] = {
          {1, 0, 0},
          {0, 1, 0},
          {0, 0, 1},
          {-1, 0, 0},
          {0, -1, 0},
          {0, 0, -1}
        };
        // First pick a direction.
        int dir_idx = rng.randomInt(5); // max value is included.

        int distance;
        if (params_.goal_distance > 0) {
          distance = std::min(params_.goal_distance, params_.grid_size / 2);
        } else {
          distance = rng.randomInt(1, params_.grid_size / 2); // A minimum of 1 prevents start == goal
        }

        int start_x = params_.grid_size / 2 + distance * directions[dir_idx][0];
        int start_y = params_.grid_size / 2 + distance * directions[dir_idx][1];
        int start_z = params_.grid_size / 2 + distance * directions[dir_idx][2];

        if (start_x == params_.grid_size) start_x = 0; // Keeps the distance same.
        if (start_y == params_.grid_size) start_y = 0;
        if (start_z == params_.grid_size) start_z = 0;

        idx = start_x * params_.grid_size * params_.grid_size + start_y * params_.grid_size + start_z;
      } else {
        idx = rng.randomInt(complete_state_vector_.size() - 1);
        while (isTerminalState(complete_state_vector_[idx])) {
          idx = rng.randomInt(complete_state_vector_.size() - 1);
        }
      }
    }
    return complete_state_vector_[idx];
  }

  float GridModel3D::getInitialTimeout() const {
    return params_.initial_planning_time;
  }

  std::map<std::string, std::string> GridModel3D::getParamsAsMap() const {
    return params_.asMap();
  }

  std::string GridModel3D::getName() const {
    return std::string("Grid3D");
  }

} /* utexas_planning */

CLASS_LOADER_REGISTER_CLASS(utexas_planning::GridModel3D, utexas_planning::GenerativeModel);
