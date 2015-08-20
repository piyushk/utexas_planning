#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <class_loader/class_loader.h>

#include "rddl_model.h"

namespace utexas_planning {

  bool RddlAction::operator<(const Action& other_base) const {
    try {
      const RddlAction& other = dynamic_cast<const RddlAction&>(other_base);
      return state < other.state;
    } catch(const std::bad_cast& exp) {
      throw DowncastException("Action", "RddlAction");
    }
  }

  bool RddlAction::operator==(const Action& other_base) const {
    try {
      const RddlAction& other = dynamic_cast<const RddlAction&>(other_base);
      return (!(state < other.state) && !(other.state < state));
    } catch(const std::bad_cast& exp) {
      throw DowncastException("Action", "RddlAction");
    }
  }

  void RddlAction::serialize(std::ostream& stream) const {
    stream << state.getName();
  }

} /* utexas_planning */

BOOST_CLASS_EXPORT_IMPLEMENT(utexas_planning::RddlAction);

namespace utexas_planning {

  bool RddlState::operator<(const State& other_base) const {
    try {
      const RddlState& other = dynamic_cast<const RddlState&>(other_base);
      return state < other.state;
    } catch(const std::bad_cast& exp) {
      throw DowncastException("State", "RddlState");
    }
  }

  void RddlState::serialize(std::ostream& stream) const {
    state.print(stream);
  }

  State::Ptr RddlState::cloneImpl() const {
    boost::shared_ptr<RddlState> clone(new RddlState);
    clone.state.reset(new rddl::State(state));
    return clone;
  }

  std::map<std::string, std::string> RddlState::asMap() const {
    std::map<std::string, std::string> state_map;
    for (int i = 0; i < state.state.size(); ++i) {

    }
    state_map["x"] = boost::lexical_cast<std::string>(x);
    state_map["y"] = boost::lexical_cast<std::string>(y);
    state_map["z"] = boost::lexical_cast<std::string>(z);
    return state_map;
  }

} /* utexas_planning */

BOOST_CLASS_EXPORT_IMPLEMENT(utexas_planning::RddlState);

namespace utexas_planning {

  void RddlModel::init(const YAML::Node& params,
                         const std::string& /* output_directory */,
                         const boost::shared_ptr<RNG>& rng) {

    params_.fromYaml(params);

    // Precache the complete state vector of 100 states.
    for (int x = 0; x < params_.grid_size; ++x) {
      for (int y = 0; y < params_.grid_size; ++y) {
        for (int z = 0; z < params_.grid_size; ++z) {
          boost::shared_ptr<RddlState> s(new RddlState);
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
      boost::shared_ptr<RddlAction> action(new RddlAction);
      action->xdiff = actions[action_type][0];
      action->ydiff = actions[action_type][1];
      action->zdiff = actions[action_type][2];
      default_action_list_.push_back(action);
    }

  }

  bool RddlModel::isTerminalState(const State::ConstPtr& state_base) const {
    boost::shared_ptr<const RddlState> state = boost::dynamic_pointer_cast<const RddlState>(state_base);
    if (!state) {
      throw DowncastException("State", "RddlState");
    }
    return ((state->x == params_.grid_size / 2) &&
            (state->y == params_.grid_size / 2) &&
            (state->z == params_.grid_size / 2));
  }

  void RddlModel::getActionsAtState(const State::ConstPtr& state,
                                    std::vector<Action::ConstPtr>& actions) const {
    actions.clear();
    if (!isTerminalState(state)) {
      actions = default_action_list_;
    }
  }

  std::vector<State::ConstPtr> RddlModel::getStateVector() const {
    return complete_state_vector_;
  }

  void RddlModel::getTransitionDynamics(const State::ConstPtr& state_base,
                                                const Action::ConstPtr& action_base,
                                                std::vector<State::ConstPtr>& next_states,
                                                std::vector<float>& rewards,
                                                std::vector<float>& probabilities) const {

    boost::shared_ptr<const RddlState> state = boost::dynamic_pointer_cast<const RddlState>(state_base);
    if (!state) {
      throw DowncastException("State", "RddlState");
    }

    boost::shared_ptr<const RddlAction> action = boost::dynamic_pointer_cast<const RddlAction>(action_base);
    if (!action) {
      throw DowncastException("Action", "RddlAction");
    }

    next_states.clear();
    rewards.clear();
    probabilities.clear();

    if (!isTerminalState(state)) {

      // Compute next state for each
      BOOST_FOREACH(const Action::ConstPtr& default_action_base, default_action_list_) {

        boost::shared_ptr<const RddlAction> default_action =
          boost::dynamic_pointer_cast<const RddlAction>(default_action_base);

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

  void RddlModel::takeAction(const State::ConstPtr& state,
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

  State::ConstPtr RddlModel::getStartState(long seed) const {
    RNG rng(seed);
    return complete_state_vector_[rng.randomInt(complete_state_vector_.size() - 1)];
    if ((params_.start_x < 0 || params_.start_x >= params_.grid_size) ||
        (params_.start_y < 0 || params_.start_y >= params_.grid_size) ||
        (params_.start_z < 0 || params_.start_z >= params_.grid_size)) {
      RNG rng(seed);
      return complete_state_vector_[rng.randomInt(complete_state_vector_.size() - 1)];
    }
    int idx =
      params_.start_x * params_.grid_size * params_.grid_size +
      params_.start_y * params_.grid_size +
      params_.start_z;
    return complete_state_vector_[idx];
  }

  float RddlModel::getInitialTimeout() const {
    return params_.initial_planning_time;
  }

  std::string RddlModel::getName() const {
    return std::string("Grid3D");
  }

} /* utexas_planning */

CLASS_LOADER_REGISTER_CLASS(utexas_planning::RddlModel, utexas_planning::GenerativeModel);
