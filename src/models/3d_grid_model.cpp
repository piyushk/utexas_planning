#include <boost/lexical_cast.hpp>
#include <class_loader/class_loader.h>
#include <utexas_planning/common/exceptions.h>
#include <utexas_planning/models/grid_model.h>

#define GRID_SIZE 10

namespace utexas_planning {

  bool 3DGridAction::operator<(const Action& other_base) const {
    try {
      const 3DGridAction& other = dynamic_cast<const 3DGridAction&>(other_base);
      if (xdiff < other.xdiff) return true;
      if (xdiff > other.xdiff) return false;

      if (ydiff < other.ydiff) return true;
      if (ydiff > other.ydiff) return false;

      if (zdiff < other.zdiff) return true;
      if (zdiff > other.zdiff) return false;

      return false;
    } catch(const std::bad_cast& exp) {
      throw DowncastException("Action", "3DGridAction");
    }
  }

  void 3DGridAction::serialize(std::ostream& stream) const {
    stream << "(" << xdiff << "," << ydiff << "," << zdiff << ")";
  }

} /* utexas_planning */

BOOST_CLASS_EXPORT_IMPLEMENT(utexas_planning::3DGridAction);

namespace utexas_planning {

  bool 3DGridState::operator<(const State& other_base) const {
    try {
      const 3DGridState& other = dynamic_cast<const 3DGridState&>(other_base);
      if (x < other.x) return true;
      if (x > other.x) return false;

      if (y < other.y) return true;
      if (y > other.y) return false;

      if (z < other.z) return true;
      if (z > other.z) return false;

      return false;
    } catch(const std::bad_cast& exp) {
      throw DowncastException("State", "3DGridState");
    }
  }

  void 3DGridState::serialize(std::ostream& stream) const {
    stream << "(" << x << "," << y << "," << z << ")";
  }

  State::Ptr 3DGridState::cloneImpl() const {
    boost::shared_ptr<3DGridState> clone(new 3DGridState);
    clone->x = x;
    clone->y = y;
    return clone;
  }

  std::map<std::string, std::string> 3DGridState::asMap() const {
    std::map<std::string, std::string> state_map;
    state_map["x"] = boost::lexical_cast<std::string>(x);
    state_map["y"] = boost::lexical_cast<std::string>(y);
    state_map["z"] = boost::lexical_cast<std::string>(z);
    return state_map;
  }

} /* utexas_planning */

BOOST_CLASS_EXPORT_IMPLEMENT(utexas_planning::3DGridState);

namespace utexas_planning {

  void 3DGridModel::init(const YAML::Node& params, const std::string& /* output_directory */) {

    params_.fromYaml(params);

    // Precache the complete state vector of 100 states.
    for (int x = 0; x < params_.grid_size; ++x) {
      for (int y = 0; y < params_.grid_size; ++y) {
        for (int z = 0; z < params_.grid_size; ++z) {
        boost::shared_ptr<3DGridState> s(new 3DGridState);
        s->x = x;
        s->y = y;
        s->z = z;
        complete_state_vector_.push_back(s);
      }
    }

    params_.num_actions = std::max(params_.num_actions, 6);
    params_.num_actions = std::min(params_.num_actions, 26);

    int actions[][] = {
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
    }

    for (int action_type = 0; action_type < 6; ++action_type) {
      a->type = (GridActionType)action_type;
      default_action_list_.push_back(a);
    }

  }

  bool 3DGridModel::isTerminalState(const State::ConstPtr& state_base) const {
    boost::shared_ptr<const 3DGridState> state = boost::dynamic_pointer_cast<const 3DGridState>(state_base);
    if (!state) {
      throw DowncastException("State", "3DGridState");
    }
    return ((state->x == params_.grid_size / 2) &&
            (state->y == params_.grid_size / 2) &&
            (state->z == params_.grid_size / 2));
  }

  void 3DGridModel::getActionsAtState(const State::ConstPtr& state,
                                    std::vector<Action::ConstPtr>& actions) const {
    actions.clear();
    if (!isTerminalState(state)) {
      actions = default_action_list_;
    }
  }

  std::vector<State::ConstPtr> 3DGridModel::getStateVector() const {
    return complete_state_vector_;
  }

  void 3DGridModel::getTransitionDynamics(const State::ConstPtr& state_base,
                                                const Action::ConstPtr& action_base,
                                                std::vector<State::ConstPtr>& next_states,
                                                std::vector<float>& rewards,
                                                std::vector<float>& probabilities) const {

    boost::shared_ptr<const 3DGridState> state = boost::dynamic_pointer_cast<const 3DGridState>(state_base);
    if (!state) {
      throw DowncastException("State", "3DGridState");
    }

    boost::shared_ptr<const 3DGridAction> action = boost::dynamic_pointer_cast<const 3DGridAction>(action_base);
    if (!action) {
      throw DowncastException("Action", "3DGridAction");
    }

    next_states.clear();
    rewards.clear();
    probabilities.clear();

    if (!isTerminalState(state)) {
      boost::shared_ptr<3DGridState> ns(new 3DGridState);
      ns->x = state->x;
      ns->y = (state->y == 0) ? GRID_SIZE - 1 : state->y - 1;
      next_states.push_back(ns);
      ns.reset(new 3DGridState);
      ns->x = state->x;
      ns->y = (state->y + 1) % GRID_SIZE;
      next_states.push_back(ns);
      ns.reset(new 3DGridState);
      ns->x = (state->x == 0) ? GRID_SIZE - 1 : state->x - 1;
      ns->y = state->y;
      next_states.push_back(ns);
      ns.reset(new 3DGridState);
      ns->x = (state->x + 1) % GRID_SIZE;
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

  State::ConstPtr 3DGridModel::getStartState(long seed) const {
    RNG rng(seed);
    return complete_state_vector_[rng.randomInt(complete_state_vector_.size() - 1)];
  }

} /* utexas_planning */

CLASS_LOADER_REGISTER_CLASS(utexas_planning::3DGridModel, utexas_planning::GenerativeModel);
