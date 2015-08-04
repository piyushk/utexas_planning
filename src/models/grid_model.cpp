#include <utexas_planning/common/exceptions.h>
#include <utexas_planning/models/grid_model.h>

#define GRID_SIZE 10

namespace utexas_planning {

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

} /* utexas_planning */

BOOST_CLASS_EXPORT_IMPLEMENT(utexas_planning::GridState);

namespace utexas_planning {

  GridModel::GridModel() {

    // Precache the complete state vector of 100 states.
    for (int x = 0; x < GRID_SIZE; ++x) {
      for (int y = 0; y < GRID_SIZE; ++y) {
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

  }

  bool GridModel::isTerminalState(const State::ConstPtr& state_base) const {
    boost::shared_ptr<const GridState> state = boost::dynamic_pointer_cast<const GridState>(state_base);
    if (!state) {
      throw DowncastException("State", "GridState");
    }
    return ((state->x == GRID_SIZE / 2) && (state->y == GRID_SIZE / 2));
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
      ns->y = (state->y == 0) ? GRID_SIZE - 1 : state->y - 1;
      next_states.push_back(ns);
      ns.reset(new GridState);
      ns->x = state->x;
      ns->y = (state->y + 1) % GRID_SIZE;
      next_states.push_back(ns);
      ns.reset(new GridState);
      ns->x = (state->x == 0) ? GRID_SIZE - 1 : state->x - 1;
      ns->y = state->y;
      next_states.push_back(ns);
      ns.reset(new GridState);
      ns->x = (state->x + 1) % GRID_SIZE;
      ns->y = state->y;
      next_states.push_back(ns);

      rewards.push_back(-1);
      rewards.push_back(-1);
      rewards.push_back(-1);
      rewards.push_back(-1);

      float p_up = (action->type == UP) ? 0.7f : 0.1f;
      float p_down = (action->type == DOWN) ? 0.7f : 0.1f;
      float p_left = (action->type == LEFT) ? 0.7f : 0.1f;
      float p_right = (action->type == RIGHT) ? 0.7f : 0.1f;

      probabilities.push_back(p_up);
      probabilities.push_back(p_down);
      probabilities.push_back(p_left);
      probabilities.push_back(p_right);

    }

  }

  State::ConstPtr GridModel::getStartState(long seed) const {
    RNG rng(seed);
    return complete_state_vector_[rng.randomInt(complete_state_vector_.size() - 1)];
  }

} /* utexas_planning */
