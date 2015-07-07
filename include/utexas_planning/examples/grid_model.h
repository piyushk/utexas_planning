#ifndef UTEXAS_PLANNING_GRID_MODEL_H_
#define UTEXAS_PLANNING_GRID_MODEL_H_

#define GRID_SIZE 10

#include <boost/serialization/export.hpp>

#include <utexas_planning/core/declarative_model.h>

namespace utexas_planning {

  enum GridActionType {
    UP = 0,
    DOWN = 1,
    LEFT = 2,
    RIGHT = 3,
    NUM_ACTIONS = 4
  };

  class GridAction : public Action {
    public:
      GridActionType type;
      void serialize(std::ostream& stream) const {
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

    private:
      friend class boost::serialization::access;
      template <typename Archive> void serialize(Archive &ar, const unsigned int version) {
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Action);
        ar & BOOST_SERIALIZATION_NVP(type);
      }
  };

} /* utexas_planning */

BOOST_CLASS_EXPORT(utexas_planning::GridAction);

namespace utexas_planning {

  class GridState : public State {
    public:
      int x;
      int y;

      // boost serialize
      bool operator<(const State& other_base) const {
        try {
          const GridState& other = dynamic_cast<const GridState&>(other_base);
          if (x < other.x) return true;
          if (x > other.x) return false;

          if (y < other.y) return true;
          if (y > other.y) return false;

          return false;
        } catch(const std::bad_cast &exp) {
          throw DowncastException("State", "GridState");
        }
      }

      void serialize(std::ostream& stream) const {
        stream << "(" << x << "," << y << ")";
      }

    private:
      friend class boost::serialization::access;
      template <typename Archive> void serialize(Archive &ar, const unsigned int version) {
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(State);
        ar & BOOST_SERIALIZATION_NVP(x);
        ar & BOOST_SERIALIZATION_NVP(y);
      }
  };

} /* utexas_planning */

BOOST_CLASS_EXPORT(utexas_planning::GridState);

namespace utexas_planning {

  class GridModel : public DeclarativeModel {

    public:

      GridModel() {

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

      bool isTerminalState(const State::ConstPtr &state_base) const {
        boost::shared_ptr<const GridState> state = boost::dynamic_pointer_cast<const GridState>(state_base);
        if (!state) {
          throw DowncastException("State", "GridState");
        }
        return ((state->x == GRID_SIZE / 2) && (state->y == GRID_SIZE / 2));
      }

      void getActionsAtState(const State::ConstPtr &state,
                             std::vector<Action::ConstPtr>& actions) const {
        actions.clear();
        if (!isTerminalState(state)) {
          actions = default_action_list_;
        }
      };

      std::vector<State::ConstPtr> getStateVector() const {
        return complete_state_vector_;
      }

      virtual void getTransitionDynamics(const State::ConstPtr &state_base,
                                         const Action::ConstPtr &action_base,
                                         std::vector<State::ConstPtr> &next_states,
                                         std::vector<float> &rewards,
                                         std::vector<float> &probabilities) const {

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

      };

    private:
      std::vector<State::ConstPtr> complete_state_vector_;
      std::vector<Action::ConstPtr> default_action_list_;
  };

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_GRID_MODEL_H_ */
