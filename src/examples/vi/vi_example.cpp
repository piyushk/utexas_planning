/**
 * \file  vi_example.cpp
 * \brief A simple example demonstrating how to use Value Iteration using the bwi_rl code base to solve for an agent
 *        trying to find a goal in 10x by 10x torus grid.
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 *
 * Copyright (c) 2015, UT Austin

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *
 * $ Id: 05/26/2015 05:00:28 PM piyushk $
 *
 **/

#include <boost/serialization/export.hpp>

#include <utexas_planning/common/exceptions.h>
#include <utexas_planning/core/abstract_planner.h>
#include <utexas_planning/core/declarative_model.h>

#include <utexas_planning/planners/vi/vi.h>

#define GRID_SIZE 10

enum ActionType {
  UP = 0,
  DOWN = 1,
  LEFT = 2,
  RIGHT = 3,
  NUM_ACTIONS = 4
};

class Action : public utexas_planning::Action {
  public:
    ActionType type;
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

    std::string getName() const {
      return std::string("GridAction");
    }

  private:
  friend class boost::serialization::access;
  template <typename Archive> void serialize(Archive &ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(utexas_planning::Action);
    ar & BOOST_SERIALIZATION_NVP(type);
  }
};

BOOST_CLASS_EXPORT(Action);

class State : public utexas_planning::State {
  public:
    int x;
    int y;

    // boost serialize
    bool operator<(const utexas_planning::State& other_base) const {
      try {
        const State& other = dynamic_cast<const State&>(other_base);
        if (x < other.x) return true;
        if (x > other.x) return false;

        if (y < other.y) return true;
        if (y > other.y) return false;

        return false;
      } catch(std::bad_cast exp) {
        throw utexas_planning::DowncastException("State", "GridState");
      }
    }

    void serialize(std::ostream& stream) const {
      stream << "(" << x << "," << y << ")";
    }

    std::string getName() const {
      return std::string("GridState");
    }

  private:
  friend class boost::serialization::access;
  template <typename Archive> void serialize(Archive &ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(utexas_planning::State);
    ar & BOOST_SERIALIZATION_NVP(x);
    ar & BOOST_SERIALIZATION_NVP(y);
  }
};

BOOST_CLASS_EXPORT(State);

class GridModel : public utexas_planning::DeclarativeModel {

  public:

    GridModel() {

      // Precache the complete state vector of 100 states.
      for (int x = 0; x < GRID_SIZE; ++x) {
        for (int y = 0; y < GRID_SIZE; ++y) {
          utexas_planning::State::Ptr s_base(new State);
          boost::shared_ptr<State> s = boost::static_pointer_cast<State>(s_base);
          s->x = x;
          s->y = y;
          complete_state_vector_.push_back(s_base);
        }
      }

      for (int action_type = 0; action_type < NUM_ACTIONS; ++action_type) {
        utexas_planning::Action::Ptr a_base(new Action);
        boost::shared_ptr<Action> a = boost::static_pointer_cast<Action>(a_base);
        a->type = (ActionType)action_type;
        default_action_list_.push_back(a_base);
      }
    }

    std::string getName() const {
      return std::string("GridModel");
    }

    bool isTerminalState(const utexas_planning::State::ConstPtr &state_base) const {
      boost::shared_ptr<const State> state = boost::dynamic_pointer_cast<const State>(state_base);
      if (!state) {
        throw utexas_planning::DowncastException("State", "GridState");
      }
      return ((state->x == GRID_SIZE / 2) && (state->y == GRID_SIZE / 2));
    }

    void getActionsAtState(const utexas_planning::State::ConstPtr &state,
                           std::vector<utexas_planning::Action::ConstPtr>& actions) const {
      actions.clear();
      if (!isTerminalState(state)) {
        actions = default_action_list_;
      }
    };

    std::vector<utexas_planning::State::ConstPtr> getStateVector() const {
      return complete_state_vector_;
    }

    virtual void getTransitionDynamics(const utexas_planning::State::ConstPtr &state_base,
                                       const utexas_planning::Action::ConstPtr &action_base,
                                       std::vector<utexas_planning::State::ConstPtr> &next_states,
                                       std::vector<float> &rewards,
                                       std::vector<float> &probabilities) const {

      boost::shared_ptr<const State> state = boost::dynamic_pointer_cast<const State>(state_base);
      if (!state) {
        throw utexas_planning::DowncastException("State", "GridState");
      }

      boost::shared_ptr<const Action> action = boost::dynamic_pointer_cast<const Action>(action_base);
      if (!action) {
        throw utexas_planning::DowncastException("Action", "GridAction");
      }

      next_states.clear();
      rewards.clear();
      probabilities.clear();

      if (!isTerminalState(state)) {
        boost::shared_ptr<State> ns(new State);
        ns->x = state->x;
        ns->y = (state->y == 0) ? GRID_SIZE - 1 : state->y - 1;
        next_states.push_back(ns);
        ns.reset(new State);
        ns->x = state->x;
        ns->y = (state->y + 1) % GRID_SIZE;
        next_states.push_back(ns);
        ns.reset(new State);
        ns->x = (state->x == 0) ? GRID_SIZE - 1 : state->x - 1;
        ns->y = state->y;
        next_states.push_back(ns);
        ns.reset(new State);
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
    std::vector<utexas_planning::State::ConstPtr> complete_state_vector_;
    std::vector<utexas_planning::Action::ConstPtr> default_action_list_;
};

int main(int argc, char **argv) {

  utexas_planning::GenerativeModel::Ptr model(new GridModel);
  utexas_planning::AbstractPlanner::Ptr solver(new utexas_planning::vi::ValueIteration);
  YAML::Node empty_params;
  std::string out_dir = "/tmp/test";
  solver->init(model, empty_params, out_dir);
  std::cout << "Computing policy..." << std::endl;

  solver->performEpisodeStartProcessing(State::ConstPtr());

  std::vector<boost::shared_ptr<State> > test_states;
  boost::shared_ptr<State> test(new State);
  test->x = 5;
  test->y = 2;
  test_states.push_back(test);
  test.reset(new State);
  test->x = 2;
  test->y = 5;
  test_states.push_back(test);
  test.reset(new State);
  test->x = 5;
  test->y = 8;
  test_states.push_back(test);
  test.reset(new State);
  test->x = 8;
  test->y = 5;
  test_states.push_back(test);

  BOOST_FOREACH(const boost::shared_ptr<State>& s, test_states) {
    utexas_planning::Action::ConstPtr action_base = solver->getBestAction(s);
    boost::shared_ptr<const Action> action = boost::dynamic_pointer_cast<const Action>(action_base);
    if (!action) {
      throw utexas_planning::DowncastException("Action", "GridAction");
    }
    std::cout << "Best action at state " << *s << " is " << *action << std::endl;
  }

  return 0;
}
