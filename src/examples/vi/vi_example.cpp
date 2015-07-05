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

#include <boost/serialization/export.
#include <utexas_planning/core/declarative_model.h>

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
};

std::ostream& operator<<(std::ostream& stream, const Action& action) {
  if (action.type == UP) {
    stream << "Up";
  } else if (action.type == DOWN) {
    stream << "Down";
  } else if (action.type == LEFT) {
    stream << "Left";
  } else {
    stream << "Right";
  }
  return stream;
}

class State : public utexas_planning::State {
  public:
    int x;
    int y;

    // boost serialize
    bool operator<(const utexas_planning::State& other) const {
      //TODO need dynamic cast here
      if (x < other.x) return true;
      if (x > other.x) return false;

      if (y < other.y) return true;
      if (y > other.y) return false;
    }

  private:
  friend class boost::serialization::access;
  template <typename Archive> void serialize(Archive &ar, const unsigned int version) {
    ar & x;
    ar & y;
  }
};

// TODO: not sure what the hell is going on with this.
// http://stackoverflow.com/questions/3396330/where-to-put-boost-class-export-for-boostserialization
BOOST_CLASS_EXPORT(State);

std::ostream& operator<<(std::ostream& stream, const State& s) {
  stream << "(" << s.x << "," << s.y << ")";
  return stream;
}

class GridModel : public utexas_planning::DeclarativeModel {

  public:

    GridModel() {

      // Precache the complete state vector of 100 states.
      for (int x = 0; x < GRID_SIZE; ++x) {
        for (int y = 0; y < GRID_SIZE; ++y) {
          utexas_planning::State::Ptr s_base(new State);
          boost::shared_ptr<State> s_derived = boost::static_pointer_cast<State>(s_base);
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

    std::string getModelName() {
      return std::string("GridModel");
    }

    bool isTerminalState(const utexas_planning::State::ConstPtr &state_base) const {
      boost::shared_ptr<State> state = boost::dynamic_pointer_cast<State>(state_base);
      if (!state) {
        throw std::runtime_error("GridModel: Unable to type cast State::ConstPtr to derived type in isTerminalState");
      }
      return ((state->x == GRID_SIZE / 2) &&
              (state->y == GRID_SIZE / 2));
    }

    void getActionsAtState(const utexas_planning::State::ConstPtr &state,
                           std::vector<utexas_planning::Action::ConstPtr>& actions) {
      actions.clear();
      if (!isTerminalState(state)) {
        actions = default_action_list_;
      }
    };

    void getStateVector(std::vector<utexas_planning::State::ConstPtr>& states) {
      states = complete_state_vector_;
    }

    virtual void getTransitionDynamics(const utexas_planning::State::ConstPtr &state_base,
                                       const utexas_planning::Action::ConstPtr &action_base,
                                       std::vector<utexas_planning::State::ConstPtr> &next_states,
                                       std::vector<float> &rewards,
                                       std::vector<float> &probabilities) const {

      next_states.clear();
      rewards.clear();
      probabilities.clear();

      boost::shared_ptr<State> state = boost::dynamic_pointer_cast<State>(state_base);
      boost::shared_ptr<Action> action = boost::dynamic_pointer_cast<Action>(action_base);

      if (!isTerminalState(state)) {
        utexas_planning::NextState::Ptr ns(new State);
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

// int main(int argc, char **argv) {
//   boost::shared_ptr<PredictiveModel<State, Action> > model(new GridModel);
//   boost::shared_ptr<VIEstimator<State, Action> > estimator(new VITabularEstimator<State, Action>);
//
//   ValueIteration<State, Action> vi(model, estimator);
//   std::cout << "Computing policy..." << std::endl;
//   vi.computePolicy();
//
//   std::vector<State> test_states;
//   State test;
//   test.x = 5;
//   test.y = 2;
//   test_states.push_back(test);
//   test.x = 2;
//   test.y = 5;
//   test_states.push_back(test);
//   test.x = 5;
//   test.y = 8;
//   test_states.push_back(test);
//   test.x = 8;
//   test.y = 5;
//   test_states.push_back(test);
//
//   BOOST_FOREACH(const State& s, test_states) {
//     std::cout << "Best action at state " << s << " is " << vi.getBestAction(s) << std::endl;
//     // std::vector<Action> actions;
//     // model->getActionsAtState(s, actions);
//     // BOOST_FOREACH(const Action& a, actions) {
//     //   std::cout << "  Next state distributions after taking action " << a << std::endl;
//     //   std::vector<State> ns;
//     //   std::vector<float> r;
//     //   std::vector<float> p;
//     //   model->getTransitionDynamics(s, a, ns, r, p);
//     //   int counter = 0;
//     //   BOOST_FOREACH(const State& s2, ns) {
//     //     std::cout << "    One possible next state is " << s2 << " with probability " << p[counter] << " and reward " << r[counter] << std::endl;
//     //     ++counter;
//     //   }
//     // }
//   }
//
//   return 0;
// }
