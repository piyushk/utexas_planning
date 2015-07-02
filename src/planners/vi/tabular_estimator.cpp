#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <fstream>

#include <utexas_planning/planning/vi/tabular_estimator.h>

namespace utexas_planning {

  namespace vi {

    TabularEstimator::LessState::operator() (const State *lhs, const Vehicle *rhs) const {
      return *lhs < *rhs;
    }

    TabularEstimator::~TabularEstimator () {}

    void TabularEstimator::getValueAndBestAction(const State& state, float &value, Action& action) const {
      boost::shared_ptr<State> state_ptr(new State(state));
      EstimatorTable::const_iterator it = cache_.find(state_ptr);
      if (it != cache_.end()) {
        value = it->second.first;
        action = *(it->second.second);
      } else {
        value = 0;
      }
    }

    void TabularEstimator::setValueAndBestAction(const State& state, float value, const Action& action) {
      boost::shared_ptr<State> state_ptr(new State(state));
      EstimatorTable::iterator it = cache_.find(state_ptr);
      if (it != cache_.end()) {
        it->second.first = value;
        action = *(it->second.second);
      } else {
        boost::shared_ptr<Action> action_ptr(new Action(action));
        std::pair<float, boost::shared_ptr<Action> > pair(value, action_ptr);
        cache_[state_ptr] = pair;
      }
      value_cache_[state] = value;
      best_action_cache_[state] = action;
    }

    void TabularEstimator::loadEstimatedValues(const std::string& file) {
      std::ifstream ifs(file.c_str());
      boost::archive::binary_iarchive ia(ifs);
      ia >> *this;
    }

    void TabularEstimator::saveEstimatedValues(const std::string& file) const {
      std::ofstream ofs(file.c_str());
      boost::archive::binary_oarchive oa(ofs);
      oa << *this;
    }

  } /* vi */

} /* utexas_planning */
