#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <fstream>

#include <utexas_planning/planners/vi/tabular_estimator.h>

namespace utexas_planning {

  namespace vi {

    bool TabularEstimator::LessState::operator() (const State::ConstPtr &lhs, const State::ConstPtr &rhs) const {
      return *lhs < *rhs;
    }

    TabularEstimator::~TabularEstimator () {}

    void TabularEstimator::getValueAndBestAction(const State::ConstPtr &state,
                                                 float &value,
                                                 Action::ConstPtr &action) const {
      EstimatorTable::const_iterator it = cache_.find(state);
      if (it != cache_.end()) {
        value = it->second.first;
        if (it->second.second) {
          action = it->second.second;
        }
      } else {
        value = 0;
      }
    }

    void TabularEstimator::setValueAndBestAction(const State::ConstPtr &state,
                                                 float value,
                                                 const Action::ConstPtr &action) {
      EstimatorTable::iterator it = cache_.find(state);
      if (it != cache_.end()) {
        it->second.first = value;
        it->second.second = action;
      } else {
        std::pair<float, Action::ConstPtr> pair(value, action);
        cache_[state] = pair;
      }
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
