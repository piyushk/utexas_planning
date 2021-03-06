#include <fstream>

#include <utexas_planning/planners/vi/tabular_estimator.h>

namespace utexas_planning {

  namespace vi {

    TabularEstimator::~TabularEstimator () {}

    float TabularEstimator::getValue(const State::ConstPtr& state) const {
      EstimatorTable::const_iterator it = cache_.find(state);
      return (it != cache_.end()) ? it->second.first : 0.0f;
    }

    Action::ConstPtr TabularEstimator::getBestAction(const State::ConstPtr& state) const {
      EstimatorTable::const_iterator it = cache_.find(state);
      return (it != cache_.end()) ? it->second.second : Action::ConstPtr();
    }

    void TabularEstimator::setValueAndBestAction(const State::ConstPtr& state,
                                                 float value,
                                                 const Action::ConstPtr& action) {
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
