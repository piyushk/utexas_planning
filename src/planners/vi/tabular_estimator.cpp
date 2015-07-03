#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <fstream>

#include <utexas_planning/planners/vi/tabular_estimator.h>

namespace utexas_planning {

  namespace vi {

    bool TabularEstimator::LessState::operator() (const State *lhs, const State *rhs) const {
      //TODO return *lhs < *rhs;
      return true;
    }

    TabularEstimator::~TabularEstimator () {}

    void TabularEstimator::getValueAndBestAction(const boost::shared_ptr<const State> &state,
                                                 float &value,
                                                 boost::shared_ptr<const Action> &action) const {
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

    void TabularEstimator::setValueAndBestAction(const boost::shared_ptr<const State> &state,
                                                 float value,
                                                 const boost::shared_ptr<const Action> &action = boost::shared_ptr<const Action>());
      EstimatorTable::iterator it = cache_.find(state);
      if (it != cache_.end()) {
        it->second.first = value;
        it->second.second = action;
      } else {
        std::pair<float, boost::shared_ptr<Action> > pair(value, action);
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
