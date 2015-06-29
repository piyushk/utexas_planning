#ifndef UTEXAS_PLANNING_VI_TABULAR_ESTIMATOR_H_
#define UTEXAS_PLANNING_VI_TABULAR_ESTIMATOR_H_

#include <fstream>
#include <map>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/utility.hpp>

#include <utexas_planning/planning/vi/estimator.h>

namespace utexas_planning {

  namespace vi {

    class TabularEstimator : public Estimator {

      public:

        typedef std::map<boost::shared_ptr<State>, std::pair<float, boost::shared_ptr<Action> >, LessState> EstimatorTable;

        virtual ~TabularEstimator () {}

        virtual void getValueAndBestAction(const State& state, float &value, Action& action) const {
          boost::shared_ptr<State> state_ptr(new State(state));
          EstimatorTable::const_iterator it = cache_.find(state_ptr);
          if (it != cache_.end()) {
            value = it->second.first;
            action = *(it->second.second);
          } else {
            value = 0;
          }
        }

        virtual void setValueAndBestAction(const State& state, float value, const Action& action) {
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

        virtual void loadEstimatedValues(const std::string& file) {
          std::ifstream ifs(file.c_str());
          boost::archive::binary_iarchive ia(ifs);
          ia >> *this;
        }

        virtual void saveEstimatedValues(const std::string& file) {
          std::ofstream ofs(file.c_str());
          boost::archive::binary_oarchive oa(ofs);
          oa << *this;
        }

      private:

        struct LessState : public std::binary_function<const State*, const State*, bool> {
          bool  operator() (const State *lhs, const Vehicle *rhs) const {
            return *lhs < *rhs;
          }
        };

        EstimatorTable cache_;

        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version) {
          ar & BOOST_SERIALIZATION_NVP(cache_);
        }

    };

  } /* vi */

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_VI_TABULAR_ESTIMATOR_H_ */
