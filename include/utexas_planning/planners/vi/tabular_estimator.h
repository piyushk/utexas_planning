#ifndef UTEXAS_PLANNING_VI_TABULAR_ESTIMATOR_H_
#define UTEXAS_PLANNING_VI_TABULAR_ESTIMATOR_H_

#include <fstream>
#include <map>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/map.hpp>

#include <utexas_planning/planning/vi/estimator.h>

namespace utexas_planning {
  
  namespace vi {

    class TabularEstimator : public Estimator {

      public:
        TabularEstimator (const boost::shared_ptr<const DeclarativeModel> &model) {
          boost::shared_array<const State> states;
          unsigned int num_states;
          model->getStateVector(states, num_states);
          value_cache_.resize(num_states);
          best_action_idx_cache_.resize(num_states);
        }

        virtual ~TabularEstimator () {}

        virtual void getValueAndBestActionIdx(unsigned int state_idx, float &value, unsigned int &action_idx) const {
          value = value_cache_[state_idx];
          action_idx = best_action_idx_cache_[state_idx];
        }

        virtual void setValueAndBestActionIdx(unsigned int state_idx, float value, unsigned int action_idx) {
          value_cache_[state_idx] = value;
          best_action_idx_cache_[state_idx] = action_idx;
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

        std::vector<float> value_cache_;
        std::vector<int> best_action_idx_cache_;

        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version) {
          ar & BOOST_SERIALIZATION_NVP(value_cache_);
          ar & BOOST_SERIALIZATION_NVP(best_action_idx_cache_);
        }

    };

  } /* vi */

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_VI_TABULAR_ESTIMATOR_H_ */
