#ifndef UTEXAS_PLANNING_VI_TABULAR_ESTIMATOR_H_
#define UTEXAS_PLANNING_VI_TABULAR_ESTIMATOR_H_

#include <map>

#include <boost/serialization/map.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/utility.hpp>

#include <utexas_planning/planners/vi/estimator.h>

namespace utexas_planning {

  namespace vi {

    class TabularEstimator : public Estimator {

      public:

        struct LessState : public std::binary_function<const State*, const State*, bool> {
          bool operator() (const State *lhs, const State *rhs) const;
        };

        typedef std::map<boost::shared_ptr<State>, std::pair<float, boost::shared_ptr<Action> >, LessState> EstimatorTable;

        virtual ~TabularEstimator ();
        virtual void getValueAndBestAction(const State& state, float &value, boost::shared_ptr<const Action> &action) const;
        virtual void setValueAndBestAction(const State& state,
                                           float value,
                                           const boost::shared_ptr<const Action> &action = boost::shared_ptr<const Action>());
        virtual void loadEstimatedValues(const std::string& file);
        virtual void saveEstimatedValues(const std::string& file) const;

      private:

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
