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

        typedef boost::shared_ptr<TabularEstimator> Ptr;
        typedef boost::shared_ptr<const TabularEstimator> ConstPtr;

        struct LessState : public std::binary_function<const State::ConstPtr, const State::ConstPtr, bool> {
          bool operator() (const State::ConstPtr &lhs, const State::ConstPtr &rhs) const;
        };

        typedef std::map<State::ConstPtr, std::pair<float, Action::ConstPtr>, LessState> EstimatorTable;

        virtual ~TabularEstimator ();

        virtual float getValue(const State::ConstPtr &state) const;
        virtual Action::ConstPtr getBestAction(const State::ConstPtr &state) const;

        virtual void setValueAndBestAction(const State::ConstPtr &state,
                                           float value,
                                           const Action::ConstPtr &action = Action::ConstPtr());

        virtual void saveEstimatedValues(const std::string& file) const;
        virtual void loadEstimatedValues(const std::string& file);

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
