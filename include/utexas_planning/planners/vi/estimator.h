#ifndef UTEXAS_PLANNING_VI_ESTIMATOR_H_
#define UTEXAS_PLANNING_VI_ESTIMATOR_H_

#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>

#include <utexas_planning/core/action.h>
#include <utexas_planning/core/state.h>

namespace utexas_planning {

  namespace vi {

    class Estimator {
      public:
        virtual ~Estimator ();

        virtual void getValueAndBestAction(const boost::shared_ptr<const State> &state,
                                           float &value,
                                           boost::shared_ptr<const Action> &action) const = 0;

        virtual void setValueAndBestAction(const boost::shared_ptr<const State> &state,
                                           float value,
                                           const boost::shared_ptr<const Action> &action = boost::shared_ptr<const Action>()) = 0;

        virtual void saveEstimatedValues(const std::string& file) const = 0;
        virtual void loadEstimatedValues(const std::string& file) = 0;
    };

  } /* vi */

} /* utexas_planning */


#endif /* end of include guard: UTEXAS_PLANNING_VI_ESTIMATOR_H_ */
