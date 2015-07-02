#ifndef UTEXAS_PLANNING_VI_ESTIMATOR_H_
#define UTEXAS_PLANNING_VI_ESTIMATOR_H_

#include <string>

#include <utexas_planning/common/Action.h>
#include <utexas_planning/common/State.h>

namespace utexas_planning {

  namespace vi {

    class Estimator {
      public:
        virtual ~Estimator () {}

        virtual void getValueAndBestAction(const State& state, float &value, Action& action) const = 0;
        virtual void setValueAndBestAction(const State& state, float value, const Action& action) = 0;

        virtual void saveEstimatedValues(const std::string& file) const = 0;
        virtual void loadEstimatedValues(const std::string& file) = 0;
    };

  } /* vi */

} /* utexas_planning */


#endif /* end of include guard: UTEXAS_PLANNING_VI_ESTIMATOR_H_ */
