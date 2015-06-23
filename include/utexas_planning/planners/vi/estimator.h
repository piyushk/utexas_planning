#ifndef UTEXAS_PLANNING_VI_ESTIMATOR_H_
#define UTEXAS_PLANNING_VI_ESTIMATOR_H_

#include <string>

#include <utexas_planning/common/Action.h>
#include <utexas_planning/common/State.h>

namespace utexas_planning {
  
  namespace vi {

    class Estimator {
      public:
        Estimator () {}
        virtual ~Estimator () {}

        virtual void getValueAndBestActionIdx(unsigned int state_idx, float &value, unsigned int &action_idx) const = 0;
        virtual void setValueAndBestActionIdx(unsigned int state_idx, float value, unsigned int action_idx) = 0;

        virtual void saveEstimatedValues(const std::string& file) const = 0;
        virtual void loadEstimatedValues(const std::string& file) = 0;
    };
    
  } /* vi */

} /* utexas_planning */


#endif /* end of include guard: UTEXAS_PLANNING_VI_ESTIMATOR_H_ */
