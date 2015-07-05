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

        typedef boost::shared_ptr<Estimator> Ptr;
        typedef boost::shared_ptr<const Estimator> ConstPtr;

        virtual ~Estimator ();

        virtual void getValueAndBestAction(const State::ConstPtr &state,
                                           float &value,
                                           Action::ConstPtr &action) const = 0;

        virtual void setValueAndBestAction(const State::ConstPtr &state,
                                           float value,
                                           const Action::ConstPtr &action = Action::ConstPtr()) = 0;

        virtual void saveEstimatedValues(const std::string& file) const = 0;
        virtual void loadEstimatedValues(const std::string& file) = 0;
    };

  } /* vi */

} /* utexas_planning */


#endif /* end of include guard: UTEXAS_PLANNING_VI_ESTIMATOR_H_ */
