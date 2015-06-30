#ifndef UTEXAS_PLANNING_GENERATIVE_MODEL_H_
#define UTEXAS_PLANNING_GENERATIVE_MODEL_H_

#include <boost/shared_array.hpp>
#include <boost/shared_ptr.hpp>
#include <string>

#include <utexas_planning/common/rng.h>
#include <utexas_planning/core/action.h>
#include <utexas_planning/core/state.h>

namespace utexas_planning {

  class GenerativeModel {

    public:
      typedef boost::shared_ptr<GenerativeModel> Ptr;

      virtual ~GenerativeModel ();

      virtual bool isTerminalState(const State &state) const = 0;

      virtual void takeAction(const State &state,
                              const Action &action,
                              float &reward,
                              State &next_state,
                              bool &terminal,
                              int &depth_count,
                              boost::shared_ptr<RNG> rng) const = 0;

      virtual void getActionsAtState(const State &state,
                                     boost::shared_array<Action>& actions,
                                     int &num_actions) const = 0;
  };

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_GENERATIVE_MODEL_H_ */

