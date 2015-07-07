#ifndef UTEXAS_PLANNING_GENERATIVE_MODEL_H_
#define UTEXAS_PLANNING_GENERATIVE_MODEL_H_

#include <boost/shared_ptr.hpp>
#include <map>
#include <string>

#include <utexas_planning/common/rng.h>
#include <utexas_planning/core/action.h>
#include <utexas_planning/core/state.h>

namespace utexas_planning {

  class GenerativeModel {

    public:
      typedef boost::shared_ptr<GenerativeModel> Ptr;
      typedef boost::shared_ptr<const GenerativeModel> ConstPtr;

      virtual ~GenerativeModel ();

      virtual std::string getName() const;

      virtual bool isTerminalState(const State::ConstPtr &state) const = 0;

      virtual void takeAction(const State::ConstPtr &state,
                              const Action::ConstPtr &action,
                              float &reward,
                              State::ConstPtr &next_state,
                              int &depth_count,
                              boost::shared_ptr<RNG> rng) const = 0;

      virtual void getActionsAtState(const State::ConstPtr &state,
                                     std::vector<Action::ConstPtr> &actions) const = 0;

      virtual std::map<std::string, std::string> getParamsAsMap() const;

  };

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_GENERATIVE_MODEL_H_ */

