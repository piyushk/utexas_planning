#ifndef UTEXAS_PLANNING_DECLARATIVE_MODEL_H_
#define UTEXAS_PLANNING_DECLARATIVE_MODEL_H_

#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>

#include <utexas_planning/core/action.h>
#include <utexas_planning/core/generative_model.h>
#include <utexas_planning/core/state.h>

namespace utexas_planning {

  class DeclarativeModel : public GenerativeModel {

    public:
      typedef boost::shared_ptr<DeclarativeModel> Ptr;
      typedef boost::shared_ptr<DeclarativeModel const> ConstPtr;

      virtual ~DeclarativeModel ();

      virtual const std::vector<State::ConstPtr>& getStateVector() const;

      virtual void getTransitionDynamics(const State::ConstPtr &state,
                                         const Action::ConstPtr &action,
                                         std::vector<State::ConstPtr> &next_states,
                                         std::vector<float> &rewards,
                                         std::vector<float> &probabilities) const = 0;

      virtual void takeAction(const State::ConstPtr &state,
                              const Action::ConstPtr &action,
                              float &reward,
                              State::ConstPtr &next_state,
                              int &depth_count,
                              boost::shared_ptr<RNG> rng) const;

  };

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_DECLARATIVE_MODEL_H_ */

