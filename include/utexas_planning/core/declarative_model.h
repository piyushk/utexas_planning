#ifndef UTEXAS_PLANNING_DECLARATIVE_MODEL_H_
#define UTEXAS_PLANNING_DECLARATIVE_MODEL_H_

#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>

#include <utexas_planning/core/action.h>
#include <utexas_planning/core/generative_model.h>
#include <utexas_planning/core/state.h>

namespace utexas_planning {

  class DeclarativeModel : public Generative Model {

    public:
      typedef boost::shared_ptr<DeclarativeModel> Ptr;

      virtual ~DeclarativeModel ();

      virtual std::string getName() const = 0;

      virtual void getStateVector(boost::shared_array<const State> &states, unsigned int num_states) const;

      virtual void getTransitionDynamics(const State &state,
                                         const Action &action,
                                         boost::shared_array<State> &next_states,
                                         std::vector<float> &rewards,
                                         std::vector<float> &probabilities) const = 0;

  };

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_DECLARATIVE_MODEL_H_ */

