#ifndef UTEXAS_PLANNING_DECLARATIVE_MODEL_H_
#define UTEXAS_PLANNING_DECLARATIVE_MODEL_H_

#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>

#include <utexas_planning/common/rng.h>
#include <utexas_planning/core/action.h>
#include <utexas_planning/core/generative_model.h>
#include <utexas_planning/core/state.h>

namespace utexas_planning {

  class DeclarativeModel : public Generative Model {

    public:
      typedef boost::shared_ptr<DeclarativeModel> Ptr;

      DeclarativeModel () {}
      virtual ~DeclarativeModel () {}

      virtual std::string getName() const = 0;

      virtual void getStateVector(boost::shared_array<const State> &states, unsigned int num_states) const {
        throw std::runtime_error("DeclarativeModel " + getName() + " does not support state enumeration. " +
                                 "Perhaps you forgot to implement the getStateVector function in the model?");
      }

      virtual void getTransitionDynamics(const State &state,
                                         const Action &action,
                                         boost::shared_array<State> &next_states,
                                         std::vector<float> &rewards,
                                         std::vector<float> &probabilities) = 0 const;

  };

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_DECLARATIVE_MODEL_H_ */

