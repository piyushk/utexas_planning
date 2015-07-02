#include <string>
#include <stdexcept>

#include <utexas_planning/core/declarative_model.h>

namespace utexas_planning {

  DeclarativeModel::~DeclarativeModel() {}

  void DeclarativeModel::getStateVector(const boost::shared_array<const State> &states, unsigned int &num_states) const {
    throw std::runtime_error("DeclarativeModel " + getModelName() + " does not support state enumeration. " +
                             "Perhaps you forgot to implement the getStateVector function in the model?");
  }

} /* utexas_planning */
