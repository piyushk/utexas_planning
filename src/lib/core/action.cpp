#include <boost/serialization/export.hpp>
#include <stdexcept>
#include <utexas_planning/core/action.h>

namespace utexas_planning {

  Action::~Action() {}

  bool Action::operator<(const Action &other) const {
    throw std::runtime_error("Action derivative does not implement operator<, but it is being used by a solver.");
  }

  bool Action::operator==(const Action &other) const {
    throw std::runtime_error("Action derivative does not implement operator==, but it is being used by a solver.");
  }

  std::size_t Action::hash() const {
    throw std::runtime_error("Action derivative does not implement hash(), but it is being used by a solver.");
  }

  std::size_t ActionHasher::operator() (const Action& action) const {
    return action.hash(); // Call polymorphic version.
  }

} /* utexas_planning */

std::ostream& operator<<(std::ostream& stream, const utexas_planning::Action& a) {
  a.serialize(stream); // Call polymorphic version.
  return stream;
}

BOOST_SERIALIZATION_ASSUME_ABSTRACT(utexas_planning::Action);
