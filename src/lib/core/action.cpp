#include <boost/serialization/export.hpp>
#include <stdexcept>
#include <utexas_planning/core/action.h>
#include <utexas_planning/common/exceptions.h>

namespace utexas_planning {

  Action::~Action() {}

  bool Action::operator<(const Action& other) const {
    throw UnimplementedFunctionException(getName(), "operator<()");
  }

  bool Action::operator==(const Action& other) const {
    throw UnimplementedFunctionException(getName(), "operator==()");
  }

  std::size_t Action::hash() const {
    throw UnimplementedFunctionException(getName(), "hash()");
  }

  std::string Action::getName() const {
    return typeid(*this).name();
  };

  std::size_t ActionHasher::operator() (const Action& action) const {
    return action.hash(); // Call polymorphic version.
  }

} /* utexas_planning */

std::ostream& operator<<(std::ostream& stream, const utexas_planning::Action& a) {
  a.serialize(stream); // Call polymorphic version.
  return stream;
}

BOOST_SERIALIZATION_ASSUME_ABSTRACT(utexas_planning::Action);
