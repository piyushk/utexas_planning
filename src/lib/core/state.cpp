#include <boost/serialization/export.hpp>

#include <utexas_planning/core/state.h>

namespace utexas_planning {

  State::~State() {}

} /* utexas_planning */

std::ostream& operator<<(std::ostream& stream, const utexas_planning::State& s) {
  s.serialize(stream);
  return stream;
}

BOOST_SERIALIZATION_ASSUME_ABSTRACT(utexas_planning::State);
