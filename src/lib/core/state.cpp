#include <boost/serialization/export.hpp>
#include <stdexcept>
#include <utexas_planning/core/state.h>

namespace utexas_planning {

  State::~State() {}

  bool State::operator<(const State &other) const {
    throw std::runtime_error("State derivative does not implement operator<, but it is being used by a solver.");
  }

  bool State::operator==(const State &other) const {
    throw std::runtime_error("State derivative does not implement operator==, but it is being used by a solver.");
  }

  std::size_t State::hash() const {
    throw std::runtime_error("State derivative does not implement hash(), but it is being used by a solver.");
  }

  std::size_t StateHasher::operator() (const State& state) const {
    return state.hash(); // Call polymorphic version.
  }

} /* utexas_planning */

std::ostream& operator<<(std::ostream& stream, const utexas_planning::State& s) {
  s.serialize(stream); // Call polymorphic version.
  return stream;
}

BOOST_SERIALIZATION_ASSUME_ABSTRACT(utexas_planning::State);
