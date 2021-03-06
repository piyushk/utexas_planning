#include <boost/serialization/export.hpp>
#include <stdexcept>
#include <utexas_planning/core/state.h>
#include <utexas_planning/common/exceptions.h>

namespace utexas_planning {

  State::~State() {}

  bool State::operator<(const State& other) const {
    throw UnimplementedFunctionException(getName(), "operator<()");
  }

  bool State::operator==(const State& other) const {
    throw UnimplementedFunctionException(getName(), "operator==()");
  }

  std::size_t State::hash() const {
    throw UnimplementedFunctionException(getName(), "hash()");
  }

  std::string State::getName() const {
    return typeid(*this).name();
  };

  State::Ptr State::clone() const {
    return cloneImpl();
  }

  std::map<std::string, std::string> State::asMap() const {
    return std::map<std::string, std::string>();
  }

  std::size_t StateHasher::operator() (const State& state) const {
    return state.hash(); // Call polymorphic version.
  }

  bool State::PtrComparator::operator() (const State::ConstPtr& lhs, const State::ConstPtr& rhs) const {
    return (*lhs < *rhs);
  }


} /* utexas_planning */

std::ostream& operator<<(std::ostream& stream, const utexas_planning::State& s) {
  s.serialize(stream); // Call polymorphic version.
  return stream;
}

BOOST_SERIALIZATION_ASSUME_ABSTRACT(utexas_planning::State);
