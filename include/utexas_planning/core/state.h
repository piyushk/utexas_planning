#ifndef UTEXAS_PLANNING_STATE_H_
#define UTEXAS_PLANNING_STATE_H_

#include <boost/serialization/serialization.hpp>

/* Included here so that BOOST_CLASS_EXPORT works as expected. */
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <boost/shared_ptr.hpp>
#include <iostream>
#include <map>

namespace utexas_planning {

  class State
  {
    public:

      typedef boost::shared_ptr<State> Ptr;
      typedef boost::shared_ptr<const State> ConstPtr;

      virtual ~State();

      virtual bool operator<(const State& other) const;
      virtual bool operator==(const State& other) const;
      virtual std::size_t hash() const;

      virtual void serialize(std::ostream& stream) const = 0;
      virtual std::string getName() const;

      State::Ptr clone() const;

      virtual std::map<std::string, std::string> asMap() const;

    private:

      virtual State::Ptr cloneImpl() const = 0;

      friend class boost::serialization::access;
      template<class Archive> void serialize(Archive&  ar, const unsigned int version) {
      }

  };

  struct StateHasher {
    std::size_t operator()(const State& state) const;
  };

} /* utexas_planning */

std::ostream& operator<<(std::ostream& stream, const utexas_planning::State& s);
#endif /* end of include guard: UTEXAS_PLANNING_STATE_H_ */
