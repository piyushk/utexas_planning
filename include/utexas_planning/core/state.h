#ifndef UTEXAS_PLANNING_STATE_H_
#define UTEXAS_PLANNING_STATE_H_

#include <boost/serialization/serialization.hpp>
#include <boost/shared_ptr.hpp>

namespace utexas_planning {

  class State
  {
    public:

      typedef boost::shared_ptr<State> Ptr;
      typedef boost::shared_ptr<const State> ConstPtr;

      virtual ~State() {}

      virtual bool operator<(const State &other) const = 0;

    private:

      friend class boost::serialization::access;
      template<class Archive> void serialize(Archive & ar, const unsigned int version) {
      }

  };

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_STATE_H_ */
