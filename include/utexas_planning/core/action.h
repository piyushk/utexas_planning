#ifndef UTEXAS_PLANNING_ACTION_H_
#define UTEXAS_PLANNING_ACTION_H_

#include <boost/serialization/serialization.hpp>
#include <boost/shared_ptr.hpp>

namespace utexas_planning {

  class Action
  {
    public:

      typedef boost::shared_ptr<Action> Ptr;
      typedef boost::shared_ptr<const Action> ConstPtr;

      virtual ~Action() {}

    private:

      friend class boost::serialization::access;
      template<class Archive> void serialize(Archive & ar, const unsigned int version) {
      }

  };

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_ACTION_H_ */
