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

      virtual ~Action();

      virtual bool operator<(const Action &other) const;
      virtual bool operator==(const Action &other) const;
      virtual std::size_t hash() const;

      virtual void serialize(std::ostream &stream) const = 0;
      virtual std::string getName() const;

    private:

      friend class boost::serialization::access;
      template<class Archive> void serialize(Archive & ar, const unsigned int version) {
      }

  };

  struct ActionHasher {
    std::size_t operator()(const Action& action) const;
  };

} /* utexas_planning */

std::ostream& operator<<(std::ostream& stream, const utexas_planning::Action& a);
#endif /* end of include guard: UTEXAS_PLANNING_ACTION_H_ */
