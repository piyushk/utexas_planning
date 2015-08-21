#ifndef TIMER_H
#define TIMER_H

#define USEC_PER_SEC (1000000)

#include <iosfwd>

namespace rddl {

  class Timer {
  public:
      Timer();
      ~Timer() {}
      void reset();
      double operator()() const;
      double getCurrentTime() const;

  private:
      double currentTime;
  };

} /* rddl */

std::ostream& operator<<(std::ostream& os, const rddl::Timer& timer);

#endif
