#ifndef UTEXAS_PLANNING_VISUALIZER_H
#define UTEXAS_PLANNING_VISUALIZER_H

#include <utexas_planning/core/state.h>

namespace utexas_planning {

  class Visualizer {

    public:

      typedef boost::shared_ptr<Visualizer> Ptr;
      typedef boost::shared_ptr<const Visualizer> ConstPtr;

      virtual ~Visualizer();
      virtual void init(int argc, char* argv[]) = 0;
      virtual void startEpisode(const State::ConstPtr& start_state) = 0;
      virtual void updateState(const State::ConstPtr& state, float timeout = 0.0f) = 0;
      virtual void exec() = 0;

  };

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_VISUALIZER_H */
