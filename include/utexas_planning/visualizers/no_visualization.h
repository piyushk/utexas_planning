#ifndef UTEXAS_PLANNING_NO_VISUALIZATION_H
#define UTEXAS_PLANNING_NO_VISUALIZATION_H

#include <utexas_planning/core/visualizer.h>

namespace utexas_planning {

  class NoVisualization : public Visualizer {

    public:

      typedef boost::shared_ptr<NoVisualization> Ptr;
      typedef boost::shared_ptr<const NoVisualization> ConstPtr;

      virtual ~NoVisualization();
      virtual void init(int argc, char* argv[]);
      virtual void startEpisode(const State::ConstPtr& start_state);
      virtual void updateState(const State::ConstPtr& state, float timeout = 0.0f);

  };
  
} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_NO_VISUALIZATION_H */
